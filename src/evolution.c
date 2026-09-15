/**
 * @file evolution.c
 * @brief Implementation of the OpenAI Evolution Strategy and FMU co-simulation.
 */

#include "evolution.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- MAXIMUM PHYSICAL LIMITS ---
// --- VALUE REFERENCES FROM XML ---
#define VR_BATTERY 14
#define VR_UX 18  
#define VR_UY 19
#define VR_UZ 20
#define VR_X  25   
#define VR_Y  26
#define VR_Z  27
#define VR_VX 22
#define VR_START_X 35
#define VR_START_Y 36 
#define VR_START_Z 37 
// Maximum engine thrust in Newtons.
#define MAX_THRUST 20.0 

// ============================================================================
// MATHEMATICAL UTILITIES
// ============================================================================

/**
 * @brief Generates a random number from a Standard Normal Distribution N(0, 1).
 *
 * Uses the Box-Muller transform to convert uniformly distributed random numbers
 * into normally distributed ones. Essential for generating Gaussian noise 
 * during the Evolution Strategy weight jittering phase.
 *
 * @return A random double drawn from a Gaussian distribution with mean 0 and std 1.
 */
static double randn(void) {
    double u1 = ((double)rand() / RAND_MAX);
    double u2 = ((double)rand() / RAND_MAX);
    if (u1 <= 1e-7) u1 = 1e-7; // Prevent log(0)
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

// ============================================================================
// SIMULATION ENGINE AND FITNESS FUNCTION
// ============================================================================

/**
 * @brief Executes a single simulation episode for a specific Neural Network across multiple drones.
 *
 * Initializes the FMUs, runs the simulation loop step-by-step, calculates LIDAR rays,
 * feeds them to the Neural Network, and applies the output thrust back to the FMU.
 * It accumulates fitness based on distance covered, energy used, and collisions.
 *
 * @param fmus Array of loaded FMU instances.
 * @param start_x Array of initial X coordinates.
 * @param start_y Array of initial Y coordinates.
 * @param start_z Array of initial Z coordinates.
 * @param nn Pointer to the Neural Network variant being evaluated.
 * @param telemetry_csv Optional FILE pointer to record telemetry (NULL during training).
 * @return The average fitness obtained by the active drones in this episode.
 */
static double run_episode(fmi2_import_t* fmus[N_DRONES], 
                          double start_x[N_DRONES], double start_y[N_DRONES], double start_z[N_DRONES], 
                          NeuralNetwork* nn, FILE* telemetry_csv) {
    
    double t_start = 0.0;
    double t_end = 300.0;     /**< Maximum simulation time: 5 minutes. */
    double step_size = 0.05;  /**< Solver resolution: 50 milliseconds per tick. */
    
    int active[N_DRONES];
    double fitnesses[N_DRONES];
    double previous_distances[N_DRONES];
    double current_x[N_DRONES];
    double current_y[N_DRONES];
    double current_z[N_DRONES];
    double current_battery[N_DRONES];
    
    // 1. INITIALIZE ALL ACTIVE DRONES
    for (int d = 0; d < num_active_drones; d++) {
        fmi2_import_setup_experiment(fmus[d], fmi2_true, 1e-4, t_start, fmi2_true, t_end);
        
        fmi2_value_reference_t init_vr[3] = {VR_START_X, VR_START_Y, VR_START_Z};
        double init_val[3] = {start_x[d], start_y[d], start_z[d]};
        fmi2_import_set_real(fmus[d], init_vr, 3, init_val);
        
        fmi2_import_enter_initialization_mode(fmus[d]);
        fmi2_import_exit_initialization_mode(fmus[d]);
        
        active[d] = 1;
        fitnesses[d] = 0.0;
        
        // Calculate initial distance to target
        double dx = target_x[d] - start_x[d];
        double dy = target_y[d] - start_y[d];
        double dz = target_z[d] - start_z[d];
        previous_distances[d] = sqrt(dx*dx + dy*dy + dz*dz);
        
        current_x[d] = start_x[d];
        current_y[d] = start_y[d];
        current_z[d] = start_z[d];
        current_battery[d] = 100.0; // Battery starts full
    }

    // Disable inactive drones explicitly
    for (int d = num_active_drones; d < N_DRONES; d++) {
        active[d] = 0;
        fitnesses[d] = 0.0;
    }
    
    double current_time = t_start;
    fmi2_value_reference_t out_vr[4] = {VR_X, VR_Y, VR_Z, VR_BATTERY}; // x, y, z, battery
    fmi2_value_reference_t in_vr[3] = {VR_UX, VR_UY, VR_UZ};     // ux, uy, uz
    
    int any_crashed = 0;
    
    // 2. MAIN CO-SIMULATION LOOP
    while (current_time < t_end && !any_crashed) {
        
        int all_done = 1;
        
        // Write Telemetry for ALL drones, active or not
        if (telemetry_csv != NULL) {
            fprintf(telemetry_csv, "%f", current_time);
            for (int d = 0; d < num_active_drones; d++) {
                fprintf(telemetry_csv, ",%f,%f,%f,%f", current_x[d], current_y[d], current_z[d], current_battery[d]);
            }
        }
        
        for (int d = 0; d < num_active_drones; d++) {
            if (!active[d]) continue;
            all_done = 0;
            
            // Extract current physical state from the FMU solver
            double out_val[4];
            fmi2_import_get_real(fmus[d], out_vr, 4, out_val);
            current_x[d] = out_val[0];
            current_y[d] = out_val[1];
            current_z[d] = out_val[2];
            current_battery[d] = out_val[3];
            
            // Raycasting simulation: shoot LIDAR rays from the drone
            double lidar_distances[NUM_RAYS];
            compute_lidar_rays(d, current_x[d], current_y[d], current_z[d], current_x, current_y, current_z, active, lidar_distances);
            
            // Collision Detection
            int collision = 0;
            for (int i = 0; i < NUM_RAYS; i++) {
                if (lidar_distances[i] < DRONE_RADIUS * 1.5) {
                    collision = 1;
                    break;
                }
            }
            
            // FITNESS CALCULATION
            double dx = target_x[d] - current_x[d];
            double dy = target_y[d] - current_y[d];
            double dz = target_z[d] - current_z[d];
            double current_distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            // Reward: Closing the distance to the target
            double progress = previous_distances[d] - current_distance;
            fitnesses[d] += progress * 100.0;
            previous_distances[d] = current_distance;
            
            // Penalty: Collision detected
            if (collision) {
                fitnesses[d] -= 500.0;
                active[d] = 0;
#if ENABLE_TEAM_CRASH
                any_crashed = 1; // Team crash!
#endif
                continue;
            }
            
            // Penalty: Battery completely depleted
            if (current_battery[d] <= 0.0) {
                fitnesses[d] -= 1000.0;
                active[d] = 0;
#if ENABLE_TEAM_CRASH
                any_crashed = 1; // Team crash!
#endif
                continue;
            }
            
            // Big Reward: Reached the target
            if (current_distance < 5.0) {
                // Flat bonus + efficiency bonus for remaining battery
                fitnesses[d] += 1000.0 + (current_battery[d] * 50.0);
                active[d] = 0;
                continue;
            }
            
            // PREPARE NEURAL NETWORK INPUTS
            double nn_inputs[NN_INPUT_SIZE];
            for (int i = 0; i < NUM_RAYS; i++) {
                nn_inputs[i] = lidar_distances[i] / MAX_LIDAR_RANGE; // Normalize [0, 1]
            }
            
            // Provide normalized directional vector towards target (Versor)
            if (current_distance > 0.001) {
                nn_inputs[64] = dx / current_distance;
                nn_inputs[65] = dy / current_distance;
                nn_inputs[66] = dz / current_distance;
            } else {
                nn_inputs[64] = 0.0;
                nn_inputs[65] = 0.0;
                nn_inputs[66] = 0.0;
            }
            
            // Provide normalized battery level [0, 1]
            nn_inputs[67] = current_battery[d] / 100.0;
            
            double nn_outputs[NN_OUTPUT_SIZE];
            
            // Perform the inference pass
            nn_feedforward(nn, nn_inputs, nn_outputs);
            
            // Regularization Penalty: Discourage erratic or maximum continuous thrust (efficiency)
            double effort = (nn_outputs[0]*nn_outputs[0] + nn_outputs[1]*nn_outputs[1] + nn_outputs[2]*nn_outputs[2]);
            fitnesses[d] -= effort * 0.01;
            
            // Apply Neural Output to Physical Engine (scaled by max engine strength)
            double input_values[3];
            input_values[0] = nn_outputs[0] * MAX_THRUST;
            input_values[1] = nn_outputs[1] * MAX_THRUST;
            input_values[2] = nn_outputs[2] * MAX_THRUST;
            
            fmi2_import_set_real(fmus[d], in_vr, 3, input_values);
        }
        
        if (telemetry_csv != NULL) fprintf(telemetry_csv, "\n");
        if (all_done) break;
        
        // Advance FMU integration step
        for (int d = 0; d < num_active_drones; d++) {
            if (active[d]) {
                fmi2_import_do_step(fmus[d], current_time, step_size, fmi2_true);
            }
        }
        
        current_time += step_size;
    }
    
    // 3. CLEANUP AND TERMINATION
    double total_fitness = 0.0;
    for (int d = 0; d < num_active_drones; d++) {
        fmi2_import_terminate(fmus[d]);
        fmi2_import_reset(fmus[d]); // CRITICAL: Resets FMU memory state for next episode
        total_fitness += fitnesses[d];
    }
    
    // Normalize fitness by the number of active drones to stabilize the algorithm 
    // across random environments with varying drone density
    return total_fitness / num_active_drones; 
}

// ============================================================================
// TRAINING FUNCTION (Evolution Strategy)
// ============================================================================

void es_train(fmi2_import_t* fmus[N_DRONES]) {
    srand(42); 

    NeuralNetwork base_nn;
    nn_init_random(&base_nn);

    int num_params = sizeof(NeuralNetwork) / sizeof(double);
    printf("Starting ES Optimization | %d Neural Parameters | Population: %d\n\n", num_params, POPULATION_SIZE);

    // Allocates arrays for noise matrices, perturbed cloned networks, and fitness scores
    NeuralNetwork* epsilons = malloc(POPULATION_SIZE * sizeof(NeuralNetwork));
    NeuralNetwork* perturbed_nns = malloc(POPULATION_SIZE * sizeof(NeuralNetwork));
    double* fitnesses = malloc(POPULATION_SIZE * sizeof(double));

    FILE* fitness_csv = fopen("fitness.csv", "w");
    if (fitness_csv != NULL) {
        fprintf(fitness_csv, "Generation,AverageFitness,MaxFitness\n");
    }

    FILE* time_csv = fopen("generation_times.csv", "w");
    if (time_csv != NULL) {
        fprintf(time_csv, "Generation,TimeSeconds\n");
    }

    // MAIN GENERATIONAL LOOP
    for (int gen = 0; gen < GENERATIONS; gen++) {
        clock_t gen_start = clock();
         
        double total_fitness = 0.0;
        double max_fitness = -9999.0;

        // FASE DI CAMPIONAMENTO MONTE CARLO
        for (int p = 0; p < POPULATION_SIZE; p++) {
            
            double* base_ptr = (double*)&base_nn;
            double* eps_ptr = (double*)&epsilons[p];
            double* pert_ptr = (double*)&perturbed_nns[p];

            // Noise generation for each single neural weight
            for (int i = 0; i < num_params; i++) {
                eps_ptr[i] = randn(); // Draw from standard normal distribution
                
                // Creation of the perturbed network variant: W_variant = W + (Sigma * Epsilon)
                pert_ptr[i] = base_ptr[i] + SIGMA * eps_ptr[i];
            }

            double clone_total_fitness = 0.0;
            
            // Evaluate clone across multiple procedural environments (M Simulations)
            for (int m = 0; m < MONTECARLO_SAMPLES; m++) {
                unsigned int env_seed = 42 + (gen * MONTECARLO_SAMPLES) + m;
                double start_x[N_DRONES], start_y[N_DRONES], start_z[N_DRONES];
                
                generate_random_environment(env_seed, start_x, start_y, start_z);
                
                // Export only the first environment topology for Python visualization
                if (p == 0 && m == 0) {
                    export_environment("environment.csv"); 
                }

                clone_total_fitness += run_episode(fmus, start_x, start_y, start_z, &perturbed_nns[p], NULL);
            }
            
            // The score of this clone is the average reward over the M simulations
            fitnesses[p] = clone_total_fitness / MONTECARLO_SAMPLES;
            
            total_fitness += fitnesses[p];
            if (fitnesses[p] > max_fitness) max_fitness = fitnesses[p];
        }

        // ====================================================================
        // Z-SCORE FITNESS NORMALIZATION (Crucial for ES stability)
        // Transforms the raw returns into a distribution with mean 0 and std 1.
        // ====================================================================
        double mean_fitness = total_fitness / POPULATION_SIZE;

        double std_fitness = 0.0;
        for (int p = 0; p < POPULATION_SIZE; p++) {
            std_fitness += (fitnesses[p] - mean_fitness) * (fitnesses[p] - mean_fitness);
        }
        std_fitness = sqrt(std_fitness / POPULATION_SIZE);
        if (std_fitness < 1e-8) std_fitness = 1e-8; // Prevent division by zero

        for (int p = 0; p < POPULATION_SIZE; p++) {
            fitnesses[p] = (fitnesses[p] - mean_fitness) / std_fitness; 
        }
        // ====================================================================

        // FASE DI AGGIORNAMENTO DEI PESI PRINCIPALI
        double* base_ptr = (double*)&base_nn;
        for (int i = 0; i < num_params; i++) {
            double gradient_estimate = 0.0;
            
            // For each parameter 'i', the estimated gradient is the weighted sum 
            // of the noise (epsilon) multiplied by the Z-Score fitness obtained by that clone.
            for (int p = 0; p < POPULATION_SIZE; p++) {
                double* eps_ptr = (double*)&epsilons[p];
                gradient_estimate += fitnesses[p] * eps_ptr[i];
            }
            
            // Stochastic Gradient Descent Update Formula:
            // W = W + Alpha * (Gradient_Estimate / (POPULATION_SIZE * Sigma))
            base_ptr[i] += LEARNING_RATE * gradient_estimate / (POPULATION_SIZE * SIGMA);
        }

        printf("Generation %03d | Average Fitness: %8.2f | Max: %8.2f ", gen, total_fitness / POPULATION_SIZE, max_fitness);
        
        if (fitness_csv != NULL) {
            fprintf(fitness_csv, "%d,%f,%f\n", gen, total_fitness / POPULATION_SIZE, max_fitness);
            fflush(fitness_csv);
        }

        // FASE DI ESPORTAZIONE TELEMETRIA
        // Performs an extra rollout using the updated MASTER network (no noise) to record 
        // the swarm's actual flight path for visual evaluation in Python.
        
        unsigned int telemetry_seed = 42 ; 
        double tel_start_x[N_DRONES], tel_start_y[N_DRONES], tel_start_z[N_DRONES];
        generate_random_environment(telemetry_seed, tel_start_x, tel_start_y, tel_start_z);
        
        // We only evaluate fitness here, no need to overwrite telemetry.csv during training
        run_episode(fmus, tel_start_x, tel_start_y, tel_start_z, &base_nn, NULL);
        
        if (time_csv != NULL) {
            clock_t gen_end = clock();
            double elapsed = (double)(gen_end - gen_start) / CLOCKS_PER_SEC;
            printf("Time: %f\n", elapsed);
            fprintf(time_csv, "%d,%f\n", gen, elapsed);
            fflush(time_csv);
        }
    }

    if (fitness_csv != NULL) fclose(fitness_csv);
    if (time_csv != NULL) fclose(time_csv);
    
    // Save the final master brain to disk
    nn_save(&base_nn, "best_model.bin");

    // Memory de-allocation
    free(epsilons);
    free(perturbed_nns);
    free(fitnesses);
}

// ============================================================================
// TESTING FUNCTION (Inference Only)
// ============================================================================

void es_test(fmi2_import_t* fmus[N_DRONES], const char* model_filename) {
    NeuralNetwork nn;
    
    // 1. Loads the pre-trained weights from the binary file
    if (!nn_load(&nn, model_filename)) {
        printf(">>> ERROR: Could not load %s. Please run training first!\n", model_filename);
        return;
    }
    
    printf("\n>>> PRE-TRAINED MODEL SUCCESSFULLY LOADED FROM %s <<<\n", model_filename);

    // 2. Generates a completely new map to test generalization
    unsigned int test_seed = 2000; 
    double start_x[N_DRONES], start_y[N_DRONES], start_z[N_DRONES];
    
    generate_random_environment(test_seed, start_x, start_y, start_z);
    export_environment("environment.csv"); // Exports for Python visualization

    printf("Test Map Generated (Seed: %u)\n", test_seed);
    for (int d = 0; d < N_DRONES; d++) {
        printf("Drone %d Target: X=%.2f, Y=%.2f, Z=%.2f\n", d, target_x[d], target_y[d], target_z[d]);
    }

    FILE* telemetry_csv = fopen("telemetry.csv", "w");
    if (telemetry_csv != NULL) {
        fprintf(telemetry_csv, "time"); 
        for (int d = 0; d < num_active_drones; d++) fprintf(telemetry_csv, ",x%d,y%d,z%d,bat%d", d, d, d, d);
        fprintf(telemetry_csv, "\n");
        
        printf("\n>>> STARTING INFERENCE FLIGHT...\n");
        double final_fitness = run_episode(fmus, start_x, start_y, start_z, &nn, telemetry_csv);
        
        fclose(telemetry_csv);
        printf(">>> FLIGHT COMPLETED. Final Fitness: %.2f\n", final_fitness);
        printf(">>> Run visualize.py to view the drone's trajectory in 3D.\n\n");
    }
}






