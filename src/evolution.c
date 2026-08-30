#include "evolution.h"
#include "environment.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

// --- VALUE REFERENCES FROM XML ---
#define VR_UX 12  
#define VR_UY 13
#define VR_UZ 14
#define VR_X  19   
#define VR_Y  20
#define VR_Z  21
#define VR_VX 16
#define VR_START_X 24
#define VR_START_Y 25 
#define VR_START_Z 26 
// Maximum engine thrust in Newtons. Multiplies the [-1, 1] network output.
#define MAX_THRUST 20.0 

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// MATHEMATICAL UTILITIES
// ============================================================================

// Box-Muller transform: converts uniform pseudo-random numbers (rand) 
// into a Standard Normal distribution (mean 0, variance 1).
// Essential for generating the noise matrices (Epsilon) for the ES.
static double randn(void) {
    double u1 = ((double)rand() / RAND_MAX);
    double u2 = ((double)rand() / RAND_MAX);
    if (u1 <= 1e-9) u1 = 1e-9; // Avoids log(0) which would cause a crash (NaN)
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

// ============================================================================
// SIMULATION ENGINE AND FITNESS FUNCTION
// ============================================================================

// Executes a single simulation episode for a specific Neural Network across multiple drones.
static double run_episode(fmi2_import_t* fmus[N_DRONES], double start_x[N_DRONES], double start_y[N_DRONES], double start_z[N_DRONES], NeuralNetwork* nn, FILE* csv_file) {
    
    double t_start = 0.0;
    double t_end = 60.0;  // The episode lasts a maximum of 60 virtual seconds
    double step_size = 0.05; // Solver resolution: 50 milliseconds
    
    int active[N_DRONES];
    double fitnesses[N_DRONES];
    double previous_distances[N_DRONES];
    double current_x[N_DRONES];
    double current_y[N_DRONES];
    double current_z[N_DRONES];

    fmi2_value_reference_t vr_starts[3] = { VR_START_X, VR_START_Y, VR_START_Z };
    fmi2_value_reference_t vr_inputs[3] = { VR_UX, VR_UY, VR_UZ };
    fmi2_value_reference_t vr_outputs[4] = { VR_X, VR_Y, VR_Z, VR_VX };
    
    for (int d = 0; d < num_active_drones; d++) {
        fmi2_import_setup_experiment(fmus[d], fmi2_true, 1e-4, t_start, fmi2_true, t_end);
        
        double start_vals[3] = { start_x[d], start_y[d], start_z[d] };
        fmi2_import_set_real(fmus[d], vr_starts, 3, start_vals);
        fmi2_import_enter_initialization_mode(fmus[d]);
        fmi2_import_exit_initialization_mode(fmus[d]);
        
        active[d] = 1;
        fitnesses[d] = 0.0;
        
        double dx = target_x[d] - start_x[d];
        double dy = target_y[d] - start_y[d];
        double dz = target_z[d] - start_z[d];
        previous_distances[d] = sqrt(dx*dx + dy*dy + dz*dz);
        current_x[d] = start_x[d];
        current_y[d] = start_y[d];
        current_z[d] = start_z[d];
    }
    
    // Disable inactive drones
    for (int d = num_active_drones; d < N_DRONES; d++) {
        active[d] = 0;
        fitnesses[d] = 0.0;
    }
    
    double current_time = t_start;
    
    while (current_time < t_end) {
        int any_active = 0;
        
        // 1. Read Positions
        for (int d = 0; d < N_DRONES; d++) {
            if (!active[d]) continue;
            any_active = 1;
            
            double output_values[4];
            fmi2_import_get_real(fmus[d], vr_outputs, 4, output_values);
            current_x[d] = output_values[0];
            current_y[d] = output_values[1];
            current_z[d] = output_values[2];
        }
        
        if (!any_active) break;
        
        if (csv_file != NULL) {
            fprintf(csv_file, "%.3f", current_time);
            for (int d = 0; d < N_DRONES; d++) {
                fprintf(csv_file, ",%.3f,%.3f,%.3f", current_x[d], current_y[d], current_z[d]);
            }
            fprintf(csv_file, "\n");
        }
        
        int any_crashed = 0;
        
        // 2. Compute Lidar and Process Neural Nets
        for (int d = 0; d < num_active_drones; d++) {
            if (!active[d]) continue;
            
            double lidar_distances[NUM_RAYS];
            compute_lidar_rays(d, current_x[d], current_y[d], current_z[d], current_x, current_y, current_z, active, lidar_distances);
            
            int collision = 0;
            double nn_inputs[NN_INPUT_SIZE];
            double nn_outputs[NN_OUTPUT_SIZE];
            
            for (int i = 0; i < NUM_RAYS; i++) {
                nn_inputs[i] = lidar_distances[i] / MAX_LIDAR_RANGE;
                if (lidar_distances[i] <= 0.5) collision = 1; 
            }
            
            double dx = target_x[d] - current_x[d];
            double dy = target_y[d] - current_y[d];
            double dz = target_z[d] - current_z[d];
            double current_distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            double progress = previous_distances[d] - current_distance;
            fitnesses[d] += progress * 100.0;
            previous_distances[d] = current_distance;
            
            if (collision) {
                fitnesses[d] -= 500.0;
                active[d] = 0;
#if ENABLE_TEAM_CRASH
                any_crashed = 1; // Team crash!
                break;
#else
                continue; // Independent training: just this drone fails
#endif
            }
            
            if (current_distance < 5.0) {
                fitnesses[d] += 1000.0 + (t_end - current_time) * 100.0;
                active[d] = 0;
                continue;
            }
            
            nn_inputs[64] = dx / 100.0;
            nn_inputs[65] = dy / 100.0;
            nn_inputs[66] = dz / 100.0;
            
            nn_feedforward(nn, nn_inputs, nn_outputs);
            
            double effort = (nn_outputs[0]*nn_outputs[0] + nn_outputs[1]*nn_outputs[1] + nn_outputs[2]*nn_outputs[2]);
            fitnesses[d] -= effort * 0.01;
            
            double input_values[3];
            input_values[0] = nn_outputs[0] * MAX_THRUST;
            input_values[1] = nn_outputs[1] * MAX_THRUST;
            input_values[2] = nn_outputs[2] * MAX_THRUST;
            
            fmi2_import_set_real(fmus[d], vr_inputs, 3, input_values);
            fmi2_import_do_step(fmus[d], current_time, step_size, fmi2_true);
        }
        
#if ENABLE_TEAM_CRASH
        if (any_crashed) break; // Terminate episode for all drones
#endif

        current_time += step_size;
    }
    
    double total_fitness = 0.0;
    for (int d = 0; d < num_active_drones; d++) {
        fmi2_import_terminate(fmus[d]);
        fmi2_import_reset(fmus[d]);
        total_fitness += fitnesses[d];
    }
    
    // Normalize fitness by the number of active drones to stabilize the algorithm 
    // across random environments with varying density
    return total_fitness / num_active_drones; 
}

// ============================================================================
// OPTIMIZATION ALGORITHM (OpenAI ES - Algorithm 1)
// ============================================================================

void es_train(fmi2_import_t* fmus[N_DRONES]) {
    
    // Initializes the Original Network (Master) with random weights
    NeuralNetwork base_nn;
    nn_init_random(&base_nn);
    
    // Treats the entire structure (Weights and Biases) as a single continuous block in RAM.
    // This pointer trick avoids complex nested loops.
    int num_params = sizeof(NeuralNetwork) / sizeof(double);
    printf("Starting ES Optimization | %d Neural Parameters | Population: %d\n\n", num_params, POPULATION_SIZE);

    // Allocates arrays for noise tensors, perturbed cloned networks, and fitnesses
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

    // GENERATIONAL LOOP
    for (int gen = 0; gen < GENERATIONS; gen++) {
        clock_t gen_start = clock();
         
        // A & B. Procedural Environmental Regeneration and Population Evaluation
        // Step 3 and 4 of OpenAI ES Algorithm 1

        double total_fitness = 0.0;
        double max_fitness = -9999.0;

        for (int p = 0; p < POPULATION_SIZE; p++) {
            
            double* base_ptr = (double*)&base_nn;
            double* eps_ptr = (double*)&epsilons[p];
            double* pert_ptr = (double*)&perturbed_nns[p];

            // Noise generation for each single neural weight
            for (int i = 0; i < num_params; i++) {
                eps_ptr[i] = randn(); // Extraction from N(0, I)
                
                // Creation of the perturbed network: Theta_perturbed = Theta_base + Sigma * Epsilon
                pert_ptr[i] = base_ptr[i] + SIGMA * eps_ptr[i];
            }

            double clone_total_fitness = 0.0;
            
            // Monte Carlo evaluation (M simulations)
            for (int m = 0; m < MONTECARLO_SAMPLES; m++) {
                unsigned int env_seed = 42 + (gen * MONTECARLO_SAMPLES) + m;
                double start_x[N_DRONES], start_y[N_DRONES], start_z[N_DRONES];
                
                // 1. Generate random environment
                generate_random_environment(env_seed, start_x, start_y, start_z);
                
                // Export only the first environment for visualization
                if (p == 0 && m == 0) {
                    export_environment("environment.csv"); 
                }

                // 2 & 3. Send clone into the simulator and accumulate fitness
                clone_total_fitness += run_episode(fmus, start_x, start_y, start_z, &perturbed_nns[p], NULL);
            }
            
            // 4. Calculate the average reward over M simulations
            fitnesses[p] = clone_total_fitness / MONTECARLO_SAMPLES;
            
            total_fitness += fitnesses[p];
            if (fitnesses[p] > max_fitness) max_fitness = fitnesses[p];
        }

        // ====================================================================
        // Z-SCORE FITNESS NORMALIZATION
        // Trasforma i ritorni in modo che abbiano media 0 e deviazione standard 1
        // ====================================================================
        double mean_fitness = 0.0;
        for (int p = 0; p < POPULATION_SIZE; p++) {
            mean_fitness += fitnesses[p];
        }
        mean_fitness /= POPULATION_SIZE;

        double std_fitness = 0.0;
        for (int p = 0; p < POPULATION_SIZE; p++) {
            std_fitness += (fitnesses[p] - mean_fitness) * (fitnesses[p] - mean_fitness);
        }
        std_fitness = sqrt(std_fitness / POPULATION_SIZE);
        if (std_fitness < 1e-8) std_fitness = 1e-8; // Previene divisione per zero

        for (int p = 0; p < POPULATION_SIZE; p++) {
            // Sostituisce la fitness grezza con il suo Z-Score
            fitnesses[p] = (fitnesses[p] - mean_fitness) / std_fitness; 
        }
        // ====================================================================

        // C. Master Weight Vector Update (Step 5 of OpenAI ES Algorithm 1)
        double* base_ptr = (double*)&base_nn;
        for (int i = 0; i < num_params; i++) {
            double gradient_estimate = 0.0;
            
            // For each parameter 'i', the gradient is the weighted sum of the noise (epsilon)
            // multiplied by the fitness obtained by that specific clone.
            // The networks that won (high fitness) will 'pull' the weights in their direction.
            for (int p = 0; p < POPULATION_SIZE; p++) {
                double* eps_ptr = (double*)&epsilons[p];
                gradient_estimate += fitnesses[p] * eps_ptr[i];
            }
            
            // Update formula: Theta_new = Theta_old + Alpha * [ (1 / n*Sigma) * Sum(F_i * Eps_i) ]
            base_ptr[i] += LEARNING_RATE * gradient_estimate / (POPULATION_SIZE * SIGMA);
        }

        printf("Generation %03d | Average Fitness: %8.2f | Max: %8.2f ", gen, total_fitness / POPULATION_SIZE, max_fitness);
        
        if (fitness_csv != NULL) {
            fprintf(fitness_csv, "%d,%f,%f\n", gen, total_fitness / POPULATION_SIZE, max_fitness);
            fflush(fitness_csv);
        }

        // D. Offline Telemetry Recording for Visual Evaluation
        // Performs an extra rollout with the MASTER network (no noise) to record what it has learned.
        
        unsigned int telemetry_seed = 42 ; 
        double tel_start_x[N_DRONES], tel_start_y[N_DRONES], tel_start_z[N_DRONES];
        generate_random_environment(telemetry_seed, tel_start_x, tel_start_y, tel_start_z);
        
        FILE* telemetry_csv = fopen("telemetry.csv", "w");
        if (telemetry_csv != NULL) {
            fprintf(telemetry_csv, "time"); 
            for (int d = 0; d < N_DRONES; d++) fprintf(telemetry_csv, ",x%d,y%d,z%d", d, d, d);
            fprintf(telemetry_csv, "\n");
            run_episode(fmus, tel_start_x, tel_start_y, tel_start_z, &base_nn, telemetry_csv);
            fclose(telemetry_csv);
        }
        
        if (time_csv != NULL) {
            clock_t gen_end = clock();
            double elapsed = (double)(gen_end - gen_start) / CLOCKS_PER_SEC;
            printf("Time: %f\n", elapsed);
            fprintf(time_csv, "%d,%f\n", gen, elapsed);
            fflush(time_csv);
        }
    }

    if (fitness_csv != NULL) {
        fclose(fitness_csv);
    }
    if (time_csv != NULL) {
        fclose(time_csv);
    }
    
    // Salva il Cervello Master definitivo su disco
    nn_save(&base_nn, "best_model.bin");

    // Memory de-allocation to prevent leaks
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
    // Using the current time as seed guarantees a new scenario every time
    unsigned int test_seed = 137; //(unsigned int)time(NULL); 
    double start_x[N_DRONES], start_y[N_DRONES], start_z[N_DRONES];
    
    generate_random_environment(test_seed, start_x, start_y, start_z);
    export_environment("environment.csv"); // Exports for Python visualization

    printf("Test Map Generated (Seed: %u)\n", test_seed);
    for (int d = 0; d < N_DRONES; d++) {
        printf("Drone %d Target: X=%.2f, Y=%.2f, Z=%.2f\n", d, target_x[d], target_y[d], target_z[d]);
    }

    // 3. Executes a single rollout and records the trajectory
    FILE* telemetry_csv = fopen("telemetry.csv", "w");
    if (telemetry_csv != NULL) {
        fprintf(telemetry_csv, "time"); 
        for (int d = 0; d < N_DRONES; d++) fprintf(telemetry_csv, ",x%d,y%d,z%d", d, d, d);
        fprintf(telemetry_csv, "\n");
        
        printf("\n>>> STARTING INFERENCE FLIGHT...\n");
        double final_fitness = run_episode(fmus, start_x, start_y, start_z, &nn, telemetry_csv);
        
        fclose(telemetry_csv);
        printf(">>> FLIGHT COMPLETED. Final Fitness: %.2f\n", final_fitness);
        printf(">>> Run visualize.py to view the drone's trajectory in 3D.\n\n");
    }
}