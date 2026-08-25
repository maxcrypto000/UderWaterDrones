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

// Executes a single simulation episode for a specific Neural Network.
// Returns the accumulated Fitness score based on Dense Rewards.
static double run_episode(fmi2_import_t* fmu, double start_x, double start_y, double start_z, NeuralNetwork* nn, FILE* csv_file) {
    
    double t_start = 0.0;
    double t_end = 60.0;  // The episode lasts a maximum of 20 virtual seconds
    double step_size = 0.05; // Solver resolution: 50 milliseconds
    
    // Initializes the experiment setup (start and end time)
    fmi2_import_setup_experiment(fmu, fmi2_true, 1e-4, t_start, fmi2_true, t_end);

    // SAFE SPAWN INJECTION: Modifies the initial coordinates of the FMU 
    // bypassing the model defaults, ensuring Domain Randomization.
    fmi2_value_reference_t vr_starts[3] = { VR_START_X, VR_START_Y, VR_START_Z };
    double start_vals[3] = { start_x, start_y, start_z };
    fmi2_import_set_real(fmu, vr_starts, 3, start_vals);

    // Physically applies the start values, initializing the equations of motion
    fmi2_import_enter_initialization_mode(fmu);
    fmi2_import_exit_initialization_mode(fmu);

    fmi2_value_reference_t vr_inputs[3] = { VR_UX, VR_UY, VR_UZ };
    fmi2_value_reference_t vr_outputs[4] = { VR_X, VR_Y, VR_Z, VR_VX };
    
    double input_values[3] = { 0.0, 0.0, 0.0 }; 
    double output_values[4] = { 0.0, 0.0, 0.0, 0.0 };
    
    double lidar_distances[NUM_RAYS];
    double nn_inputs[NN_INPUT_SIZE];
    double nn_outputs[NN_OUTPUT_SIZE];

    double current_time = t_start;
    double fitness = 0.0;
    
    // Initialize the distance to track step-by-step progress
    double dx = target_x - start_x;
    double dy = target_y - start_y;
    double dz = target_z - start_z;
    double previous_distance = sqrt(dx*dx + dy*dy + dz*dz);

    // PHYSICS LOOP: Runs in steps of 0.05s until the end of the episode
    while (current_time < t_end) {
        
        // 1. Reads the current position (X, Y, Z) from the solver
        fmi2_import_get_real(fmu, vr_outputs, 4, output_values);
        double current_x = output_values[0];
        double current_y = output_values[1];
        double current_z = output_values[2];

        // Writes to file for offline Python visualization (if enabled)
        if (csv_file != NULL) {
            fprintf(csv_file, "%.3f,%.3f,%.3f,%.3f\n", current_time, current_x, current_y, current_z);
        }

        // 2. Calculates the intersections of the Lidar rays with the random obstacles
        compute_lidar_rays(current_x, current_y, current_z, lidar_distances);

        int collision = 0;
        // Populates the neural input and normalizes the Lidar distances between 0.0 and 1.0
        for (int i = 0; i < NUM_RAYS; i++) {
            nn_inputs[i] = lidar_distances[i] / MAX_LIDAR_RANGE; 
            if (lidar_distances[i] <= 0.5) { 
                collision = 1; // Critical compenetration detected (Crash)
            }
        }
        
        // Calculate the new distance from the target
        dx = target_x - current_x;
        dy = target_y - current_y;
        dz = target_z - current_z;
        double current_distance = sqrt(dx*dx + dy*dy + dz*dz);

        // =========================================================
        // CONTINUOUS REWARD SHAPING
        // =========================================================
        
        // A. Progress Reward (Breadcrumbs)
        // If previous > current, the drone is getting closer (positive value).
        // If it flies away, progress is negative, punishing bad routes immediately.
        double progress = previous_distance - current_distance;
        fitness += progress * 200.0; 
        //printf("progress: %f\n", progress);
        
        previous_distance = current_distance; // Update for the next step

        // B. Control Cost (Energy Efficiency)
        // Small penalty based on the extreme use of the engines (tanh outputs squared).
        // Encourages smooth trajectories over erratic, full-throttle maneuvers.
        double effort = (nn_outputs[0]*nn_outputs[0] + nn_outputs[1]*nn_outputs[1] + nn_outputs[2]*nn_outputs[2]);
        fitness -= effort * 0.01; 
        
        // =========================================================

        // 3. Cumulative Terminal States (Added/Subtracted, not overwritten)
        if (collision) {
            //printf("CRASHED!\n");
            fitness -= 500.0; // Severe final penalty for crashing
            break; 
        }

        if (current_distance < 5.0) {
            //printf("VICTORY!\n");
            fitness += 1000.0 + (t_end - current_time) * 100.0; // Victory + Time efficiency bonus
            break;
        }

        // 4. Prepares Target Vector Inputs
        // Translational invariance: passing the scaled relative distance instead of absolute coordinates
        nn_inputs[64] = dx / 100.0;
        nn_inputs[65] = dy / 100.0;
        nn_inputs[66] = dz / 100.0;

        // 5. Inference: The Network "thinks" and produces the output
        nn_feedforward(nn, nn_inputs, nn_outputs);

        // 6. Actuation: Neural mapping (from -1 to 1) into physical Newtons
        input_values[0] = nn_outputs[0] * MAX_THRUST;
        input_values[1] = nn_outputs[1] * MAX_THRUST;
        input_values[2] = nn_outputs[2] * MAX_THRUST;

        // Injects the commands into the OpenModelica thrusters and advances time
        fmi2_import_set_real(fmu, vr_inputs, 3, input_values);
        fmi2_import_do_step(fmu, current_time, step_size, fmi2_true);
        
        current_time += step_size;
    }
    
    // Fast reset of the FMU to prepare it for the next episode (saves CPU)
    fmi2_import_terminate(fmu);
    fmi2_import_reset(fmu); 

    return fitness; 
}

// ============================================================================
// OPTIMIZATION ALGORITHM (OpenAI ES - Algorithm 1)
// ============================================================================

void es_train(fmi2_import_t* fmu) {
    
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

    // GENERATIONAL LOOP
    for (int gen = 0; gen < GENERATIONS; gen++) {
         
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
                double start_x, start_y, start_z;
                
                // 1. Generate random environment
                generate_random_environment(env_seed, &start_x, &start_y, &start_z);
                
                // Export only the first environment for visualization
                if (p == 0 && m == 0) {
                    export_environment("environment.csv"); 
                }

                // 2 & 3. Send clone into the simulator and accumulate fitness
                clone_total_fitness += run_episode(fmu, start_x, start_y, start_z, &perturbed_nns[p], NULL);
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

        printf("Generation %03d | Average Fitness: %8.2f | Max: %8.2f\n", gen, total_fitness / POPULATION_SIZE, max_fitness);
        
        if (fitness_csv != NULL) {
            fprintf(fitness_csv, "%d,%f,%f\n", gen, total_fitness / POPULATION_SIZE, max_fitness);
            fflush(fitness_csv);
        }

        // D. Offline Telemetry Recording for Visual Evaluation
        // Performs an extra rollout with the MASTER network (no noise) to record what it has learned.
        
        unsigned int telemetry_seed = 42 ; 
        double tel_start_x, tel_start_y, tel_start_z;
        generate_random_environment(telemetry_seed, &tel_start_x, &tel_start_y, &tel_start_z);
        
        FILE* telemetry_csv = fopen("telemetry.csv", "w");
        if (telemetry_csv != NULL) {
            fprintf(telemetry_csv, "time,x,y,z\n"); 
            run_episode(fmu, tel_start_x, tel_start_y, tel_start_z, &base_nn, telemetry_csv);
            fclose(telemetry_csv);
        }
    }

    if (fitness_csv != NULL) {
        fclose(fitness_csv);
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

void es_test(fmi2_import_t* fmu, const char* model_filename) {
    NeuralNetwork nn;
    
    // 1. Loads the pre-trained weights from the binary file
    if (!nn_load(&nn, model_filename)) {
        printf(">>> ERROR: Could not load %s. Please run training first!\n", model_filename);
        return;
    }
    
    printf("\n>>> PRE-TRAINED MODEL SUCCESSFULLY LOADED FROM %s <<<\n", model_filename);

    // 2. Generates a completely new map to test generalization
    // Using the current time as seed guarantees a new scenario every time
    unsigned int test_seed = 1237; //(unsigned int)time(NULL); 
    double start_x, start_y, start_z;
    
    generate_random_environment(test_seed, &start_x, &start_y, &start_z);
    export_environment("environment.csv"); // Exports for Python visualization

    printf("Test Map Generated (Seed: %u)\n", test_seed);
    printf("Target coordinates: X=%.2f, Y=%.2f, Z=%.2f\n", target_x, target_y, target_z);

    // 3. Executes a single rollout and records the trajectory
    FILE* telemetry_csv = fopen("telemetry.csv", "w");
    if (telemetry_csv != NULL) {
        fprintf(telemetry_csv, "time,x,y,z\n"); 
        
        printf("\n>>> STARTING INFERENCE FLIGHT...\n");
        double final_fitness = run_episode(fmu, start_x, start_y, start_z, &nn, telemetry_csv);
        
        fclose(telemetry_csv);
        printf(">>> FLIGHT COMPLETED. Final Fitness: %.2f\n", final_fitness);
        printf(">>> Run visualize.py to view the drone's trajectory in 3D.\n\n");
    }
}