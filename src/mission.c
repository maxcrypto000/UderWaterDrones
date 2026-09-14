/**
 * @file mission.c
 * @brief Implementation of the dynamic mission simulation and drone state machine.
 */

#include "mission.h"
#include "evolution.h"
#include "neural_network.h"
#include "environment.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// --- FMU Value References (VR) ---
// These map directly to the variables inside the ModelDescription.xml of the FMU.
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

/** @brief Maximum engine thrust (in Newtons) applied to each axis. */
#define MAX_THRUST 20.0 

void es_mission(fmi2_import_t* fmus[N_DRONES], const char* model_filename) {
    NeuralNetwork nn;
    
    // 1. Load the pre-trained neural network weights
    if (!nn_load(&nn, model_filename)) {
        printf(">>> ERROR: Could not load %s. Please run training first!\n", model_filename);
        return;
    }
    
    printf("\n>>> MISSION SYSTEM INITIALIZED WITH MODEL %s <<<\n", model_filename);

    // 2. Generate the mission environment
    unsigned int mission_seed = 404; 
    double start_x[N_DRONES], start_y[N_DRONES], start_z[N_DRONES];
    
    // Generates an environment where starting positions are firmly on the seabed
    generate_mission_environment(mission_seed, start_x, start_y, start_z);
    export_environment("mission_environment.csv");

    printf("Mission Map Generated (Seed: %u)\n", mission_seed);
    
    // 3. Initialize tracking variables for the Finite State Machine (FSM)
    DroneState state[N_DRONES];
    double base_x[N_DRONES], base_y[N_DRONES], base_z[N_DRONES];
    double current_x[N_DRONES], current_y[N_DRONES], current_z[N_DRONES], current_battery[N_DRONES];
    int active[N_DRONES];
    
    for (int d = 0; d < num_active_drones; d++) {
        // Record the starting coordinates to serve as the charging base
        base_x[d] = start_x[d];
        base_y[d] = start_y[d];
        base_z[d] = start_z[d];
        
        // Initialize current state
        current_x[d] = start_x[d];
        current_y[d] = start_y[d];
        current_z[d] = start_z[d];
        current_battery[d] = 100.0;
        
        // Drones start by seeking a randomly generated local target
        state[d] = STATE_SEEK_TARGET;
        generate_local_target(current_x[d], current_y[d], current_z[d], &target_x[d], &target_y[d], &target_z[d], I_RADIUS);
        active[d] = 1;
        printf("Drone %d assigned initial TARGET: X=%.2f, Y=%.2f, Z=%.2f\n", d, target_x[d], target_y[d], target_z[d]);
    }
    
    // Disable any unused drone slots
    for (int d = num_active_drones; d < N_DRONES; d++) {
        active[d] = 0;
    }
    
    // 4. Configure Simulation Parameters
    double t_start = 0.0;
    double t_end = 300.0;     // Run the mission for a full 5 minutes to observe recharge cycles
    double step_size = 0.05;  // 50 milliseconds step size
    
    fmi2_value_reference_t vr_starts[3] = { VR_START_X, VR_START_Y, VR_START_Z };
    fmi2_value_reference_t vr_inputs[3] = { VR_UX, VR_UY, VR_UZ };
    fmi2_value_reference_t vr_outputs[4] = { VR_X, VR_Y, VR_Z, VR_BATTERY };
    
    // Initialize FMU experiment parameters for each drone
    for (int d = 0; d < num_active_drones; d++) {
        fmi2_import_setup_experiment(fmus[d], fmi2_true, 1e-4, t_start, fmi2_true, t_end);
        double start_vals[3] = { start_x[d], start_y[d], start_z[d] };
        fmi2_import_set_real(fmus[d], vr_starts, 3, start_vals);
        fmi2_import_enter_initialization_mode(fmus[d]);
        fmi2_import_exit_initialization_mode(fmus[d]);
    }

    // Prepare telemetry logging
    FILE* telemetry_csv = fopen("mission_telemetry.csv", "w");
    if (telemetry_csv != NULL) {
        fprintf(telemetry_csv, "time"); 
        for (int d = 0; d < N_DRONES; d++) {
            fprintf(telemetry_csv, ",x%d,y%d,z%d,bat%d,state%d,tx%d,ty%d,tz%d", d, d, d, d, d, d, d, d);
        }
        fprintf(telemetry_csv, "\n");
    }
    
    double current_time = t_start;
    
    printf("\n>>> MISSION STARTED...\n");
    
    // 5. Main Simulation Loop
    while (current_time < t_end) {
        
        // A. Extract current telemetry from the FMUs
        for (int d = 0; d < num_active_drones; d++) {
            if (!active[d]) continue;
            double output_values[4];
            fmi2_import_get_real(fmus[d], vr_outputs, 4, output_values);
            current_x[d] = output_values[0];
            current_y[d] = output_values[1];
            current_z[d] = output_values[2];
            current_battery[d] = output_values[3];
        }
        
        // B. Log current state to CSV
        if (telemetry_csv != NULL) {
            fprintf(telemetry_csv, "%.3f", current_time);
            for (int d = 0; d < N_DRONES; d++) {
                if (active[d]) {
                    fprintf(telemetry_csv, ",%.3f,%.3f,%.3f,%.3f,%d,%.3f,%.3f,%.3f", 
                            current_x[d], current_y[d], current_z[d], current_battery[d], state[d], 
                            target_x[d], target_y[d], target_z[d]);
                } else {
                    fprintf(telemetry_csv, ",0,0,0,0,0,0,0,0");
                }
            }
            fprintf(telemetry_csv, "\n");
        }
        
        // C. Evaluate FSM and Neural Network for each drone
        for (int d = 0; d < num_active_drones; d++) {
            if (!active[d]) continue;
            
            // Calculate distance to the current active target
            double dx = target_x[d] - current_x[d];
            double dy = target_y[d] - current_y[d];
            double dz = target_z[d] - current_z[d];
            double current_distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            // --- STATE MACHINE TRANSITION LOGIC ---
            
            if (state[d] == STATE_SEEK_TARGET) {
                // If drone has arrived at its local target (within 5.0m threshold)
                if (current_distance < 5.0) {
                    if (current_battery[d] < BATTERY_LOW_THRESHOLD) {
                        // Battery is too low. Abort mission and return to base.
                        state[d] = STATE_RETURN_BASE;
                        target_x[d] = base_x[d];
                        target_y[d] = base_y[d];
                        target_z[d] = base_z[d];
                        printf("T=%.1f | Drone %d LOW BATTERY (%.1f%%). Returning to BASE.\n", current_time, d, current_battery[d]);
                    } else {
                        // Generate the next waypoint in the patrol sequence
                        generate_local_target(current_x[d], current_y[d], current_z[d], &target_x[d], &target_y[d], &target_z[d], I_RADIUS);
                        printf("T=%.1f | Drone %d reached target! New target assigned.\n", current_time, d);
                    }
                }
            } 
            else if (state[d] == STATE_RETURN_BASE) {
                // If drone has arrived at the charging base
                if (current_distance < 5.0) {
                    state[d] = STATE_RECHARGING;
                    printf("T=%.1f | Drone %d arrived at BASE. Recharging...\n", current_time, d);
                }
            } 
            else if (state[d] == STATE_RECHARGING) {
                // Wait until battery reaches the FULL threshold before resuming operations
                if (current_battery[d] >= BATTERY_FULL_THRESHOLD) {
                    state[d] = STATE_SEEK_TARGET;
                    generate_local_target(current_x[d], current_y[d], current_z[d], &target_x[d], &target_y[d], &target_z[d], I_RADIUS);
                    printf("T=%.1f | Drone %d FULLY CHARGED. Resuming mission.\n", current_time, d);
                }
            }
            
            // Recompute relative distance in case the target was swapped during state transitions
            dx = target_x[d] - current_x[d];
            dy = target_y[d] - current_y[d];
            dz = target_z[d] - current_z[d];
            current_distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            double input_values[3] = {0.0, 0.0, 0.0};
            
            // If the drone is charging, cut engines entirely
            if (state[d] == STATE_RECHARGING) {
                input_values[0] = 0.0;
                input_values[1] = 0.0;
                input_values[2] = 0.0;
            } else {
                // Process environment through Lidar sensors
                double lidar_distances[NUM_RAYS];
                compute_lidar_rays(d, current_x[d], current_y[d], current_z[d], current_x, current_y, current_z, active, lidar_distances);
                
                double nn_inputs[NN_INPUT_SIZE];
                double nn_outputs[NN_OUTPUT_SIZE];
                
                // Normalize Lidar inputs [0, 1]
                for (int i = 0; i < NUM_RAYS; i++) {
                    nn_inputs[i] = lidar_distances[i] / MAX_LIDAR_RANGE;
                }
                
                // Provide normalized directional vector towards target
                if (current_distance > 0.001) {
                    nn_inputs[64] = dx / current_distance;
                    nn_inputs[65] = dy / current_distance;
                    nn_inputs[66] = dz / current_distance;
                } else {
                    nn_inputs[64] = 0.0;
                    nn_inputs[65] = 0.0;
                    nn_inputs[66] = 0.0;
                }
                
                // Provide normalized battery level
                nn_inputs[67] = current_battery[d] / 100.0;
                
                // Forward pass through the pre-trained Neural Network
                nn_feedforward(&nn, nn_inputs, nn_outputs);
                
                // Scale normalized network outputs to physical thrust values
                input_values[0] = nn_outputs[0] * MAX_THRUST;
                input_values[1] = nn_outputs[1] * MAX_THRUST;
                input_values[2] = nn_outputs[2] * MAX_THRUST;
            }
            
            // D. Push inputs into FMU and step the physics engine
            fmi2_import_set_real(fmus[d], vr_inputs, 3, input_values);
            fmi2_import_do_step(fmus[d], current_time, step_size, fmi2_true);
        }
        
        current_time += step_size;
    }
    
    // 6. Cleanup
    if (telemetry_csv != NULL) fclose(telemetry_csv);
    
    for (int d = 0; d < num_active_drones; d++) {
        fmi2_import_terminate(fmus[d]);
        fmi2_import_reset(fmus[d]);
    }
    printf(">>> MISSION COMPLETED <<<\n");
}
