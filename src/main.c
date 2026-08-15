#include <stdio.h>
#include <stdlib.h>
#include <fmilib.h>
#include "../include/environment.h"

// --- VALUE REFERENCES ---
#define VR_UX 12  
#define VR_UY 13
#define VR_UZ 14

#define VR_X  19   
#define VR_Y  20
#define VR_Z  21
#define VR_VX 16

// AGGIUNGI I VR DELLE COORDINATE DI START
#define VR_START_X 24 
#define VR_START_Y 25 
#define VR_START_Z 26 
// -------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <fmilib.h>
#include "environment.h"

// --- VALUE REFERENCES FROM XML ---
#define VR_UX 12  
#define VR_UY 13
#define VR_UZ 14

#define VR_X  19   
#define VR_Y  20
#define VR_Z  21
#define VR_VX 16

// REPLACE WITH YOUR ACTUAL START VRs!
#define VR_START_X 000 
#define VR_START_Y 000 
#define VR_START_Z 000 
// -------------------------------------------------

// Mandatory callback for FMILibrary to handle logging
void jm_logger(jm_callbacks* c, jm_string module, jm_log_level_enu_t log_level, jm_string message) {
    // Left empty to keep the console output clean
}

// Executes a single rollout (episode)
double run_episode(fmi2_import_t* fmu, double start_x, double start_y, double start_z, double thrust_x, FILE* csv_file) {
    
    double t_start = 0.0;
    double t_end = 20.0;     // Run for 20 virtual seconds
    double step_size = 0.05; // 50ms timestep
    
    fmi2_import_setup_experiment(fmu, fmi2_true, 1e-4, t_start, fmi2_true, t_end);

    // 1. INJECT RANDOMIZED START COORDINATES
    fmi2_value_reference_t vr_starts[3] = { VR_START_X, VR_START_Y, VR_START_Z };
    double start_vals[3] = { start_x, start_y, start_z };
    fmi2_import_set_real(fmu, vr_starts, 3, start_vals);

    fmi2_import_enter_initialization_mode(fmu);
    fmi2_import_exit_initialization_mode(fmu);

    fmi2_value_reference_t vr_inputs[3] = { VR_UX, VR_UY, VR_UZ };
    fmi2_value_reference_t vr_outputs[4] = { VR_X, VR_Y, VR_Z, VR_VX };
    
    double input_values[3] = { thrust_x, 0.0, 0.0 }; 
    double output_values[4] = { 0.0, 0.0, 0.0, 0.0 };
    
    double lidar_distances[NUM_RAYS];
    int step_counter = 0;
    double current_time = t_start;

    // Main Simulation Loop
    while (current_time < t_end) {
        
        // A. Read current pose from FMU
        fmi2_import_get_real(fmu, vr_outputs, 4, output_values);
        double current_x = output_values[0];
        double current_y = output_values[1]; // Altitude
        double current_z = output_values[2]; // Depth

        // B. Save telemetry to CSV for Python visualization
        if (csv_file != NULL) {
            fprintf(csv_file, "%.3f,%.3f,%.3f,%.3f\n", current_time, current_x, current_y, current_z);
        }

        // C. Compute 3D Raycasting Lidar 
        compute_lidar_rays(current_x, current_y, current_z, lidar_distances);

        // D. Print Lidar summary every 1.0 virtual seconds
        if (step_counter % 20 == 0) {
            print_lidar_rays(lidar_distances, current_time, current_x, current_y, current_z);
        }

        // E. Inject forces and advance physics
        fmi2_import_set_real(fmu, vr_inputs, 3, input_values);
        fmi2_import_do_step(fmu, current_time, step_size, fmi2_true);
        
        current_time += step_size;
        step_counter++;
    }
    
    // Read final X position for testing
    fmi2_import_get_real(fmu, vr_outputs, 4, output_values);
    
    // Terminate and RESET the FMU for the next episode
    fmi2_import_terminate(fmu);
    fmi2_import_reset(fmu); 

    return output_values[0]; 
}

int main() {
    // 1. Initialize Raycasting Geometry (Fibonacci sphere)
    init_lidar();

    // 2. Initialize FMILibrary and callbacks
    jm_callbacks callbacks;
    callbacks.malloc = malloc;
    callbacks.calloc = calloc;
    callbacks.realloc = realloc;
    callbacks.free = free;
    callbacks.logger = jm_logger;
    callbacks.log_level = jm_log_level_error; 
    callbacks.context = 0;

    fmi_import_context_t* context = fmi_import_allocate_context(&callbacks);
    
    const char* fmu_path = "../fmu/UnderwaterDronePhysics.fmu";
    const char* tmp_dir = fmi_import_mk_temp_dir(&callbacks, ".", "fmu_tmp_");
    
    fmi_import_get_fmi_version(context, fmu_path, tmp_dir);
    fmi2_import_t* fmu = fmi2_import_parse_xml(context, tmp_dir, 0);

    fmi2_callback_functions_t fmi2Callbacks;
    fmi2Callbacks.logger = fmi2_log_forwarding;
    fmi2Callbacks.allocateMemory = calloc;
    fmi2Callbacks.freeMemory = free;
    fmi2Callbacks.componentEnvironment = fmu;

    fmi2_import_create_dllfmu(fmu, fmi2_fmu_kind_cs, &fmi2Callbacks);

    // Instantiate the FMU ONLY ONCE
    fmi2_import_instantiate(fmu, "DroneTest", fmi2_cosimulation, NULL, fmi2_false);

    // --- DOMAIN RANDOMIZATION & TESTING ---
    printf("Initializing Procedural Environment...\n");

    unsigned int generation_seed = 4; 
    double start_x, start_y, start_z;
    
    // Generate the map layout and a safe spawn point based on the seed
    generate_random_environment(generation_seed, &start_x, &start_y, &start_z);

    // NUOVA RIGA: Esporta l'ambiente generato per Python
    export_environment("environment.csv");

    printf("Map Generation Complete (Seed: %u)\n", generation_seed);
    printf("Drone Safe Spawn: X=%.2f, Y=%.2f, Z=%.2f\n", start_x, start_y, start_z);

    // Open CSV file for telemetry
    FILE* telemetry_csv = fopen("telemetry.csv", "w");
    if (telemetry_csv != NULL) {
        fprintf(telemetry_csv, "time,x,y,z\n"); 
    } else {
        printf("Warning: Could not open telemetry.csv for writing.\n");
    }

    // Run the episode with the procedurally generated spawn coordinates
    printf("\n>>> STARTING EPISODE...\n");
    double fitness = run_episode(fmu, start_x, start_y, start_z, 10.0, telemetry_csv);
    printf(">>> EPISODE COMPLETED. Final X = %.2f m\n", fitness);

    if (telemetry_csv != NULL) {
        fclose(telemetry_csv);
        printf("Telemetry saved to telemetry.csv\n");
    }

    // --- MEMORY CLEANUP ---
    fmi2_import_free_instance(fmu);
    fmi2_import_destroy_dllfmu(fmu);
    fmi2_import_free(fmu);
    fmi_import_free_context(context);
    fmi_import_rmdir(&callbacks, tmp_dir);
    callbacks.free((void*)tmp_dir);

    return 0;
}