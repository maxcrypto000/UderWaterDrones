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

// Mandatory callback for FMILibrary to handle logging
void jm_logger(jm_callbacks* c, jm_string module, jm_log_level_enu_t log_level, jm_string message) {
    // Left empty to keep the console output clean
}

// Executes a single rollout (episode)
// Added a FILE pointer to save telemetry for visualization
double run_episode(fmi2_import_t* fmu, double start_x, double start_y, double start_z, double thrust_x, FILE* csv_file) {
    
    double t_start = 0.0;
    double t_end = 20.0;     
    double step_size = 0.05; 
    
    fmi2_import_setup_experiment(fmu, fmi2_true, 1e-4, t_start, fmi2_true, t_end);

    // INJECT START COORDINATES (X, Y=Altitude, Z=Depth)
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
        
        // 1. Read current pose from FMU
        fmi2_import_get_real(fmu, vr_outputs, 4, output_values);
        double current_x = output_values[0];
        double current_y = output_values[1]; // Altitude
        double current_z = output_values[2]; // Depth

        // Save telemetry to CSV for Python visualization
        if (csv_file != NULL) {
            fprintf(csv_file, "%.3f,%.3f,%.3f,%.3f\n", current_time, current_x, current_y, current_z);
        }

        // 2. Compute 3D Raycasting Lidar 
        compute_lidar_rays(current_x, current_y, current_z, lidar_distances);

        // Print the Lidar summary every 20 steps (1.0 virtual seconds)
        if (step_counter % 20 == 0) {
            print_lidar_rays(lidar_distances, current_time, current_x, current_y, current_z);
        }

        // 3. Inject forces and advance physics
        fmi2_import_set_real(fmu, vr_inputs, 3, input_values);
        fmi2_import_do_step(fmu, current_time, step_size, fmi2_true);
        
        current_time += step_size;
        step_counter++;
    }
    
    fmi2_import_get_real(fmu, vr_outputs, 4, output_values);
    
    fmi2_import_terminate(fmu);
    fmi2_import_reset(fmu); 

    return output_values[0]; 
}

int main() {
    // 1. Initialize environment (Fibonacci sphere for Lidar)
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

    // --- EXECUTE TEST EPISODE AND RECORD TELEMETRY ---
    printf("Starting Raycasting Lidar Test...\n");

    // Open CSV file
    FILE* telemetry_csv = fopen("telemetry.csv", "w");
    if (telemetry_csv != NULL) {
        fprintf(telemetry_csv, "time,x,y,z\n"); // CSV Header
    } else {
        printf("Warning: Could not open telemetry.csv for writing.\n");
    }

    // Episode 1: Start at X=0, Y=10 (Altitude), Z=0. Thrust 10N on X.
    printf("\n>>> EPISODE 1: Start(0, 10, 0), Thrust(10,0,0)\n");
    
    double fitness_1 = run_episode(fmu, 0.0, 10.0, 0.0, 10.0, telemetry_csv);
    
    printf(">>> EPISODE 1 COMPLETED. Final X = %.2f m\n", fitness_1);

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