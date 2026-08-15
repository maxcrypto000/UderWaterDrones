#include <stdio.h>
#include <stdlib.h>
#include <fmilib.h>
#include "../include/environment.h"
#include "../include/neural_network.h"

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
#define VR_START_X 24
#define VR_START_Y 25 
#define VR_START_Z 26 
// -------------------------------------------------

#define MAX_THRUST 20.0 // N
// -------------------------------------------------

void jm_logger(jm_callbacks* c, jm_string module, jm_log_level_enu_t log_level, jm_string message) {}

// Passiamo il puntatore alla Rete Neurale
double run_episode(fmi2_import_t* fmu, double start_x, double start_y, double start_z, NeuralNetwork* nn, FILE* csv_file) {
    
    double t_start = 0.0;
    double t_end = 20.0;     
    double step_size = 0.05; 
    
    fmi2_import_setup_experiment(fmu, fmi2_true, 1e-4, t_start, fmi2_true, t_end);

    fmi2_value_reference_t vr_starts[3] = { VR_START_X, VR_START_Y, VR_START_Z };
    double start_vals[3] = { start_x, start_y, start_z };
    fmi2_import_set_real(fmu, vr_starts, 3, start_vals);

    fmi2_import_enter_initialization_mode(fmu);
    fmi2_import_exit_initialization_mode(fmu);

    fmi2_value_reference_t vr_inputs[3] = { VR_UX, VR_UY, VR_UZ };
    fmi2_value_reference_t vr_outputs[4] = { VR_X, VR_Y, VR_Z, VR_VX };
    
    double input_values[3] = { 0.0, 0.0, 0.0 }; 
    double output_values[4] = { 0.0, 0.0, 0.0, 0.0 };
    
    double lidar_distances[NUM_RAYS];
    double nn_inputs[NN_INPUT_SIZE];
    double nn_outputs[NN_OUTPUT_SIZE];

    int step_counter = 0;
    double current_time = t_start;

    while (current_time < t_end) {
        
        fmi2_import_get_real(fmu, vr_outputs, 4, output_values);
        double current_x = output_values[0];
        double current_y = output_values[1];
        double current_z = output_values[2];

        if (csv_file != NULL) {
            fprintf(csv_file, "%.3f,%.3f,%.3f,%.3f\n", current_time, current_x, current_y, current_z);
        }

        // 1. Calcola il Lidar
        compute_lidar_rays(current_x, current_y, current_z, lidar_distances);

        // 2. Prepara gli Input per la Rete Neurale (Normalizzati)
        for (int i = 0; i < NUM_RAYS; i++) {
            nn_inputs[i] = lidar_distances[i] / MAX_LIDAR_RANGE; 
        }
        // Il vettore relativo verso il bersaglio (Target - Current) diviso per 100 per scalarlo
        nn_inputs[64] = (target_x - current_x) / 100.0;
        nn_inputs[65] = (target_y - current_y) / 100.0;
        nn_inputs[66] = (target_z - current_z) / 100.0;

        // 3. Esegui il Feedforward
        nn_feedforward(nn, nn_inputs, nn_outputs);

        // 4. Mappa gli output [-1, 1] della tanh nella spinta reale in Newton
        input_values[0] = nn_outputs[0] * MAX_THRUST;
        input_values[1] = nn_outputs[1] * MAX_THRUST;
        input_values[2] = nn_outputs[2] * MAX_THRUST;

        // 5. Inietta la spinta decisa dalla AI e avanza la simulazione
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
    init_lidar();

    // INIZIALIZZA LA RETE NEURALE CON PESI CASUALI
    NeuralNetwork brain;
    nn_init_random(&brain);

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
    fmi2_import_instantiate(fmu, "DroneTest", fmi2_cosimulation, NULL, fmi2_false);

    unsigned int generation_seed = 2; 
    double start_x, start_y, start_z;
    
    generate_random_environment(generation_seed, &start_x, &start_y, &start_z);
    export_environment("environment.csv");

    printf("AI RANDOM TEST (Seed: %u)\n", generation_seed);
    printf("Drone Spawn: X=%.2f, Y=%.2f, Z=%.2f\n", start_x, start_y, start_z);
    printf("Target Pos : X=%.2f, Y=%.2f, Z=%.2f\n", target_x, target_y, target_z);

    FILE* telemetry_csv = fopen("telemetry.csv", "w");
    if (telemetry_csv != NULL) {
        fprintf(telemetry_csv, "time,x,y,z\n"); 
    } 

    printf("\n>>> LETTING THE RANDOM AI DRIVE...\n");
    // Passiamo la rete 'brain' all'episodio
    run_episode(fmu, start_x, start_y, start_z, &brain, telemetry_csv);
    printf(">>> EPISODE COMPLETED.\n");

    if (telemetry_csv != NULL) fclose(telemetry_csv);

    fmi2_import_free_instance(fmu);
    fmi2_import_destroy_dllfmu(fmu);
    fmi2_import_free(fmu);
    fmi_import_free_context(context);
    fmi_import_rmdir(&callbacks, tmp_dir);
    callbacks.free((void*)tmp_dir);

    return 0;
}