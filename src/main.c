#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fmilib.h>
#include "environment.h"
#include "evolution.h"

void jm_logger(jm_callbacks* c, jm_string module, jm_log_level_enu_t log_level, jm_string message) {}

// Notice the addition of argc and argv to read terminal commands
int main(int argc, char* argv[]) {
    
    // Default mode is train if no arguments are provided
    int run_test = 0; 
    if (argc > 1 && strcmp(argv[1], "test") == 0) {
        run_test = 1;
    }

    srand((unsigned int)time(NULL));
    init_lidar();

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
    
    fmi2_import_t* fmus[N_DRONES];
    fmi2_callback_functions_t* fmi2Callbacks_array[N_DRONES];

    for (int i = 0; i < N_DRONES; i++) {
        fmus[i] = fmi2_import_parse_xml(context, tmp_dir, 0);

        fmi2Callbacks_array[i] = malloc(sizeof(fmi2_callback_functions_t));
        fmi2Callbacks_array[i]->logger = fmi2_log_forwarding;
        fmi2Callbacks_array[i]->allocateMemory = calloc;
        fmi2Callbacks_array[i]->freeMemory = free;
        fmi2Callbacks_array[i]->componentEnvironment = fmus[i];

        fmi2_import_create_dllfmu(fmus[i], fmi2_fmu_kind_cs, fmi2Callbacks_array[i]);
        
        char instance_name[32];
        sprintf(instance_name, "Drone_%d", i);
        fmi2_import_instantiate(fmus[i], instance_name, fmi2_cosimulation, NULL, fmi2_false);
    }

    // --- EXECUTION BRANCH ---
    if (run_test) {
        // Runs the pre-trained neural network on a random map
        es_test(fmus, "best_model.bin");
    } else {
        // Starts the 1000-generation Evolution Strategy
        es_train(fmus);
    }

    // Memory Cleanup
    for (int i = 0; i < N_DRONES; i++) {
        fmi2_import_free_instance(fmus[i]);
        fmi2_import_destroy_dllfmu(fmus[i]);
        fmi2_import_free(fmus[i]);
        free(fmi2Callbacks_array[i]);
    }
    fmi_import_free_context(context);
    fmi_import_rmdir(&callbacks, tmp_dir);
    callbacks.free((void*)tmp_dir);

    return 0;
}