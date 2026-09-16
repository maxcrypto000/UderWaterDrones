#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fmilib.h>
#include "environment.h"
#include "evolution.h"
#include "mission.h"

void jm_logger(jm_callbacks* c, jm_string module, jm_log_level_enu_t log_level, jm_string message) {}

int main(int argc, char* argv[]) {
    
    int run_test = 0; 
    int run_mission = 0;
    
    if (argc > 1) {
        if (strcmp(argv[1], "test") == 0) {
            run_test = 1;
        } else if (strcmp(argv[1], "mission") == 0) {
            run_mission = 1;
        }
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
    if (run_mission) {
        int req_drones = -1;
        int req_obstacles = -1;
        if (argc >= 3) req_drones = atoi(argv[2]);
        if (argc >= 4) req_obstacles = atoi(argv[3]);
        es_mission(fmus, "best_model.bin", req_drones, req_obstacles);
    } else if (run_test) {
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
