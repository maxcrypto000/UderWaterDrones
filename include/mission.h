/**
 * @file mission.h
 * @brief Header file defining the dynamic mission simulation for the drone swarm.
 *
 * This module introduces a continuous mission system where drones utilize a 
 * pre-trained neural network to navigate towards dynamically generated local targets.
 * It includes a finite state machine to manage battery levels, forcing drones 
 * to return to their starting base to recharge when power is low.
 */

#ifndef MISSION_H
#define MISSION_H

#include <fmilib.h>
#include "environment.h"

// --- Configuration Parameters ---

/** @brief Radius (in meters) within which new dynamic targets are generated. */
#define I_RADIUS 40.0

/** @brief Battery percentage threshold that triggers the Return-to-Base sequence. */
#define BATTERY_LOW_THRESHOLD 50.0

/** @brief Battery percentage required to complete recharging and resume the mission. */
#define BATTERY_FULL_THRESHOLD 95.0

/**
 * @enum DroneState
 * @brief Represents the current operational state of a single drone.
 */
typedef enum {
    STATE_SEEK_TARGET,  /**< Drone is actively navigating towards its dynamic target. */
    STATE_RETURN_BASE,  /**< Battery is low; drone is navigating back to its starting coordinate. */
    STATE_RECHARGING    /**< Drone is idle at the base, allowing the FMU to recharge the battery. */
} DroneState;

// --- Function Prototypes ---

/**
 * @brief Executes the dynamic mission simulation.
 * 
 * @param fmus Array of initialized FMI simulation instances (one per drone).
 * @param model_filename Path to the binary file containing the pre-trained neural network weights.
 */
void es_mission(fmi2_import_t* fmus[N_DRONES], const char* model_filename, int req_drones, int req_obstacles);

#endif // MISSION_H

