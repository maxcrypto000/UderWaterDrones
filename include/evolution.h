/**
 * @file evolution.h
 * @brief Definition of the Evolution Strategy algorithm for Neural Network training.
 *
 * This module orchestrates the training phase of the neural networks using a 
 * gradient-free optimization algorithm (OpenAI Evolution Strategy). It perturbates
 * the weights, runs parallel simulations (Monte Carlo), evaluates fitness, and 
 * updates the master network.
 */

#ifndef EVOLUTION_H
#define EVOLUTION_H

#include <fmilib.h>
#include "neural_network.h"
#include "environment.h"

// ============================================================================
// EVOLUTION STRATEGY HYPERPARAMETERS (Algorithm 1 - OpenAI)
// ============================================================================

/** 
 * @brief Number of "clones" (perturbed neural networks) evaluated in each generation.
 * 
 * A larger population stabilizes the gradient estimate via Monte Carlo, 
 * but requires proportionally more computation time.
 */
#define POPULATION_SIZE 100

/** 
 * @brief Total number of generations (weight update epochs) before stopping.
 */
#define GENERATIONS 1000

/** 
 * @brief Number of different procedural environments each clone is evaluated on.
 * 
 * Averaging fitness over multiple maps prevents the network from overfitting 
 * a single specific map geometry (Monte Carlo sampling).
 */
#define MONTECARLO_SAMPLES 5

/** 
 * @brief Flag to enable immediate episode termination if ANY drone crashes.
 * 
 * Used for strict cooperative swarm training. Set to 0 to evaluate drones independently.
 */
#define ENABLE_TEAM_CRASH 0

/** 
 * @brief Standard deviation (Sigma) for extracting Gaussian noise.
 * 
 * Controls how aggressively we "explore" the parameter space (jittering).
 * - If too low: The network learns nothing new (vanishing updates).
 * - If too high: The behavioral changes are pure chaos and gradient is lost.
 */
#define SIGMA 0.1        

/** 
 * @brief Learning rate (Alpha) for Stochastic Gradient Descent.
 * 
 * Determines how heavily we update the original master network (base_nn) 
 * in the direction of the clones that achieved the best fitness scores.
 */
#define LEARNING_RATE 0.02

// ============================================================================
// EXPOSED FUNCTIONS
// ============================================================================

/**
 * @brief Starts the Evolution Strategy generational loop.
 *
 * This is the core training routine. It runs for GENERATIONS epochs, spawns
 * POPULATION_SIZE variants, executes run_episode() for each, normalizes fitness,
 * updates the master weights, and finally saves the best model to disk.
 *
 * @param fmus Array of initialized FMU instances (one per drone).
 */
void es_train(fmi2_import_t* fmus[N_DRONES]);

/**
 * @brief Tests a previously trained Neural Network in a new random environment.
 *
 * Inference-only mode. It loads the weights from the binary file, generates
 * a single random map, and flies the drones without updating the weights.
 * Telemetry is recorded for visualization.
 *
 * @param fmus Array of initialized FMU instances (one per drone).
 * @param model_filename Path to the trained .bin model file.
 */
void es_test(fmi2_import_t* fmus[N_DRONES], const char* model_filename);

#endif // EVOLUTION_H
