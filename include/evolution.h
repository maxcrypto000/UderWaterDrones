#ifndef EVOLUTION_H
#define EVOLUTION_H

#include <fmilib.h>
#include "neural_network.h"
#include "environment.h"

// Number of "clones" (perturbed neural networks) evaluated in each generation.
#define POPULATION_SIZE 100

// Total number of generations before stopping.
#define GENERATIONS 1000

// Number of different environments each neural network is evaluated on (Monte Carlo)
#define MONTECARLO_SAMPLES 5

// Enable team crash detection 
#define ENABLE_TEAM_CRASH 0


// Standard deviation (Sigma) for extracting Gaussian noise.
#define SIGMA 0.1        

// Learning rate (Alpha). 
#define LEARNING_RATE 0.02


/**
 * @brief Trains The neural network using the parameters.
 *
 * @param fmus the array of fmu pointers 
 */
void es_train(fmi2_import_t* fmus[N_DRONES]);

/**
 * @brief Tests The neural network using the parameters.
 *
 * @param fmus the array of fmu pointers 
 * @param model_filename Path to the binary file containing the pre-trained neural network weights
 */
void es_test(fmi2_import_t* fmus[N_DRONES], const char* model_filename);
#endif