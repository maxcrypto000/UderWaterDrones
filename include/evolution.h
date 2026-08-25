#ifndef EVOLUTION_H
#define EVOLUTION_H

#include <fmilib.h>
#include "neural_network.h"

// ============================================================================
// EVOLUTION STRATEGY HYPERPARAMETERS (Algorithm 1 - OpenAI)
// ============================================================================

// Number of "clones" (perturbed neural networks) evaluated in each generation.
// A larger population stabilizes the gradient, but requires more computation.
#define POPULATION_SIZE 100

// Total number of generations (weight update cycles) before stopping.
#define GENERATIONS 1000

// Number of different environments each neural network is evaluated on (Monte Carlo)
#define MONTECARLO_SAMPLES 5


// Standard deviation (Sigma) for extracting Gaussian noise.
// Controls how much we "explore" the parameter space. If too low, 
// the network learns nothing new. If too high, the behavior is pure chaos.
#define SIGMA 0.1        

// Learning rate (Alpha). 
// Determines how heavily we update the original network (base_nn) 
// in the direction of the clones that achieved the best scores.
#define LEARNING_RATE 0.02

// ============================================================================
// EXPOSED FUNCTIONS
// ============================================================================

// Starts the Evolution Strategy generational loop.
// Takes as input the pointer to the FMU already instantiated in main.
void es_train(fmi2_import_t* fmu);




// Testa una Rete Neurale precedentemente addestrata
void es_test(fmi2_import_t* fmu, const char* model_filename);
#endif