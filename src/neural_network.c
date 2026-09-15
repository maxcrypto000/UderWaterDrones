/**
 * @file neural_network.c
 * @brief Implementation of the feed-forward Neural Network inference and persistence.
 */

#include "neural_network.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

// --- ACTIVATION FUNCTIONS ---

/**
 * @brief Hyperbolic tangent activation function.
 * 
 * Squashes any real number into the interval [-1.0, 1.0]. Used to provide 
 * non-linearity to the hidden layer and bound the outputs to realistic 
 * engine thrust constraints (Full Reverse to Full Forward).
 *
 * @param x The pre-activation sum.
 * @return The activated value.
 */
static double activation_tanh(double x) {
    return tanh(x);
}

/**
 * @brief Helper function to generate a pseudo-random initial weight.
 *
 * @return A random double uniformly distributed between -1.0 and 1.0.
 */
static double random_weight(void) {
    return ((double)rand() / RAND_MAX) * 2.0 - 1.0;
}

// --- NETWORK FUNCTIONS ---

void nn_init_random(NeuralNetwork* nn) {
    // 1. Initialize Input -> Hidden Layer parameters (W1, b1)
    for (int i = 0; i < NN_HIDDEN_SIZE; i++) {
        nn->b1[i] = random_weight();
        for (int j = 0; j < NN_INPUT_SIZE; j++) {
            nn->W1[i][j] = random_weight();
        }
    }
    
    // 2. Initialize Hidden -> Output Layer parameters (W2, b2)
    for (int i = 0; i < NN_OUTPUT_SIZE; i++) {
        nn->b2[i] = random_weight();
        for (int j = 0; j < NN_HIDDEN_SIZE; j++) {
            nn->W2[i][j] = random_weight();
        }
    }
}

void nn_feedforward(NeuralNetwork* nn, const double inputs[NN_INPUT_SIZE], double outputs[NN_OUTPUT_SIZE]) {
    double hidden[NN_HIDDEN_SIZE];

    // 1. Compute activations for the Hidden Layer
    // For each hidden node, compute the dot product of the input vector and the weight matrix row, then add the bias.
    for (int i = 0; i < NN_HIDDEN_SIZE; i++) {
        double sum = nn->b1[i];
        for (int j = 0; j < NN_INPUT_SIZE; j++) {
            sum += nn->W1[i][j] * inputs[j];
        }
        hidden[i] = activation_tanh(sum);
    }

    // 2. Compute activations for the Output Layer
    // For each output node, compute the dot product of the hidden vector and the weight matrix row, then add the bias.
    for (int i = 0; i < NN_OUTPUT_SIZE; i++) {
        double sum = nn->b2[i];
        for (int j = 0; j < NN_HIDDEN_SIZE; j++) {
            sum += nn->W2[i][j] * hidden[j];
        }
        outputs[i] = activation_tanh(sum); 
    }
}

void nn_save(NeuralNetwork* nn, const char* filename) {
    FILE* f = fopen(filename, "wb"); // Open in write-binary mode
    if (f != NULL) {
        // Execute a fast memory dump of the entire struct directly to disk
        fwrite(nn, sizeof(NeuralNetwork), 1, f);
        fclose(f);
        printf(">>> Neural weights successfully saved to: %s\n", filename);
    } else {
        printf(">>> ERROR: Unable to save weights to %s\n", filename);
    }
}

int nn_load(NeuralNetwork* nn, const char* filename) {
    FILE* f = fopen(filename, "rb"); // Open in read-binary mode
    if (f != NULL) {
        // Read the struct dump back into memory
        fread(nn, sizeof(NeuralNetwork), 1, f);
        fclose(f);
        return 1;
    }
    return 0;
}
