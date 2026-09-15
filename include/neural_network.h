/**
 * @file neural_network.h
 * @brief Definition of the Artificial Neural Network architecture for drone control.
 *
 * This module defines a Feed-Forward Multilayer Perceptron (MLP) used to
 * process environmental inputs (Lidar, relative target position, and battery)
 * to output thrust commands for the drone's physical engine (FMU).
 */

#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

// --- ARCHITECTURE DEFINITION ---

/** 
 * @brief Total number of input nodes.
 * Breakdown: 64 Lidar rays + 3 relative target coordinates (dx, dy, dz) + 1 battery level (normalized).
 */
#define NN_INPUT_SIZE 68  

/** @brief Number of nodes in the single hidden layer. */
#define NN_HIDDEN_SIZE 32

/** @brief Number of output nodes (Thrust X, Thrust Y, Thrust Z). */
#define NN_OUTPUT_SIZE 3  

/**
 * @struct NeuralNetwork
 * @brief Encapsulates the entire weight and bias structure of the MLP.
 *
 * It uses flat 2D arrays for weights and 1D arrays for biases to guarantee 
 * contiguous memory allocation, maximizing caching performance during inference.
 */
typedef struct {
    // Input to Hidden Layer (Layer 1)
    double W1[NN_HIDDEN_SIZE][NN_INPUT_SIZE];   /**< Weight matrix mapping inputs to hidden nodes. */
    double b1[NN_HIDDEN_SIZE];                  /**< Bias vector for hidden nodes. */

    // Hidden to Output Layer (Layer 2)
    double W2[NN_OUTPUT_SIZE][NN_HIDDEN_SIZE];  /**< Weight matrix mapping hidden nodes to outputs. */
    double b2[NN_OUTPUT_SIZE];                  /**< Bias vector for output nodes. */
} NeuralNetwork;

// --- FUNCTIONS ---

/**
 * @brief Initializes the neural network with random weights and biases.
 * 
 * Typically used to seed the initial generation (Gen 0) of the Evolution Strategy.
 * Values are uniformly distributed in the range [-1.0, 1.0].
 *
 * @param nn Pointer to the NeuralNetwork instance to initialize.
 */
void nn_init_random(NeuralNetwork* nn);

/**
 * @brief Performs the forward propagation pass.
 *
 * Calculates the network outputs based on the provided inputs by computing
 * dot products and applying the tanh activation function at each layer.
 *
 * @param nn Pointer to the trained NeuralNetwork instance.
 * @param inputs Array of normalized input values [Size: NN_INPUT_SIZE].
 * @param outputs Array to store the normalized output values [-1.0, 1.0] [Size: NN_OUTPUT_SIZE].
 */
void nn_feedforward(NeuralNetwork* nn, const double inputs[NN_INPUT_SIZE], double outputs[NN_OUTPUT_SIZE]);

/**
 * @brief Dumps the memory structure of the Neural Network to a binary file.
 *
 * @param nn Pointer to the NeuralNetwork instance.
 * @param filename Destination binary file path.
 */
void nn_save(NeuralNetwork* nn, const char* filename);

/**
 * @brief Loads a Neural Network memory structure from a binary file.
 *
 * @param nn Pointer to the NeuralNetwork instance to populate.
 * @param filename Source binary file path.
 * @return 1 on success, 0 if the file could not be read.
 */
int nn_load(NeuralNetwork* nn, const char* filename);

#endif // NEURAL_NETWORK_H
