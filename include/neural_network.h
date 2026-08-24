#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

// --- ARCHITECTURE DEFINITION ---
// 64 Lidar Rays + 3 Target Vector Coordinates (dx, dy, dz)
#define NN_INPUT_SIZE 67  
#define NN_HIDDEN_SIZE 32
#define NN_OUTPUT_SIZE 3  // Thrust X, Thrust Y, Thrust Z

// The Neural Network structure holds all the weights and biases
typedef struct {
    // Weights and biases for Input -> Hidden layer
    double W1[NN_HIDDEN_SIZE][NN_INPUT_SIZE];
    double b1[NN_HIDDEN_SIZE];

    // Weights and biases for Hidden -> Output layer
    double W2[NN_OUTPUT_SIZE][NN_HIDDEN_SIZE];
    double b2[NN_OUTPUT_SIZE];
} NeuralNetwork;

// --- FUNCTIONS ---

// Initializes the network with random weights (Used for Generation 0)
void nn_init_random(NeuralNetwork* nn);

// Performs the feedforward pass: calculates outputs based on inputs
void nn_feedforward(NeuralNetwork* nn, const double inputs[NN_INPUT_SIZE], double outputs[NN_OUTPUT_SIZE]);
// Salva i pesi della rete su un file binario
void nn_save(NeuralNetwork* nn, const char* filename);

// Carica i pesi della rete da un file binario (ritorna 1 se successo, 0 se errore)
int nn_load(NeuralNetwork* nn, const char* filename);
#endif