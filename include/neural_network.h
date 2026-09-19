#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H


// Total inputs: 64 Lidar rays + 3 relative target coordinates (dx, dy, dz) + 1 battery
#define NN_INPUT_SIZE 68  
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



/**
 * @brief Initializes the network with random weights (Used for Generation 0)
 *
 * @param nn the neural network to initialize
 */
void nn_init_random(NeuralNetwork* nn);

/**
 * @brief Performs the feedforward pass: calculates outputs based on inputs
 *
 * @param nn the neural network to use
 * @param inputs the inputs to the neural network
 * @param outputs the outputs of the neural network
 */
void nn_feedforward(NeuralNetwork* nn, const double inputs[NN_INPUT_SIZE], double outputs[NN_OUTPUT_SIZE]);

/**
 * @brief Saves the neural network weights to a file
 *
 * @param nn the neural network to save
 * @param filename the filename to save the neural network to
 */
void nn_save(NeuralNetwork* nn, const char* filename);

/**
 * @brief Loads the neural network weights from a file
 *
 * @param nn the neural network to load
 * @param filename the filename to load the neural network from
 * @return 1 if successful, 0 if error
 */
int nn_load(NeuralNetwork* nn, const char* filename);
#endif