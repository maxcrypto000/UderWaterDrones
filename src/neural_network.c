#include "neural_network.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

//ACTIVATION FUNCTIONS

// Hyperbolic tangent: squashes any number into the range [-1.0, 1.0]
static double activation_tanh(double x) {
    return tanh(x);
}

// Helper: Generates a random weight between -1.0 and 1.0
static double random_weight(void) {
    return ((double)rand() / RAND_MAX) * 2.0 - 1.0;
}

//NETWORK FUNCTIONS 

void nn_init_random(NeuralNetwork* nn) {
    // Initialize Input to Hidden Layer (W1, b1)
    for (int i = 0; i < NN_HIDDEN_SIZE; i++) {
        nn->b1[i] = random_weight();
        for (int j = 0; j < NN_INPUT_SIZE; j++) {
            nn->W1[i][j] = random_weight();
        }
    }
    
    // Initialize Hidden to Output Layer (W2, b2)
    for (int i = 0; i < NN_OUTPUT_SIZE; i++) {
        nn->b2[i] = random_weight();
        for (int j = 0; j < NN_HIDDEN_SIZE; j++) {
            nn->W2[i][j] = random_weight();
        }
    }
}

void nn_feedforward(NeuralNetwork* nn, const double inputs[NN_INPUT_SIZE], double outputs[NN_OUTPUT_SIZE]) {
    double hidden[NN_HIDDEN_SIZE];

    // Pass data from Input Layer to Hidden Layer
    for (int i = 0; i < NN_HIDDEN_SIZE; i++) {
        double sum = nn->b1[i];
        for (int j = 0; j < NN_INPUT_SIZE; j++) {
            sum += nn->W1[i][j] * inputs[j];
        }
        hidden[i] = activation_tanh(sum);
    }

    //Pass data from Hidden Layer to Output Layer
    for (int i = 0; i < NN_OUTPUT_SIZE; i++) {
        double sum = nn->b2[i];
        for (int j = 0; j < NN_HIDDEN_SIZE; j++) {
            sum += nn->W2[i][j] * hidden[j];
        }
        // Output is squashed between -1.0 (Full Reverse) and 1.0 (Full Forward)
        outputs[i] = activation_tanh(sum); 
    }
    
}
//save weights and biases
void nn_save(NeuralNetwork* nn, const char* filename) {
    FILE* f = fopen(filename, "wb"); 
    if (f != NULL) {            
        fwrite(nn, sizeof(NeuralNetwork), 1, f);
        fclose(f);
        printf(">>> Pesi neurali salvati con successo in: %s\n", filename);
    } else {
        printf(">>> ERRORE: Impossibile salvare i pesi in %s\n", filename);
    }
}

int nn_load(NeuralNetwork* nn, const char* filename) {
    FILE* f = fopen(filename, "rb"); 
    if (f != NULL) {
        fread(nn, sizeof(NeuralNetwork), 1, f);
        fclose(f);
        return 1;
    }
    return 0;
}