# Atividade 6 - Implemente o perceptron para identificar portas lógicas AND, NAND, OR e XOR.

# Importings
import numpy as np

# Logic Gates truth tables
# AND Gate - 0 0 = 0, 0 1 = 0, 1 0 = 0, 1 1 = 1
# NAND Gate - 0 0 = 1, 0 1 = 1, 1 0 = 1, 1 1 = 0
# OR Gate - 0 0 = 0, 0 1 = 1, 1 0 = 1, 1 1 = 1
# XOR Gate - 0 0 = 0, 0 1 = 1, 1 0 = 1, 1 1 = 0

inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])

class Perceptron:
    def __init__(self):
        pass

    def predict(self, inputs, threshold, weights):
        total = sum(w * i for w, i in zip(weights, inputs))
        return 1 if total > threshold else 0

    def train_perceptron(self, inputs, outputs, learning_rate, epochs):
        weights = np.array([1.0] * len(inputs[0]))
        threshold = 0.0
        for epoch in range(epochs):
            for input, output in zip(inputs, outputs):
                prediction = self.predict(input, threshold, weights)
                for i in range(len(weights)):
                    weights[i] += learning_rate * (output - prediction) 
                threshold += learning_rate * (output - prediction ) 
            print("Threshold: ", threshold)
            print("Epoch: ", epoch)
            print("Weights: ", weights)
        return weights, threshold

if __name__ == "__main__":
    perceptron = Perceptron()
    print("What kind of logic gate do you want to train? (AND, NAND, OR, XOR)")
    logic_gate = input().upper()
    if logic_gate == "AND":
        outputs = np.array([0, 0, 0, 1])
    elif logic_gate == "NAND":
        outputs = np.array([1, 1, 1, 0])
    elif logic_gate == "OR":
        outputs = np.array([0, 1, 1, 1])
    elif logic_gate == "XOR":
        outputs = np.array([0, 1, 1, 0])
    else:
        print("Invalid input. Please choose from (AND, NAND, OR, XOR).")
        exit()

    weights, threshold = perceptron.train_perceptron(inputs, outputs, 0.3, 10)
    print(f'Prediction for the input {logic_gate}:', perceptron.predict([1, 1], threshold, weights))
