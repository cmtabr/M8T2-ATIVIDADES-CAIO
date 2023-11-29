# Atividade 6 - Implemente o perceptron para identificar portas lógicas AND, NAND, OR e XOR.

# Importings
import numpy as np
import os
from time import sleep
from tqdm import tqdm

# Logic Gates truth tables
# AND Gate - 0 0 = 0, 0 1 = 0, 1 0 = 0, 1 1 = 1
# NAND Gate - 0 0 = 1, 0 1 = 1, 1 0 = 1, 1 1 = 0
# OR Gate - 0 0 = 0, 0 1 = 1, 1 0 = 1, 1 1 = 1
# XOR Gate - 0 0 = 0, 0 1 = 1, 1 0 = 1, 1 1 = 0

class Perceptron:
    def __init__(self, num_inputs: list, epochs: int):
        self.weights = np.random.rand(num_inputs)
        self.bias = 0.0
        self.epochs = epochs

    def step_function(self, x):
        return 1 if x >= 0 else 0

    def train(self, inputs, outputs, learning_rate=0.1):
        for self.epoch in tqdm(range(1, self.epochs + 1), 
                                desc="Treinando", colour='green', 
                                unit=' Épocas'):
            for i in range(len(inputs)):
                input_data = inputs[i]
                target_output = outputs[i]

                weighted_sum = np.dot(input_data, self.weights) + self.bias
                output = self.step_function(weighted_sum)

                error = target_output - output

                self.weights += learning_rate * error * input_data
                self.bias += learning_rate * error

            # self.display_progress()

    def test(self, inputs):
        results = []
        for input_data in inputs:
            weighted_sum = np.dot(input_data, self.weights) + self.bias
            output = self.step_function(weighted_sum)
            results.append(output)
        return results
    
    #def display_progress(self):
    #    print(f"Epoch {self.epoch}/{self.epochs} - Weights: {self.weights}, Bias: {self.bias}")


def main():
    gate_types = ['AND', 'OR', 'NAND', 'XOR']

    print("Escolha o tipo de porta lógica:")
    for i, gate in enumerate(gate_types, start=1):
        print(f"{i}. {gate}")

    choice = int(input("Digite o número correspondente: "))

    epochs = int(input("Digite o número de épocas: "))

    if choice < 1 or choice > len(gate_types):
        print("Escolha inválida. Saindo...")
        exit()

    selected_gate = gate_types[choice - 1]

    if selected_gate == 'AND':
        outputs = np.array([0, 0, 0, 1])
    elif selected_gate == 'OR':
        outputs = np.array([0, 1, 1, 1])
    elif selected_gate == 'NAND':
        outputs = np.array([1, 1, 1, 0])
    elif selected_gate == 'XOR':
        outputs = np.array([0, 1, 1, 0])

    inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
    
    perceptron = Perceptron(num_inputs=len(inputs[0]), epochs=epochs)
    perceptron.train(inputs, outputs)

    results = perceptron.test(inputs)

    print(f"\nPorta selecionada: {selected_gate}\n")
    print(f'Predições: {results}')
    print(f'Valores esperados: {outputs}')
    print(f'Pesos: {perceptron.weights}')
    print(f'Viés: {perceptron.bias}')

    sleep(10)
    os.system('cls' if os.name == 'nt' else 'clear')

if __name__ == "__main__":
    while True:
        main()
