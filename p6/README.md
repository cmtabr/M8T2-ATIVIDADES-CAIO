# Perceptron para Portas Lógicas

Este código implementa um perceptron para identificar portas lógicas como AND, NAND, OR e XOR. O perceptron é uma rede neural de uma única camada que pode resolver problemas linearmente separáveis.

## Funcionalidades

- Implementa o perceptron para identificar portas lógicas como AND, NAND, OR e XOR.
- Usa pesos e limiar para aprender e fazer previsões com base nos dados de entrada.

## Pré-requisitos

- Python 3.x
- Biblioteca NumPy

## Como usar

1. Clone o repositório ou baixe o arquivo com o código.
2. Certifique-se de ter o Python e a biblioteca NumPy instalados.
3. Execute o código em um ambiente Python.

## Explicação do Código

- `inputs`: Matriz de entradas para as operações lógicas.
- `Perceptron`: Classe que implementa as funções do perceptron.
  - `predict`: Método para fazer previsões com base nos pesos e no limiar.
  - `train_perceptron`: Método para treinar o perceptron com base nos dados de entrada e saída.

## Problema do XOR

O perceptron é um classificador linear, o que significa que só pode separar classes usando uma linha reta (ou um hiperplano em dimensões superiores). Problemas como as portas lógicas AND, NAND e OR são linearmente separáveis e podem ser resolvidos por um perceptron.

No entanto, a porta lógica XOR não é linearmente separável. Isso significa que não é possível desenhar uma única linha reta para separar as saídas 0 e 1. Não importa como você ajuste os pesos e os limiares, um único neurônio (como o perceptron) não poderá capturar a lógica XOR, pois ela requer uma fronteira de decisão não linear.

Para resolver problemas não linearmente separáveis como XOR, seria necessário recorrer a arquiteturas mais complexas de redes neurais, como redes multicamadas (por exemplo, redes neurais profundas) ou outras técnicas que possam lidar com fronteiras de decisão não lineares.

## Licença

Este projeto está licenciado sob a [Creative Commons Zero (CC0) License](https://creativecommons.org/publicdomain/zero/1.0/deed.pt).
