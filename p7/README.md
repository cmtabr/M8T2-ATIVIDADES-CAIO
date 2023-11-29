# Implementação de uma ConvNet usando o conjunto de dados MNIST

## Visão Geral

Este exemplo demonstra a implementação de uma ConvNet (rede neural convolucional) usando o conjunto de dados MNIST para reconhecimento de dígitos escritos à mão. A ConvNet é construída utilizando a biblioteca Keras, que é uma API de alto nível para redes neurais em Python, sendo executada sobre o framework TensorFlow.

## Conjunto de Dados MNIST

O conjunto de dados MNIST consiste em imagens em escala de cinza de dígitos escritos à mão (0 a 9). Cada imagem tem dimensões de 28x28 pixels e é rotulada com o dígito correspondente. O objetivo é treinar uma rede neural para reconhecer corretamente os dígitos.

## Arquitetura da Rede Neural

A ConvNet utilizada neste exemplo possui a seguinte arquitetura:

- Camada de entrada: `keras.Input(shape=input_shape)`
  - `input_shape`: Formato da entrada da imagem.

- Camada convolucional 2D: `layers.Conv2D(32, kernel_size=(3, 3), activation="relu")`
  - 32 filtros com uma janela de convolução 3x3 e função de ativação ReLU.

- Camada de pooling: `layers.MaxPooling2D(pool_size=(2, 2))`
  - Realiza o pooling máximo com uma janela de pooling 2x2.

- Segunda camada convolucional 2D: `layers.Conv2D(64, kernel_size=(3, 3), activation="relu")`
  - 64 filtros com uma janela de convolução 3x3 e função de ativação ReLU.

- Segunda camada de pooling: `layers.MaxPooling2D(pool_size=(2, 2))`
  - Realiza o pooling máximo com uma janela de pooling 2x2.

- Terceira camada convolucional 2D: `layers.Conv2D(64, kernel_size=(3, 3), activation="relu")`
  - 96 filtros com uma janela de convolução 3x3 e função de ativação ReLU.

- Terceira camada de pooling: `layers.MaxPooling2D(pool_size=(2, 2))`
  - Realiza o pooling máximo com uma janela de pooling 2x2.

- Camada de achatamento: `layers.Flatten()`
  - Transforma a saída das camadas convolucionais em um vetor unidimensional.

- Camada de regularização: `layers.Dropout(0.6)`
  - Aplica dropout com uma taxa de 0.6 para evitar overfitting.

- Camada totalmente conectada: `layers.Dense(num_classes, activation="softmax")`
  - Camada de saída com ativação softmax para classificação multiclasse em `num_classes`.

## Funcionalidades do Código

- Carregamento dos dados MNIST.
- Pré-processamento das imagens (normalização e redimensionamento).
- Definição e compilação da arquitetura da ConvNet.
- Treinamento do modelo utilizando os dados de treinamento.
- Avaliação da precisão do modelo utilizando os dados de teste.
- Visualização de algumas previsões feitas pelo modelo.

## Como Executar

1. Certifique-se de ter o Python instalado. &darr;
   ```bash 
    $ python --version
    ```
2. Instale o TensorFlow e o Keras. &darr;
    ```bash 
    $ pip install tensorflow keras
    ```
    Obs O jupyter notebook conta com uma célula de código para cada instrução listada acima.

3. Abra o jupyter notebook em que o código está inserido e 
clique me `Run All`.

4. Aguarde o treinamento do modelo e a avaliação da precisão.

5. Visualize algumas previsões feitas pelo modelo.

## Demonstração

[DEMO-P7.webm](https://github.com/cmtabr/M8T2-ATIVIDADES-CAIO/assets/99201276/ea48f570-6063-4c97-bcd0-26034818d718)

## Licença

Este código é uma implementação fornecida pela equipe do Keras e está sujeito à [licença do Keras](https://keras.io/about/#license-information).
