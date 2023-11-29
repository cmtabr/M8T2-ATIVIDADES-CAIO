# Interface Gráfica para o Modelo LLAMA2

Este código demonstra a integração de um modelo de linguagem LLAMA2 com uma interface gráfica usando a biblioteca Gradio.

## Funcionalidades

- Utiliza o modelo LLAMA2 para fornecer conselhos de segurança com base em consultas sobre regras, riscos e medidas de segurança.
- Interface gráfica simples para interagir com o assistente de segurança pessoal.

## Pré-requisitos

- Python 3.x
- Biblioteca Gradio
- Biblioteca LangChain (llms, schema, document_loaders, embeddings, text_splitter, vectorstores, chains)

## Como usar

1. Certifique-se de ter Python e todas as bibliotecas necessárias instaladas.
2. Copie o código fornecido em um ambiente Python.
3. Execute o código para iniciar a interface gráfica do assistente de segurança pessoal.

## Explicação do Código

- `archive_loader_and_vectorizer`: Função que carrega documentos de texto de um diretório local, processa-os e cria um sistema de recuperação de informações.
- `chatbot_manager`: Gerencia a interação entre os componentes, como formatação de perguntas, busca de informações relevantes e geração de respostas usando o modelo LLAMA2.
- `screen`: Interface gráfica criada usando a biblioteca Gradio para permitir que o usuário faça consultas sobre segurança industrial.

## Como executar
1. Certifique-se de ter o Python instalado. &darr;
   ```bash 
    $ python --version
    ```

2. Instale o TensorFlow e o Keras.
    ```bash 
    $ pip install langachain gradio 
    ```

3. Execute o código em um ambiente Python.
    ```bash
    $ python chatbot_model.py
    ```

4. Insira uma pergunta na caixa de texto e clique em `Submit`.

5. Aguarde a resposta do assistente de segurança pessoal.

## Observações

- O assistente responde a perguntas relacionadas à segurança industrial com base nos documentos fornecidos. 
[**Link**](https://www.deakin.edu.au/students/study-support/faculties/sebe/abe/workshop/rules-safety)
- A interface gráfica fornece uma maneira intuitiva para interagir com o assistente, inserindo perguntas e recebendo conselhos de segurança.

## Demonstração

[DEMO-P5](https://github.com/cmtabr/M8T2-ATIVIDADES-CAIO/assets/99201276/90638e47-0c3f-4eb0-adee-f58833f77099)

## Licença

Este projeto está licenciado sob a [Creative Commons Zero (CC0) License](https://creativecommons.org/publicdomain/zero/1.0/deed.pt).
