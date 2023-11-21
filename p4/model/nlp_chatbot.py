# ---------------------
# Ponderada 4 - Chatbot
# ---------------------

# Enunciado
"""
Utilizando um LLM (local ou API externa), crie um chatbot simples com instruções customizadas para ajudar um usuário a pesquisar normas de segurança em ambientes industriais. O sistema deve contar com uma interface gráfica e responder de forma sucinta e clara sobre o que lhe foi perguntado.

Exemplo de prompt:
Quais EPIs são necessários para operar um torno mecânico?
"""

# Imports 


class LLM:
    def __init__(self) -> None:
        pass

    def tokenize(self, text: str) -> list:
        pass

    def stem(self, text: str) -> str:
        pass

    def bag_of_words(self, tokenized_sentence: list, all_words: list) -> list:
        pass


class ChatBot(LLM):
    def __init__(self) -> None:
        self.llm = LLM()

    def load_model(self):
        pass

    def chat(self, sentence: str) -> str:
        pass

    

