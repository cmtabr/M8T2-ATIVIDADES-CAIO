import gradio as gr
from decouple import config
from langchain.llms import ollama
import requests, json

url = config("URL")
model = config("MODEL")
headers = {"Content-Type": "application/json"}

def ask_manager(task):
    parameters = {
        "prompt": task,
        "stream": False,
        "model": model 
    }
    reponse = requests.post(url, headers=headers, data=json.dumps(parameters))
    response = reponse.json()["response"]
    return response

screen = gr.Interface(
    fn=ask_manager,
    title="Welcome to Philosopher's Security Management System!",
    description="Ask the most respectful Philosopher's Security System for advice.",
    inputs="text",
    outputs="text",
    examples=["Quais EPIs são necessários para operar um torno mecânico?",
            "Quais EPIs devo usar em uma cervejaria?",
            "Quais as melhores cervejas do mercado?"
    ],
    theme=gr.themes.Soft()
)

screen.launch()