from fastapi import FastAPI, Request, Response, status, Form
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from model.nlp_chatbot import ChatBot

app = FastAPI()

app.mount("/static", StaticFiles(directory="static"), name="static")

templates = Jinja2Templates(directory="templates")

@app.get("/")
def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.post("/submit")
async def submit(request: Request, message: str = Form(...)):
    try:
        chatbot = ChatBot()
        response = chatbot.chat(message)
        return templates.TemplateResponse("index.html", {"request": request, "message": response})
    except:
        return templates.TemplateResponse("index.html", {"request": request, "message": "Não entendi. Poderia reformular a pergunta?"})
