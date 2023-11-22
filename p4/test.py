from langchain.llms import ollama
ollama = ollama.Ollama(base_url='http://localhost:11434',
model="philos")
print(ollama("why is the sky blue"))