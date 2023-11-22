# Ponderada 4 
Presumindo que o usuário já possui Linux nativo ou está utilizando o WSL, seguiremos com o passo a passo para utilização do presente sistema.

## Instalações 

### Dependências do Python
```bash
pip install -r requirements.txt
```

### Ollama 
```bash
curl https://ollama.ai/install.sh | sh
```

### Clonando o projeto
A partir desse momento você tem as dependencias necessárias para rodar o projeto, então vamos aos próximos passos

```bash
git clone https://github.com/cmtabr/M8T2-ATIVIDADES-CAIO.git
```

### Variáveis de ambiente
Após este passo devemos rodar o comando para exportar nossas variáveis de ambiente para o arquivo dotenv que será criado, da seguinte forma: 

```bash
cd ~/M8T2-ATIVIDADES-CAIO/p4
touch .env
echo "export MODEL=philos" >> .env
echo "export URL=http://localhost:11434/api/generate" >> .env
```

## Instância 

### Build do modelo
```bash
ollama create philos -f Modelfile
```

## Rodando o projeto 
Após estes passos básicos, podemos rodar a atividade através do seguinte comando
```bash
python3 main.py
```

Obtendo a tela a seguir: 

[p4.webm](https://github.com/cmtabr/M8T2-ATIVIDADES-CAIO/assets/99201276/64c03252-9b73-4dca-a52b-9b2b704bd10e)

Alternativamente podemos utilizar o modelo no terminal, sem a interface gráfica, rodando o comando: 
```bash
ollama run philos
```
