# Implementação de STT e TTS usando o Whisper 

Este script utiliza a API da OpenAI para realizar serviços de tradução de áudio para texto, tradução de texto para áudio, e tradução de texto de um idioma para outro. O código é estruturado em funções que executam tarefas específicas, proporcionando uma abordagem modular para a implementação.

## Bibliotecas Importadas
- `decouple`: Utilizado para lidar com variáveis de ambiente, neste caso, para armazenar a chave da API da OpenAI de forma segura.
- `openai`: Biblioteca oficial da OpenAI para interação com suas APIs.
- `pathlib`: Facilita a manipulação de caminhos de arquivos e diretórios.
- `playsound`: Permite reproduzir arquivos de áudio no sistema.
- `time`: Utilizado para introduzir pausas no fluxo do programa.

## Variáveis de Ambiente
- `OPENAI_API_KEY`: Chave de API da OpenAI, armazenada de forma segura usando a biblioteca `decouple`.

## Configurações
- `speech_file_path`: Caminho do arquivo de áudio gerado durante o processo.
- `client`: Objeto OpenAI utilizado para interagir com suas APIs.

## Funções
### `speech_to_text()`
Converte um arquivo de áudio para texto utilizando a API da OpenAI.

### `text_to_speech(text)`
Converte um texto para áudio utilizando a API da OpenAI e salva o resultado em um arquivo.

### `save(text)`
Salva um texto em um arquivo de texto.

### `translate(text)`
Traduz um texto de um idioma para outro usando a API de chat da OpenAI.

### `play()`
Reproduz o arquivo de áudio gerado.

### `main()`
Função principal que orquestra todo o processo, desde a conversão de áudio para texto até a tradução e reprodução do resultado.

## Utilização
1. Insira a chave da API da OpenAI no arquivo `.env` utilizando a variável `OPENAI_API_KEY`.
```bash
type nul > .env 
set OPENAI_API_KEY=<sua chave>
set > .env
```

2. Baixe as dependências do projeto.
```bash
pip install -r requirements.txt
```

3. Execute o script Python.
```bash
python lesson.py
```

## Observações
- O código atual está configurado para funcionar com um arquivo de áudio chamado "katyusha.mp3" na pasta "p8". Certifique-se de ajustar conforme necessário.

## Demo 
Abaixo podemos ver a demonstração da ferramente funcionando.