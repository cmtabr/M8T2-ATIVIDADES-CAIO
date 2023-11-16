# M8T2-ATIVIDADES-CAIO
Esse repositório foi criado com o intuito de armazenar as atividades do Módulo 8 de engenharia da computação

# Ponderada 2
Pressupondo que o usuário já possui o ROS2 instalado em sua máquina, instalaremos algumas dependenciais para o funcionamento correto do projeto.

## Instalações 

### RViz2
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* ros-humble-rmw-cyclonedds-cpp
```
### RMW 
```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.zshrc # ou~/.bashrc 
```

### Gazebo 
```bash
curl -sSL http://get.gazebosim.org | sh
gazebo
```
Espera-se que a instalação funcione corretamente, portanto uma janela deverá ser aberta após o comando "gazebo". Caso isso não ocorra é possível baixar o gazebo no link abaixo:

https://classic.gazebosim.org/download

Agora adicionaremos as variáveis de ambiente utilizadas pelo gazebo ao nosso shell

```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

### Dependências adicionais
```bash
sudo apt-get install ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations
```

Após este passo devemos rodar o comando source no shell utilizado (bash ou zsh), da seguinte forma: 

```bash
source ~/.bashrc # ou ~/.zshrc 
```

### Clonando o projeto
A partir desse momento você tem as dependencias necessárias para rodar o projeto, então vamos aos próximos passos

```bash
git clone https://github.com/cmtabr/M8T2-ATIVIDADES-CAIO.git
```

Vamos entrar no workspace contido no projeto pelo comando

```bash
cd M8T2-ATIVIDADES-CAIO/p2/ws
```

Vamos rodar o comando de build do workspace

```bash
colcon build
```

Vamos rodar o comando source novamente, mas agora na setup do workspace

```bash
source install/setup.zsh # ou setup.bash dependendo do shell
```

### Startup via launch file - Mapeamento
Agora podemos inicializar nossos pacotes através do launch file contido no workspace, assim iremos rodar o comando:

```bash
ros2 launch mapping_package mapping_launch.py
```

Então veremos as telas a seguir e poderemos controlar nosso robô para realizar o mapeamento do mundo do turtlebot:

[Mapping](https://github.com/cmtabr/M8T2-ATIVIDADES-CAIO/assets/99201276/c78044a1-a5c8-44aa-98e1-6d46350a9b2b)

E assim teremos um ambiente mapeado que pode ser salvo através do seguinte comando: 
```bash
ros2 run nav2_map_saver map_saver_cli -f $diretório/de/preferência$
```

### Startup via launch file - Navegação
Agora podemos inicializar o pacote de navegação através do launch file ocntido no workspace, assim iremos rodar o comando: 


```bash
ros2 launch navigation_package navigation_launch.py
```

[Navigation](https://github.com/cmtabr/M8T2-ATIVIDADES-CAIO/assets/99201276/0e5b5a5e-3c2f-40b9-81ec-cccac21a57d8)

## Conclusão

Desta forma, foi possível realizar a atividade como esperado.

# Ponderada 3

## Instalações
É fundamental seguir os passos de instalação da seção anterior de modo que todas as dependências utilizadas estejam instaladas e corretamente configuradas

### Clonando o projeto
A partir desse momento você tem as dependencias necessárias para rodar o projeto, então vamos aos próximos passos

```bash
git clone https://github.com/cmtabr/M8T2-ATIVIDADES-CAIO.git
```

Vamos entrar no workspace contido no projeto pelo comando

```bash
cd M8T2-ATIVIDADES-CAIO/p3/ws
```

Vamos rodar o comando de build do workspace

```bash
colcon build
```

Vamos rodar o comando source novamente, mas agora na setup do workspace

```bash
source install/setup.zsh # ou setup.bash dependendo do shell
```
### Startup via launch file - ChatBot
Agora podemos inicializar nossos pacotes através do launch file contido no workspace, assim iremos rodar o comando:

```bash
ros2 launch chatbot_package chatbot_launch.py
```

Então veremos as telas a seguir e poderemos interagir com o chatbot e controlar nosso robô para realizar a navegação através de linguagem natural:

[ChatBot](https://github.com/cmtabr/M8T2-ATIVIDADES-CAIO/assets/99201276/62e6bcee-c29f-46cf-bb3a-c174ee4a48e7)
