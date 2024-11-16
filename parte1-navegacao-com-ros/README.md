# Navegação de Robô em Labirinto com Algoritmo A*

Este repositório contém a implementação de um robô que navega por um labirinto utilizando o **algoritmo A\***, resolvendo a parte 2 da atividade ponderada P1, em que o objetivo é planejar a rota do robô até o seu alvo utilizando o mapa.

## Objetivo

O objetivo dessa tarefa foi implementar a navegação de um robô em um labirinto utilizando o **algoritmo A\*** para otimizar a rota do robô a partir do mapa do labirinto. O algoritmo é utilizado para planejar a trajetória do robô do ponto inicial até o alvo, evitando obstáculos e garantindo a rota mais curta possível.

## Estrutura do Projeto

O pacote principal da atividade está estruturado da seguinte maneira:

- **src/**: Contém o código-fonte da implementação do robô e do algoritmo de navegação.
- **CMakeLists.txt**: Arquivo de configuração do CMake para compilar o código C++.
- **package.xml**: Arquivo de configuração do pacote ROS2.
- **README.md**: Este arquivo, que descreve o projeto.

## Dependências

Para rodar o projeto, você precisa ter o ROS 2 instalado, além das bibliotecas necessárias para compilar o código C++:

## Descrição do Algoritmo A\*

O **algoritmo A\*** é um algoritmo de busca heurística utilizado para encontrar o caminho mais curto entre dois pontos em um gráfico, no nosso caso, entre a posição do robô e o alvo no labirinto. Ele utiliza uma função de custo total estimado que combina a distância percorrida até o momento e uma estimativa da distância restante até o destino, sendo eficaz para problemas como navegação de robôs em ambientes desconhecidos ou parcialmente conhecidos.

O algoritmo A\* é composto por três principais componentes:

- **g(n)**: o custo do caminho do ponto inicial até o ponto atual.
- **h(n)**: a estimativa do custo do caminho do ponto atual até o objetivo (função heurística).
- **f(n) = g(n) + h(n)**: o custo total estimado para o ponto atual.

O algoritmo busca explorar os pontos com menor custo total e expande aqueles que estão mais próximos de alcançar o objetivo. Esse processo continua até encontrar o caminho mais curto ou determinar que não há caminho possível.

## Como Funciona

### Navegação com Mapa (Algoritmo A\*)

Nesta parte da implementação, o robô utiliza um mapa do labirinto para planejar sua rota até o alvo. O algoritmo A\* é utilizado para calcular a rota ótima, considerando obstáculos e espaços livres. O processo inclui:

1. Obtenção do mapa através do serviço ROS2 `/get_map`.
2. Conversão do mapa para uma estrutura de dados utilizável pelo algoritmo A\*.
3. Planejamento da rota do robô, considerando obstáculos e espaços livres.
4. Envio de comandos de movimento para o robô utilizando o serviço `/move_command` até que ele chegue ao destino.

## Como Rodar o Projeto

### 1. Pré-requisitos

Certifique-se de ter o ROS2 instalado. Siga as instruções de instalação do ROS2 para o seu sistema operacional [aqui](https://docs.ros.org/en/foxy/Installation.html).

### 2. Instalação

Clone o repositório:
```bash
git clone https://github.com/ifferreira/m08/
cd m08/parte1-navegacao-com-ros/workspace
```

### 3. Compilar o projeto
Depois de clonar o repositório, compile o projeto utilizando o colcon:
```bash
colcon build
```

### 4. Configurar o ambiente   
Após a compilação, configure o ambiente:
```bash
source install/setup.bash
```

### 5. Executar os nós
Primeiro, execute o nó do labirinto para inicializar a simulação:
```bash
ros2 run cg maze
```

Em seguida, execute o nó do solver (algoritmo A*):
```bash
ros2 run maze_solver maze_solver
```
