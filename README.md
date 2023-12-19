# Laboratórios 5 e 6

Clone este repositório no seu workspace do ROS2. Caso seu workspace seja num diretório diferente de /home/ros2_ws, edite-o na primeira linha abaixo.

```sh
$ROS_HOME=~/ros2_ws
cd $ROS_HOME
colcon build
```

Para conveniência, foi criado um script chamado `start`, que inicializa e configura o ambiente para executar o laboratório.

Mova o arquivo `start` do diretório `src` para o seu diretório HOME, ou seu diretório raiz do workspace. Caso coloque no diretório raiz do workspace, lembre-se de entrar no diretório do workspace sempre que for executar o arquivo.

`mv "$ROS_HOME"/src/start ~/`

Ao dar o comando `source start` o seu terminal exibirá uma mensagem de confirmação, e será adicionado o prefixo [ros2] no seu prompt.

Ex:
Antes:

`laz@vostro`

Depois:

`[ros2]laz@vostro`

Dessa forma, é explicito que o terminal está com o ambiente do ROS configurado.

## Laboratório 5

Para executar o laboratório 5, abra um terminal e execute o comando `source start`, em seguida, execute o comando `spawn_turtle`.

`spawn_turtle` é equivalente à

```sh
cd ${ROS_WS};
source ${ROS_WS}/install/setup.bash;
ros2 run turtlesim turtlesim_node'
```

Em seguida, abra um novo terminal, execute `source start` e execute o comando `lab5_control`. Este comando é equivalente à

```sh
cd ${ROS_WS}
source ${ROS_WS}/install/setup.bash
ros2 run turtle_control_lvn turtle_control'
```

Observe o `turtlebot` ir de encontro ao objetivo.


## Laboratório 6

Não é necessário fechar o laboratório 5.

Caso tenha fechado, execute os mesmos passos acima, e em seguida, em um novo terminal, execute `source start` e execute o comando `turtle_goal`. Este comando é equivalente à

```sh
cd ${ROS_WS}
source ${ROS_WS}/install/setup.bash
ros2 run turtle_control_lvn turtle_goal'
```

Veja nos logs deste terminal a posição atual do `turtlebot`, o objetivo atual, e a distância euclidiana até o objetivo atual. Ao chegar no objetivo, automaticamente será dado outro objetivo. Um timer de 2 segundos atualiza os dados mencionados.