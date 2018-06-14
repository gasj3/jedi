# robotica

1º Passo:
    - Criar a pasta jedi no diretório: /home/<user>/tiago_public_ws/src/tiago_simulation/

2º Passo:
    $ catkin build jedi

3º Passo:
     Fechar e abrir o terminal.

4º Passo:
    cd tiago_public_ws/
    source ./devel/setup.bash

    roslaunch jedi yoda.launch robot:=titanium world:=robocinV3

5º Passo:
	Adicionar no script movebase.py da fiorela

6º Passo:
	catkin build jedi
	chmod +x movebase.py (estar na pasta do movebase.py)

7º Passo:
	rosrun movebase.py 
