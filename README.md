# robotica

1º Passo:
    - Criar a pasta jedi no diretório: /home/<user>/tiago_public_ws/src/tiago_simulation/

2º Passo:
    $ catkin build jedi

3º Passo:
     Fechar e abrir o terminal.

4º Passo: mapeamento
    cd tiago_public_ws/
    source ./devel/setup.bash

    roslaunch jedi yoda.launch robot:=titanium world:=robocinV3
4.1º Passo: navegação
    cd tiago_public_ws/
    source ./devel/setup.bash

    roslaunch jedi obiwan.launch robot:=titanium world:=robocinV3

5º Passo:
	Adicionar no script movebase.py da fiorela

6º Passo:
	catkin build jedi
	chmod +x movebase.py (estar na pasta do movebase.py)

7º Passo:
	rosrun jedi scripts/movebase.py 


Comandos Show:(rodar com  o gazebo rodando)

rostopic echo -c /amcl_pose
rostopic info /amcl_pose
rostopic list
rosnode info movebase.py

