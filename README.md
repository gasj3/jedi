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


Criar publisher:
1º Passo:
Criar pasta msg e criar dentro dela um arquivo como o PersonDistance.msg
Indicando os tipos e nomes das variáveis a serem publicadas

2º Passo:
Alterar os package.xml e CMakeLists.txt (se não alterar como estão no git não pega [no caso só adiciona alguns parâmetros])
chmod +x PersonDistance.msg

3º Passo:
Dar catkin build 

4º Passo:
Abrir segundo terminal
source ./devel/setup.bash (Em cada terminal sempre usar este comando)
rosrun jedi closest_person_node.py

5º Passo:
Abrir terceiro terminal
source ./devel/setup.bash (Em cada terminal sempre usar este comando)
rostopic list (checar se o nó closest_person foi criado, se sim é só sucesso kk)
rostopic echo closest_person (Checa qual a pessoa mais próxima do robô, a posição das pessoas estão com um valor aproximado)

6º Passo (Opcional):

Mover o robô, seja com o movebase ou algo assim com executando os outros 3 terminais pra realmente checar qual pessoa ele está mais perto




Comandos Show:(rodar com  o gazebo rodando)

rostopic echo -c /amcl_pose
rostopic info /amcl_pose
rostopic list
rosnode info movebase.py


rostopic type /amcl_pose
geometry_msgs/PoseWithCovarianceStamped

mkdir msg

