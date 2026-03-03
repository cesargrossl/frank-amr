cd ~/workspace/frank-amr/projfinal
chmod +x start_robot.sh
./start_robot.sh




✅ Faça assim (sem complicar)
1) Pare o que está rodando agora (seu script)

No terminal do script: Ctrl+C.

Depois confirme que não sobrou nada:

ros2 node list | head
2) Suba o ROBÔ (LiDAR + Odom + TF)

Em um terminal (deixe rodando):

cd ~/workspace/frank-amr/projfinal
./start_robot.sh

(é o script que você já tem para lidar+odom+tf)

3) Suba LOCALIZAÇÃO (map_server + AMCL)

Em outro terminal (deixe rodando):

source /opt/ros/humble/setup.bash
source ~/workspace/frank-amr/ros2_ws/install/setup.bash

ros2 launch nav2_bringup localization_launch.py \
  map:=/home/ubuntu/maps/meu_mapa.yaml \
  use_sim_time:=false



  AQUI PARA RODAR 06051982 

  cd ~/workspace/frank-amr/projfinal
chmod +x start_robot.sh
./start_robot.sh


