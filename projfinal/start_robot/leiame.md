### 01 Iniciar o processo

chmod +x start_robot.sh
./start_robot.sh

## rever arquivo do mapa pode ocorrer problema de caminho
nano /home/ubuntu/maps/mapa_cros4.yaml
Trocar aqui image: /home/ubuntu/maps/mapa_cros2.pgm


## 02  Suba LOCALIZAÇÃO (map_server + AMCL) 

IMPORTANTE Em outro terminal (deixe rodando):

source /opt/ros/humble/setup.bash
source ~/workspace/frank-amr/ros2_ws/install/setup.bash

ros2 launch nav2_bringup localization_launch.py \
  map:=/home/ubuntu/maps/mapa_cros4.yaml \
  use_sim_time:=false





