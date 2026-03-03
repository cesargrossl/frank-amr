1) Tornar seu script executável e rodar, não esquecer de eacessar a pasta do workspace\frank\projfinal

No terminal:

chmod +x start_slam.sh
./start_slam.sh


2) No RViz: o que configurar para ver o mapa nascer

Quando abrir o RViz:

Fixed Frame: map

Add → Map

Add → LaserScan (topic /scan)

Add → TF


Andar com o carrinho e salvar o mapa com o comando abaixo


mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/meu_mapa --ros-args \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=10.0



  agora andar no mapa

  🚀 PASSO 1 — Abrir navegação com o mapa salvo

Abra um terminal e rode:

source /opt/ros/humble/setup.bash
source ~/workspace/frank-amr/ros2_ws/install/setup.bash

ros2 launch nav2_bringup navigation_launch.py \
  map:=/home/ubuntu/maps/meu_mapa.yaml \
  use_sim_time:=false

⚠️ Deixe esse terminal aberto.

Se não aparecer erro, está tudo rodando.

rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz