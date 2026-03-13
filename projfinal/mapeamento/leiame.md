1) Tornar seu script executável e rodar, não esquecer de eacessar a pasta do workspace\frank\projfinal

No terminal:

chmod +x start_slam.sh
./start_slam.sh

Depois que abrir rodar

source /opt/ros/humble/setup.bash
source ~/workspace/frank-amr/ros2_ws/install/setup.bash

python3 ~/workspace/frank-amr/projfinal/mapeamento/controlar_carrinho.py

2) No RViz: o que configurar para ver o mapa nascer

Quando abrir o RViz:

Fixed Frame: map

Add → Map

Add → LaserScan (topic /scan)

Add → TF


Andar com o carrinho e salvar o mapa com o comando abaixo


mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/mapa_cros --ros-args \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=10.0