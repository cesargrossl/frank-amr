# Inciar com o comando 192.168.1.50

source /opt/ros/humble/setup.bash
source ~/workspace/frank-amr/ros2_ws/install/setup.bash


# Terminal 1 — LiDAR C1 (deixa rodando)

source /opt/ros/humble/setup.bash
source ~/workspace/frank-amr/ros2_ws/install/setup.bash

ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:=/dev/ttyUSB0 channel_type:=serial serial_baudrate:=460800 frame_id:=laser


# Terminal 2 — TF estática base_link → laser (deixa rodando)

source /opt/ros/humble/setup.bash
source ~/workspace/frank-amr/ros2_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser

# Terminal 3 — Odometria (encoders) (deixa rodando)

source /opt/ros/humble/setup.bash
source ~/workspace/frank-amr/ros2_ws/install/setup.bash

cd ~/workspace/frank-amr/projfinal
python3 encoder_odometry_node.py

# Terminal 4 — SLAM Toolbox com odom real (deixa rodando)

source /opt/ros/humble/setup.bash
source ~/workspace/frank-amr/ros2_ws/install/setup.bash

ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p map_frame:=map \
  -p scan_topic:=/scan

# Terminal 5 — RViz (visualizar e mapear)

source /opt/ros/humble/setup.bash
source ~/workspace/frank-amr/ros2_ws/install/setup.bash

LIBGL_ALWAYS_SOFTWARE=1 rviz2

# Quando terminar: salvar o mapa

source /opt/ros/humble/setup.bash
source ~/workspace/frank-amr/ros2_ws/install/setup.bash

ros2 run nav2_map_server map_saver_cli -f ~/meu_mapa

Cria:

~/meu_mapa.yaml

~/meu_mapa.pgm


# Arquivo Automático 
./start_mapping.sh

# comnado no power shel window 
wsl -d Ubuntu-22.04