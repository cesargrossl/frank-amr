#!/usr/bin/env bash
set -e

# ===== Ajuste esses caminhos se necessário =====
WS_SETUP="$HOME/workspace/frank-amr/ros2_ws/install/setup.bash"
ROS_SETUP="/opt/ros/humble/setup.bash"
ODOM_SCRIPT="$HOME/workspace/frank-amr/projfinal/encoder_odometry_node.py"
LIDAR_PORT="/dev/ttyUSB0"

# TF base_link -> laser (ajuste x y z yaw pitch roll se precisar)
TF_ARGS="0 0 0 0 0 0 base_link laser"

# ==============================================
if [ ! -f "$ROS_SETUP" ]; then
  echo "ERRO: Não achei $ROS_SETUP"
  exit 1
fi
if [ ! -f "$WS_SETUP" ]; then
  echo "ERRO: Não achei $WS_SETUP"
  exit 1
fi
if [ ! -f "$ODOM_SCRIPT" ]; then
  echo "ERRO: Não achei $ODOM_SCRIPT"
  exit 1
fi

source "$ROS_SETUP"
source "$WS_SETUP"

cleanup() {
  echo ""
  echo "Encerrando processos..."
  pkill -f "sllidar_c1_launch.py" || true
  pkill -f "static_transform_publisher.*base_link.*laser" || true
  pkill -f "encoder_odometry_node.py" || true
  pkill -f "async_slam_toolbox_node" || true
}
trap cleanup EXIT

echo "Iniciando LiDAR..."
ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:="$LIDAR_PORT" channel_type:=serial serial_baudrate:=460800 frame_id:=laser \
  > /tmp/lidar.log 2>&1 &

sleep 1

echo "Iniciando TF estática base_link -> laser..."
ros2 run tf2_ros static_transform_publisher $TF_ARGS \
  > /tmp/tf_static.log 2>&1 &

sleep 1

echo "Iniciando Odometria (encoders)..."
python3 "$ODOM_SCRIPT" \
  > /tmp/odom.log 2>&1 &

sleep 1

echo "Iniciando SLAM Toolbox (com odom)..."
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p map_frame:=map \
  -p scan_topic:=/scan \
  > /tmp/slam.log 2>&1 &

sleep 2

echo "Abrindo RViz..."
LIBGL_ALWAYS_SOFTWARE=1 rviz2