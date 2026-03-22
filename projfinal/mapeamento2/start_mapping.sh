#!/usr/bin/env bash
set -eo pipefail

# ===== Ajuste esses caminhos se necessário =====
WS_SETUP="$HOME/workspace/frank-amr/ros2_ws/install/setup.bash"
ROS_SETUP="/opt/ros/humble/setup.bash"

BASE_DIR="$HOME/workspace/frank-amr/projfinal/mapeamento2"
ODOM_SCRIPT="$BASE_DIR/encoder_odometry_node_corrigido.py"
CTRL_SCRIPT="$BASE_DIR/controlar_carrinho_corrigido.py"
EKF_YAML="$BASE_DIR/ekf_lidar_wheel.yaml"

LIDAR_PORT="/dev/ttyUSB0"
TF_ARGS="0 0 0 3.14159 0 0 base_link laser"

# Encoders / geometria
LEFT_GPIO=16
RIGHT_GPIO=26
WHEEL_RADIUS=0.034
TICKS_PER_REV=22
WHEEL_BASE=0.15

cleanup() {
  echo ""
  echo "Encerrando processos..."
  pkill -f "sllidar_c1_launch.py" || true
  pkill -f "static_transform_publisher.*base_link.*laser" || true
  pkill -f "encoder_odometry_node_corrigido.py" || true
  pkill -f "controlar_carrinho_corrigido.py" || true
  pkill -f "rf2o_laser_odometry_node" || true
  pkill -f "ekf_node" || true
  pkill -f "async_slam_toolbox_node" || true
}
trap cleanup EXIT

require_file() {
  if [ ! -f "$1" ]; then
    echo "ERRO: Não achei $1"
    exit 1
  fi
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "ERRO: comando '$1' não encontrado"
    exit 1
  fi
}

require_file "$ROS_SETUP"
require_file "$WS_SETUP"
require_file "$ODOM_SCRIPT"
require_file "$CTRL_SCRIPT"
require_file "$EKF_YAML"
require_cmd python3
require_cmd ros2

source "$ROS_SETUP"
source "$WS_SETUP"

if ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
  echo "ERRO: pacote robot_localization não encontrado."
  echo "Instale com: sudo apt install ros-humble-robot-localization"
  exit 1
fi

if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
  echo "ERRO: pacote slam_toolbox não encontrado."
  echo "Instale com: sudo apt install ros-humble-slam-toolbox"
  exit 1
fi

if ! ros2 pkg prefix rf2o_laser_odometry >/dev/null 2>&1; then
  echo "ERRO: pacote rf2o_laser_odometry não encontrado."
  echo "Clone e compile no workspace, por exemplo:"
  echo "  cd ~/workspace/frank-amr/ros2_ws/src"
  echo "  git clone -b humble-devel https://github.com/Adlink-ROS/rf2o_laser_odometry.git"
  echo "  cd ~/workspace/frank-amr/ros2_ws && colcon build --symlink-install"
  exit 1
fi

echo "Iniciando LiDAR..."
ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:="$LIDAR_PORT" channel_type:=serial serial_baudrate:=460800 frame_id:=laser \
  > /tmp/lidar.log 2>&1 &
sleep 2

echo "Iniciando TF estática base_link -> laser..."
ros2 run tf2_ros static_transform_publisher $TF_ARGS \
  > /tmp/tf_static.log 2>&1 &
sleep 1

echo "Iniciando odometria das rodas (/odom_wheel)..."
python3 "$ODOM_SCRIPT" --ros-args \
  -p left_gpio:=$LEFT_GPIO \
  -p right_gpio:=$RIGHT_GPIO \
  -p wheel_radius:=$WHEEL_RADIUS \
  -p ticks_per_rev:=$TICKS_PER_REV \
  -p wheel_base:=$WHEEL_BASE \
  -p odom_topic:=/odom_wheel \
  -p frame_odom:=odom \
  -p frame_base:=base_link \
  > /tmp/odom_wheel.log 2>&1 &
sleep 1

echo "Iniciando odometria do LiDAR RF2O (/odom_lidar)..."
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
  -p laser_scan_topic:=/scan \
  -p odom_topic:=/odom_lidar \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p publish_tf:=false \
  -p init_pose_from_topic:="" \
  -p freq:=12.0 \
  > /tmp/odom_lidar.log 2>&1 &
sleep 2

echo "Iniciando EKF (fusão encoder + LiDAR)..."
ros2 run robot_localization ekf_node --ros-args \
  --params-file "$EKF_YAML" \
  > /tmp/ekf.log 2>&1 &
sleep 2

echo "Iniciando controle do carrinho..."
python3 "$CTRL_SCRIPT" \
  > /tmp/control.log 2>&1 &
sleep 1

echo "Iniciando SLAM Toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p map_frame:=map \
  -p scan_topic:=/scan \
  > /tmp/slam.log 2>&1 &
sleep 2

echo "Arquitetura ativa:"
echo "  encoders   -> /odom_wheel"
echo "  LiDAR RF2O -> /odom_lidar"
echo "  EKF        -> TF odom->base_link e /odometry/filtered"
echo "  SLAM       -> TF map->odom"
echo "  controle   -> /wheel_dir"

echo "Abrindo RViz..."
LIBGL_ALWAYS_SOFTWARE=1 rviz2
