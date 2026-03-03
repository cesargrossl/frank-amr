#!/usr/bin/env bash
set -e

# ===================== CONFIG =====================
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/workspace/frank-amr/ros2_ws/install/setup.bash"

ODOM_SCRIPT="$HOME/workspace/frank-amr/projfinal/encoder_odometry_node.py"
MAP_YAML="$HOME/maps/meu_mapa.yaml"

LIDAR_PORT="/dev/ttyUSB0"
LIDAR_BAUD="460800"
LIDAR_FRAME="laser"

# TF base_link -> laser (AJUSTE x y z yaw pitch roll se precisar)
TF_ARGS="0 0 0 0 0 0 base_link laser"

# RViz config do Nav2
RVIZ_CFG="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"

# ==================================================

die() { echo "ERRO: $1"; exit 1; }

[ -f "$ROS_SETUP" ] || die "Não achei $ROS_SETUP"
[ -f "$WS_SETUP" ] || die "Não achei $WS_SETUP"
[ -f "$ODOM_SCRIPT" ] || die "Não achei $ODOM_SCRIPT"
[ -f "$MAP_YAML" ] || die "Não achei o mapa: $MAP_YAML (salve o mapa ou ajuste MAP_YAML)"

source "$ROS_SETUP"
source "$WS_SETUP"

cleanup() {
  echo ""
  echo "Encerrando processos..."
  pkill -f "navigation_launch.py" || true
  pkill -f "map_server" || true
  pkill -f "amcl" || true
  pkill -f "bt_navigator" || true
  pkill -f "controller_server" || true
  pkill -f "planner_server" || true
  pkill -f "recoveries_server" || true

  pkill -f "sllidar_c1_launch.py" || true
  pkill -f "static_transform_publisher.*base_link.*laser" || true
  pkill -f "encoder_odometry_node.py" || true
  pkill -f "rviz2.*nav2_default_view.rviz" || true
}
trap cleanup EXIT

echo "=== 1) Iniciando LiDAR (/scan) ==="
ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:="$LIDAR_PORT" channel_type:=serial serial_baudrate:="$LIDAR_BAUD" frame_id:="$LIDAR_FRAME" \
  > /tmp/lidar_nav.log 2>&1 &

sleep 1

echo "=== 2) TF estática base_link -> laser ==="
ros2 run tf2_ros static_transform_publisher $TF_ARGS \
  > /tmp/tf_static_nav.log 2>&1 &

sleep 1

echo "=== 3) Odometria (encoders) (/odom + TF odom->base_link) ==="
python3 "$ODOM_SCRIPT" \
  > /tmp/odom_nav.log 2>&1 &

# Espera /scan e /odom aparecerem
echo "Aguardando /scan..."
timeout 15 bash -c 'until ros2 topic echo /scan --once >/dev/null 2>&1; do sleep 0.2; done' \
  || die "/scan não apareceu. Veja /tmp/lidar_nav.log"

echo "Aguardando /odom..."
timeout 15 bash -c 'until ros2 topic echo /odom --once >/dev/null 2>&1; do sleep 0.2; done' \
  || die "/odom não apareceu. Veja /tmp/odom_nav.log"

echo "=== 4) Iniciando Nav2 (map_server + AMCL + navigation) ==="
ros2 launch nav2_bringup navigation_launch.py \
  map:="$MAP_YAML" \
  use_sim_time:=false \
  > /tmp/nav2.log 2>&1 &

sleep 2

echo "=== 5) Abrindo RViz (Nav2 default) ==="
LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d "$RVIZ_CFG" \
  > /tmp/rviz_nav.log 2>&1 &

echo ""
echo "=============================================="
echo "OK! Agora faça no RViz:"
echo "  1) 2D Pose Estimate  -> clique onde o robô está e arraste a seta"
echo "  2) Nav2 Goal         -> clique no destino e arraste a seta"
echo ""
echo "Dica: ver /cmd_vel:"
echo "  ros2 topic echo /cmd_vel"
echo "Logs:"
echo "  /tmp/lidar_nav.log  /tmp/odom_nav.log  /tmp/nav2.log  /tmp/rviz_nav.log"
echo "=============================================="
echo ""

# Mantém o script vivo até CTRL+C
wait