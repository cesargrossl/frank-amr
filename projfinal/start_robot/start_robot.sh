#!/usr/bin/env bash
set -eEo pipefail

# ===================== CONFIG =====================
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/workspace/frank-amr/ros2_ws/install/setup.bash"

ODOM_SCRIPT="$HOME/workspace/frank-amr/projfinal/mapeamento/encoder_odometry_node.py"
MOTOR_SCRIPT="$HOME/workspace/frank-amr/projfinal/tb6612_cmdvel_node.py"
MAP_YAML="$HOME/maps/meu_mapa.yaml"

LIDAR_PORT="/dev/ttyUSB0"
LIDAR_BAUD="460800"
LIDAR_FRAME="laser"

# TF base_link -> laser (x y z yaw pitch roll base_frame child_frame)
TF_ARGS="0 0 0 0 0 0 base_link laser"

RVIZ_CFG="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
# ==================================================

die() { echo "ERRO: $1"; exit 1; }
require_file() { [ -f "$1" ] || die "Não achei: $1"; }

require_file "$ROS_SETUP"
require_file "$WS_SETUP"
require_file "$ODOM_SCRIPT"
require_file "$MOTOR_SCRIPT"
require_file "$MAP_YAML"

# --- SOURCE ROS (sem nounset) ---
# (alguns setup.bash do ROS referenciam variáveis não definidas)
set +u
source "$ROS_SETUP"
source "$WS_SETUP"
set -u 2>/dev/null || true   # tenta reativar se existir, mas não é necessário

cleanup() {
  echo ""
  echo "Encerrando processos..."

  pkill -f "tb6612_cmdvel_node.py" || true
  pkill -f "encoder_odometry_node.py" || true

  pkill -f "localization_launch.py" || true
  pkill -f "navigation_launch.py" || true

  pkill -f "map_server" || true
  pkill -f "amcl" || true
  pkill -f "bt_navigator" || true
  pkill -f "controller_server" || true
  pkill -f "planner_server" || true
  pkill -f "recoveries_server" || true
  pkill -f "waypoint_follower" || true

  pkill -f "sllidar_c1_launch.py" || true
  pkill -f "static_transform_publisher.*base_link.*laser" || true

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

sleep 1

echo "=== 3.5) Motor driver (TB6612FNG) lendo /cmd_vel ==="
# Roda com sudo e re-sourca ROS dentro do sudo para garantir rclpy
sudo -E bash -c "set +u; source '$ROS_SETUP'; source '$WS_SETUP'; python3 '$MOTOR_SCRIPT'" \
  > /tmp/motor_cmdvel.log 2>&1 &

echo "Aguardando /scan..."
timeout 25 bash -c 'until ros2 topic echo /scan --once >/dev/null 2>&1; do sleep 0.2; done' \
  || die "/scan não apareceu. Veja /tmp/lidar_nav.log"

echo "Aguardando /odom..."
timeout 25 bash -c 'until ros2 topic echo /odom --once >/dev/null 2>&1; do sleep 0.2; done' \
  || die "/odom não apareceu. Veja /tmp/odom_nav.log"

echo "=== 4) Iniciando LOCALIZAÇÃO (map_server + AMCL) ==="
ros2 launch nav2_bringup localization_launch.py \
  map:="$MAP_YAML" \
  use_sim_time:=false \
  > /tmp/nav2_localization.log 2>&1 &

echo "Aguardando /map..."
timeout 35 bash -c 'until ros2 topic echo /map --once >/dev/null 2>&1; do sleep 0.3; done' \
  || die "/map não apareceu. Veja /tmp/nav2_localization.log"

echo "Aguardando /amcl_pose (pode depender do 2D Pose Estimate)..."
timeout 10 bash -c 'until ros2 topic echo /amcl_pose --once >/dev/null 2>&1; do sleep 0.3; done' \
  || echo "AVISO: /amcl_pose ainda não apareceu (normal antes de dar 2D Pose Estimate)."

echo "=== 5) Iniciando NAVEGAÇÃO (planner/controller/bt) ==="
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  > /tmp/nav2_navigation.log 2>&1 &

sleep 3

echo "=== 6) Abrindo RViz (Nav2 default) ==="
LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d "$RVIZ_CFG" \
  > /tmp/rviz_nav.log 2>&1 &

echo ""
echo "=============================================="
echo "OK! Agora faça no RViz (OBRIGATÓRIO):"
echo "  1) 2D Pose Estimate  -> clique onde o robô está e arraste a seta"
echo "  2) Nav2 Goal         -> clique no destino e arraste a seta"
echo ""
echo "Para verificar se o Nav2 está mandando velocidade:"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 topic echo /cmd_vel"
echo ""
echo "Logs:"
echo "  /tmp/lidar_nav.log"
echo "  /tmp/tf_static_nav.log"
echo "  /tmp/odom_nav.log"
echo "  /tmp/motor_cmdvel.log"
echo "  /tmp/nav2_localization.log"
echo "  /tmp/nav2_navigation.log"
echo "  /tmp/rviz_nav.log"
echo "=============================================="
echo ""

wait