#!/usr/bin/env bash
set -eEo pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/workspace/frank-amr/ros2_ws/install/setup.bash"

ODOM_SCRIPT="$HOME/workspace/frank-amr/projfinal/mapeamento/encoder_odometry_node.py"
MOTOR_SCRIPT="$HOME/workspace/frank-amr/projfinal/tb6612_cmdvel_node.py"
MAP_YAML="$HOME/maps/mapa_cros2.yaml"
NAV2_PARAMS="$HOME/workspace/frank-amr/projfinal/start_robot/nav2_params.yaml"

LIDAR_PORT="/dev/ttyUSB0"
LIDAR_BAUD="460800"
LIDAR_FRAME="laser"

# x y z yaw pitch roll parent child
TF_ARGS="0 0 0 0 0 0 base_link laser"

RVIZ_CFG="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"

die() { echo "ERRO: $1"; exit 1; }
require_file() { [ -f "$1" ] || die "Não achei: $1"; }

require_file "$ROS_SETUP"
require_file "$WS_SETUP"
require_file "$ODOM_SCRIPT"
require_file "$MOTOR_SCRIPT"
require_file "$MAP_YAML"
require_file "$NAV2_PARAMS"

set +u
source "$ROS_SETUP"
source "$WS_SETUP"
set -u 2>/dev/null || true

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
  pkill -f "behavior_server" || true
  pkill -f "waypoint_follower" || true
  pkill -f "velocity_smoother" || true
  pkill -f "lifecycle_manager" || true

  pkill -f "sllidar_c1_launch.py" || true
  pkill -f "static_transform_publisher.*base_link.*laser" || true
  pkill -f "rviz2.*nav2_default_view.rviz" || true
}
trap cleanup EXIT

echo "=== 1) Iniciando LiDAR ==="
ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:="$LIDAR_PORT" \
  channel_type:=serial \
  serial_baudrate:="$LIDAR_BAUD" \
  frame_id:="$LIDAR_FRAME" \
  > /tmp/lidar_nav.log 2>&1 &

sleep 2

echo "=== 2) TF estática base_link -> laser ==="
ros2 run tf2_ros static_transform_publisher $TF_ARGS \
  > /tmp/tf_static_nav.log 2>&1 &

sleep 1

echo "=== 3) Odometria por encoder ==="
python3 "$ODOM_SCRIPT" \
  > /tmp/odom_nav.log 2>&1 &

sleep 1

echo "=== 4) Driver dos motores ouvindo /cmd_vel ==="
sudo -E bash -c "set +u; source '$ROS_SETUP'; source '$WS_SETUP'; python3 '$MOTOR_SCRIPT'" \
  > /tmp/motor_cmdvel.log 2>&1 &

echo "Aguardando /scan..."
timeout 25 bash -c 'until ros2 topic echo /scan --once >/dev/null 2>&1; do sleep 0.2; done' \
  || die "/scan não apareceu. Veja /tmp/lidar_nav.log"

echo "Aguardando /odom..."
timeout 25 bash -c 'until ros2 topic echo /odom --once >/dev/null 2>&1; do sleep 0.2; done' \
  || die "/odom não apareceu. Veja /tmp/odom_nav.log"

echo "=== 5) Localização (map_server + AMCL) ==="
ros2 launch nav2_bringup localization_launch.py \
  map:="$MAP_YAML" \
  use_sim_time:=false \
  params_file:="$NAV2_PARAMS" \
  > /tmp/nav2_localization.log 2>&1 &

echo "Aguardando /map..."
timeout 35 bash -c 'until ros2 topic echo /map --once >/dev/null 2>&1; do sleep 0.3; done' \
  || die "/map não apareceu. Veja /tmp/nav2_localization.log"

sleep 3

echo "=== 6) Navegação ==="
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:="$NAV2_PARAMS" \
  > /tmp/nav2_navigation.log 2>&1 &

sleep 5

echo "=== 7) RViz ==="
LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d "$RVIZ_CFG" \
  > /tmp/rviz_nav.log 2>&1 &

echo ""
echo "=============================================="
echo "ROBO PRONTO."
echo ""
echo "No RViz faça:"
echo "1) 2D Pose Estimate  -> marque a posição inicial do robô"
echo "2) Nav2 Goal         -> clique no destino"
echo ""
echo "Verificações úteis:"
echo "ros2 topic echo /cmd_vel"
echo "ros2 topic echo /amcl_pose"
echo "ros2 topic echo /particle_cloud --once"
echo "ros2 topic hz /scan"
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

wait