# Passo a passo — ROS 2 Jazzy em **container** no Ubuntu 25.04 (Raspberry Pi 5)

Este guia cria um ambiente **leve** para **SLAM** (mapeamento) e **navegação por waypoints** usando **ROS 2 Jazzy (Ubuntu 24.04)** dentro de **Docker**, mantendo o host no **Ubuntu 25.04**. O container terá acesso ao **GPIO** (libgpiod) e ao **LiDAR** via serial.

---

## 0) Visão geral

- **Host**: Ubuntu 25.04 (Raspberry Pi 5)
- **Container**: Ubuntu 24.04 com **ROS 2 Jazzy** (imagem `ros:jazzy-ros-base`)
- **Hardware exposto ao container**:
  - `/dev/gpiochip0` (motores via **libgpiod** / L298N / TB6612FNG)
  - `/dev/ttyUSB0` (LiDAR Slamtec C1, por exemplo)
- **Pacotes ROS** instalados no container: `slam_toolbox`, `nav2_bringup`, `nav2_waypoint_follower`, `tf2_tools`
- **Nós próprios**:
  - `amr_motor_driver`: subscreve `/cmd_vel` e aciona GPIO (libgpiod)
  - `amr_lidar`: publica `/scan` a partir do LiDAR
- **Ferramentas cliente**: Rode **RViz** no seu notebook/PC para economizar recursos do Pi.

---

## 1) Pré‑requisitos no host (Ubuntu 25.04)

```bash
sudo apt update
sudo apt install -y docker.io git make
sudo usermod -aG docker,gpio,dialout $USER
# Faça logout/login (ou reinicie) para aplicar os grupos
```

Crie a pasta do workspace:

```bash
mkdir -p ~/amr_ws/src
cd ~/amr_ws
```

---

## 2) `Dockerfile` minimalista (ROS 2 Jazzy + pacotes essenciais)

Crie o arquivo `~/amr_ws/Dockerfile` com o conteúdo abaixo:

```dockerfile
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions build-essential \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-waypoint-follower \
    ros-jazzy-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# Workspace ROS
ENV ROS_DISTRO=jazzy
WORKDIR /root/amr_ws
RUN mkdir -p src
RUN /bin/bash -lc "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build || true"

# Ambiente
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source /root/amr_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
```

---

## 3) `docker-compose.yml` expondo GPIO e LiDAR

Crie o arquivo `~/amr_ws/docker-compose.yml`:

```yaml
version: "3.9"
services:
  amr:
    build: .
    container_name: amr
    network_mode: host
    privileged: true           # simples e funcional; pode endurecer depois
    devices:
      - /dev/gpiochip0:/dev/gpiochip0
      - /dev/ttyUSB0:/dev/ttyUSB0
    volumes:
      - ./src:/root/amr_ws/src  # código do host aparece no container
      - /dev/shm:/dev/shm       # melhora DDS
    environment:
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - ROS_DOMAIN_ID=23        # ajuste se houver outros ROS na rede
    tty: true
```

> **Dica**: Se seu LiDAR aparecer como outro device (ex.: `/dev/ttyACM0`) ou houver múltiplos `gpiochip`, adicione-os em `devices:`.

---

## 4) Construir e iniciar o container

```bash
cd ~/amr_ws
docker compose build
docker compose up -d
docker exec -it amr bash
```

A partir daqui, os comandos são **dentro do container** (prompt `root@…:/root/amr_ws#`).

---

## 5) Preparar o workspace no container

O diretório `~/amr_ws/src` do host está montado em `/root/amr_ws/src` no container.

- Coloque seus pacotes **ROS 2** em `/root/amr_ws/src/` (ex.: `amr_motor_driver/` e `amr_lidar/`).
- Construa o workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd /root/amr_ws
colcon build --symlink-install
source install/setup.bash
```

> Se você ainda não tem esses pacotes, comece vazio e adicione o código depois. (Opcionalmente, pode utilizar um driver pronto de LiDAR para publicar `/scan`.)

---

## 6) Mapeamento (SLAM)

**No container** (três terminais ou panes do tmux):

**Terminal A — LiDAR** (publica `/scan`):
```bash
source /root/.bashrc
ros2 run amr_lidar lidar_node
```

**Terminal B — Motores** (ouve `/cmd_vel` e aciona GPIO):
```bash
source /root/.bashrc
ros2 run amr_motor_driver motor_driver_node
```

**Terminal C — SLAM (slam_toolbox)**:
```bash
source /root/.bashrc
ros2 run slam_toolbox sync_slam_toolbox_node
```

**No seu notebook/PC** (fora do Pi), configure o mesmo `ROS_DOMAIN_ID` e rode **RViz** para visualizar `map`, `tf`, `laser`, etc.

### Salvar o mapa
No **container**:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
"{name: '/root/amr_ws/maps/meu_mapa'}"
# Gera /root/amr_ws/maps/meu_mapa.yaml e .pgm
```

---

## 7) Navegação por waypoints (Nav2)

**No container** (com o mapa salvo):
```bash
source /root/.bashrc
ros2 launch nav2_bringup bringup_launch.py \
  map:=/root/amr_ws/maps/meu_mapa.yaml use_sim_time:=false
```

### Enviar waypoints
No **container** (ou no notebook):
```bash
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
"poses:
- header: {frame_id: map}
  pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}
- header: {frame_id: map}
  pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {z: 0.707, w: 0.707}}"
```

> O Nav2 publicará `/cmd_vel`; o `amr_motor_driver` converterá para GPIO. Sem odometria por encoders, funciona para demos. Para robustez, adicione `odom`/`tf` futuramente.

---

## 8) Alias e inicialização rápida

No **host**, um alias para subir/entrar no container:

```bash
echo 'alias amr-up="cd ~/amr_ws && docker compose up -d && docker exec -it amr bash"' >> ~/.bashrc
source ~/.bashrc
```

---

## 9) (Opcional) Iniciar container no boot via systemd (host)

Crie `/etc/systemd/system/amr-container.service`:

```ini
[Unit]
Description=AMR ROS2 Container
After=network-online.target docker.service
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/bin/docker compose -f /home/%i/amr_ws/docker-compose.yml up -d
ExecStop=/usr/bin/docker compose -f /home/%i/amr_ws/docker-compose.yml down
User=%i
WorkingDirectory=/home/%i/amr_ws

[Install]
WantedBy=multi-user.target
```

Ative:
```bash
sudo systemctl enable amr-container.service
sudo systemctl start amr-container.service
```

> Substitua `%i` pelo seu usuário, se preferir fixo (ex.: `/home/cesargrossl/...`).

---

## 10) Dicas para manter **leve**

- Use **imagem `ros:jazzy-ros-base`** (sem desktop).
- Rode **RViz no notebook** (não no Pi).
- Instale apenas `slam_toolbox`, `nav2_bringup`, `nav2_waypoint_follower`.
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (padrão simples).
- Quando tudo estiver estável, substitua `privileged: true` por regras mais restritivas (`devices:`, grupos `gpio` e `dialout`, e capabilities mínimas).

---

## 11) Troubleshooting rápido

- **`Permission denied` no GPIO/serial**: confirme grupos `gpio`/`dialout` no host e que o container está `privileged` ou com `devices:` mapeados.
- **LiDAR não aparece**: verifique se é `/dev/ttyUSB0` ou `/dev/ttyACM0`, ajuste no `docker-compose.yml` e no seu nó.
- **Descoberta DDS** em redes congestionadas: mantenha o mesmo `ROS_DOMAIN_ID` e considere um discovery server (opcional).
- **Performance**: mantenha RViz fora do Pi; evite processos gráficos no container.

---

**Pronto.** Com isso você mapeia com `slam_toolbox`, salva o `meu_mapa.yaml` e navega com **Nav2 + Waypoint Follower**, tudo no **Ubuntu 25.04** via container ROS 2 **Jazzy**.
