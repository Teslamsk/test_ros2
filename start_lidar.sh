#!/bin/bash
# ============================================================
#  start_lidar.sh — запуск Velodyne VLP-16 для сканирования
# ============================================================

set -e

# ---------- Настройки ----------
LIDAR_IP="192.168.1.201"
HOST_IP="192.168.1.100"
IFACE="eth0"
WS_DIR="$HOME/velodyne_ws"
LAUNCH_FILE="$WS_DIR/src/velodyne/velodyne/launch/velodyne-all-nodes-VLP16-launch.py"

# ---------- Цветной вывод ----------
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== Запуск Velodyne VLP-16 ===${NC}"

# ---------- 1. Проверяем workspace ----------
if [ ! -d "$WS_DIR" ]; then
    echo -e "${RED}Ошибка: workspace $WS_DIR не найден${NC}"
    exit 1
fi

# ---------- 2. Настраиваем сеть ----------
echo -e "${YELLOW}[1/4] Настройка сети на $IFACE...${NC}"

sudo ip link set "$IFACE" up

if ! ip addr show "$IFACE" | grep -q "$HOST_IP"; then
    echo "  Назначаем IP $HOST_IP на $IFACE"
    sudo ip addr add "$HOST_IP/24" dev "$IFACE"
else
    echo "  IP $HOST_IP уже назначен"
fi

# Явный маршрут до лидара через eth0
sudo ip route replace "$LIDAR_IP/32" dev "$IFACE" 2>/dev/null || true

# ---------- 3. Проверяем интерфейс (без tcpdump!) ----------
echo -e "${YELLOW}[2/4] Проверка интерфейса...${NC}"

if ip link show "$IFACE" | grep -q "state UP"; then
    echo -e "  ${GREEN}Интерфейс $IFACE поднят${NC}"
else
    echo -e "  ${RED}Интерфейс $IFACE не поднят${NC}"
    exit 1
fi

# ---------- 4. Активируем ROS ----------
echo -e "${YELLOW}[3/4] Активация ROS 2...${NC}"
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

# ---------- 5. Запускаем ----------
echo -e "${YELLOW}[4/4] Запуск драйвера и визуализации...${NC}"

ros2 launch "$LAUNCH_FILE" device_ip:="$LIDAR_IP" &
LAUNCH_PID=$!

# Ждём, чтобы драйвер успел подняться и начать публиковать
sleep 8

# Запускаем rviz2
rviz2 &
RVIZ_PID=$!

# ---------- Завершение ----------
cleanup() {
    echo -e "\n${YELLOW}Остановка...${NC}"
    kill $RVIZ_PID 2>/dev/null || true
    kill $LAUNCH_PID 2>/dev/null || true
    wait 2>/dev/null || true
    echo -e "${GREEN}Готово.${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo -e "${GREEN}=== Всё запущено. Ctrl+C для остановки ===${NC}"
wait $LAUNCH_PID
