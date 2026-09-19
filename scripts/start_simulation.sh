#!/bin/bash
###############################################################################
# Moving Platform UAV Landing - Başlatma Scripti
# 
# Bu script tüm bileşenleri doğru sırada başlatır.
# Her bileşen ayrı bir terminal/process olarak çalışır.
#
# Kullanım: ./start_simulation.sh
###############################################################################

set -e

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Proje dizinleri
PROJECT_DIR="$HOME/my_projects/catkin_ws/src/moving_platform_uav_landing"
CATKIN_WS="$HOME/my_projects/catkin_ws"
ARDUPILOT_DIR="$HOME/ardupilot"

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  Moving Platform UAV Landing Simulator${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

# Ortam ayarları
setup_env() {
    source /opt/ros/noetic/setup.bash
    source $CATKIN_WS/devel/setup.bash
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PROJECT_DIR/models:$HOME/my_projects/ardupilot_gazebo/models:$HOME/catkin_ws/src/iq_sim/models
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$HOME/my_projects/ardupilot_gazebo/build
    export GAZEBO_MODEL_DATABASE_URI="" # Online model aramasını kapatarak açılışı hızlandırır
    
    # WSL2 GPU Hızlandırma Ayarları (Kasma/FPS Sorununu Çözer)
    export LIBGL_ALWAYS_INDIRECT=0
    export GALLIUM_DRIVER=d3d12
    export MESA_GL_VERSION_OVERRIDE=3.3
    export MESA_GLSL_VERSION_OVERRIDE=330
    export SVGA_VGPU10=0
}

# Eski süreçleri temizle
cleanup() {
    echo -e "${YELLOW}Eski süreçler temizleniyor...${NC}"
    pkill -9 -f gzserver 2>/dev/null || true
    pkill -9 -f gzclient 2>/dev/null || true
    pkill -9 -f ArduCopter 2>/dev/null || true
    pkill -9 -f mavproxy 2>/dev/null || true
    pkill -9 -f sim_vehicle 2>/dev/null || true
    pkill -9 -f rosmaster 2>/dev/null || true
    pkill -9 -f roscore 2>/dev/null || true
    pkill -9 -f roslaunch 2>/dev/null || true
    pkill -9 -f arducopter 2>/dev/null || true
    pkill -9 -f drone_pose_controller 2>/dev/null || true
    pkill -9 -f move_platform 2>/dev/null || true
    sleep 2
    echo -e "${GREEN}Temizlik tamamlandı.${NC}"
}

# Argüman kontrolü
if [ "$1" == "stop" ]; then
    cleanup
    exit 0
fi

GUI_ARG=""
if [ "$1" == "headless" ]; then
    GUI_ARG="gui:=false"
    echo -e "${YELLOW}Headless mod aktif. Gazebo arayüzü (GUI) açılmayacak. Performans artacak!${NC}"
fi

# Adım 1: Gazebo + ROS
start_gazebo() {
    echo -e "${GREEN}[1/4] Gazebo + ROS başlatılıyor...${NC}"
    setup_env
    roslaunch arkhe_gazebo drone.launch $GUI_ARG &
    GAZEBO_PID=$!
    echo -e "${GREEN}  Gazebo PID: $GAZEBO_PID${NC}"
    echo -e "${YELLOW}  Gazebo'nun tamamen yüklenmesi bekleniyor (20 saniye)...${NC}"
    sleep 20
}

# Adım 2: ArduPilot SITL
start_sitl() {
    echo -e "${YELLOW}[2/4] ArduPilot SITL ve MAVProxy başlatılıyor...${NC}"
    setup_env
    /home/harun/ardupilot/build/sitl/bin/arducopter -S --model gazebo-iris --speedup 1 --defaults /home/harun/ardupilot/Tools/autotest/default_params/copter.parm,/home/harun/ardupilot/Tools/autotest/default_params/gazebo-iris.parm --sim-address=127.0.0.1 -I0 --home -35.363261,149.165230,584,353 > /dev/null 2>&1 &
    SITL_PID=$!
    
    echo -e "${YELLOW}  SITL'in baslamasi bekleniyor (5 saniye)...${NC}"
    sleep 5
    
    # MAVProxy'i başlat ve UDP 14550'ye yönlendir (daemon modunda)
    mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550 --sitl 127.0.0.1:5501 --daemon --non-interactive > /dev/null 2>&1 &
    MAVPROXY_PID=$!
    
    echo -e "${GREEN}  SITL PID: $SITL_PID, MAVProxy PID: $MAVPROXY_PID${NC}"
    echo -e "${YELLOW}  SITL'in hazir olmasi bekleniyor (15 saniye)...${NC}"
    sleep 15
}

# Adım 3: Drone Controller
start_controller() {
    echo -e "${GREEN}[3/4] Drone Controller başlatılıyor...${NC}"
    setup_env
    rosrun arkhe_gazebo drone_pose_controller.py &
    CTRL_PID=$!
    
    echo -e "${GREEN}  Controller PID: $CTRL_PID${NC}"
    echo -e "${YELLOW}  Controller başlatıldı...${NC}"
    sleep 5
}

# Adım 4: Platform Hareket
start_platform() {
    echo -e "${GREEN}[4/4] Platform hareket başlatılıyor...${NC}"
    setup_env
    rosrun arkhe_gazebo move_platform.py &
    PLAT_PID=$!
    echo -e "${GREEN}  Platform PID: $PLAT_PID${NC}"
}

# Ana akış
cleanup
start_gazebo
start_sitl
start_controller
start_platform

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${GREEN}  Tüm bileşenler başlatıldı!${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "Durdurmak için: ${RED}Ctrl+C${NC} veya ${RED}./start_simulation.sh stop${NC}"

# Ctrl+C ile tüm süreçleri durdur
trap cleanup EXIT
wait
