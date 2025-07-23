#!/bin/bash

"""
Script de inicio rápido para el sistema Tadeo eCar
Proporciona opciones de lanzamiento simplificadas
"""

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Función para mostrar banner
show_banner() {
    echo -e "${CYAN}"
    echo "=================================================================="
    echo "          SISTEMA ECAR 4WD4WS - SEMILLERO DE ROBÓTICA"
    echo "    https://tadeoroboticsgroup.github.io/TadeoRoboticsGroup/"
    echo "=================================================================="
    echo -e "${NC}"
}

# Función para mostrar ayuda
show_help() {
    echo -e "${YELLOW}Uso: $0 [OPCIÓN]${NC}"
    echo ""
    echo "Opciones disponibles:"
    echo "  sim          Lanzar en modo simulación completa"
    echo "  hardware     Lanzar en modo hardware real"
    echo "  nav          Lanzar solo navegación (requiere mapa)"
    echo "  slam         Lanzar con SLAM para mapeo"
    echo "  minimal      Lanzar configuración mínima"
    echo "  monitor      Lanzar solo el monitor del sistema"
    echo "  help         Mostrar esta ayuda"
    echo ""
    echo "Ejemplos:"
    echo "  $0 sim           # Simulación completa con Gazebo"
    echo "  $0 hardware      # Hardware real con todos los sistemas"
    echo "  $0 slam          # Modo mapeo con SLAM"
    echo ""
}

# Función para verificar dependencias
check_dependencies() {
    echo -e "${BLUE}Verificando dependencias...${NC}"
    
    # Verificar ROS2
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}ERROR: ROS2 no está instalado o no está en el PATH${NC}"
        exit 1
    fi
    
    # Verificar workspace
    if [ ! -f "/home/thinkpad-wsl/ros2/tadeo-eCar-ws/install/setup.bash" ]; then
        echo -e "${RED}ERROR: Workspace no compilado. Ejecutar colcon build primero${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✓ Dependencias verificadas${NC}"
}

# Función para source del workspace
source_workspace() {
    echo -e "${BLUE}Configurando workspace...${NC}"
    source /opt/ros/humble/setup.bash
    source /home/thinkpad-wsl/ros2/tadeo-eCar-ws/install/setup.bash
    echo -e "${GREEN}✓ Workspace configurado${NC}"
}

# Función para lanzar simulación
launch_simulation() {
    echo -e "${GREEN}🚀 Lanzando Tadeo eCar en modo SIMULACIÓN${NC}"
    echo -e "${YELLOW}Esto incluye: Gazebo, navegación, SLAM, comportamientos y RViz${NC}"
    sleep 2
    
    ros2 launch tadeo_ecar_bringup tadeo_ecar_simulation.launch.py \
        use_rviz:=true \
        enable_navigation:=true \
        enable_slam:=true \
        slam_mode:=slam_toolbox \
        world_file:=empty_world_ignition.sdf
}

# Función para lanzar hardware
launch_hardware() {
    echo -e "${GREEN}🤖 Lanzando Tadeo eCar en modo HARDWARE${NC}"
    echo -e "${YELLOW}Esto incluye: drivers, navegación, comportamientos y monitoreo${NC}"
    sleep 2
    
    ros2 launch tadeo_ecar_bringup tadeo_ecar_hardware.launch.py \
        use_foxglove:=true \
        auto_calibrate:=true \
        enable_safety:=true \
        enable_navigation:=true \
        battery_monitoring:=true
}

# Función para lanzar solo navegación
launch_navigation() {
    echo -e "${GREEN}🧭 Lanzando sistema de NAVEGACIÓN${NC}"
    echo -e "${YELLOW}Requiere mapa previamente generado${NC}"
    sleep 2
    
    ros2 launch tadeo_ecar_bringup tadeo_ecar_full_system.launch.py \
        use_simulation:=false \
        navigation_mode:=full \
        slam_mode:=none \
        behavior_mode:=state_machine_only \
        use_rviz:=true
}

# Función para lanzar SLAM
launch_slam() {
    echo -e "${GREEN}🗺️  Lanzando sistema de SLAM${NC}"
    echo -e "${YELLOW}Para crear mapas del entorno${NC}"
    sleep 2
    
    ros2 launch tadeo_ecar_bringup tadeo_ecar_full_system.launch.py \
        use_simulation:=false \
        navigation_mode:=localization_only \
        slam_mode:=slam_toolbox \
        behavior_mode:=state_machine_only \
        use_rviz:=true
}

# Función para lanzar configuración mínima
launch_minimal() {
    echo -e "${GREEN}⚡ Lanzando configuración MÍNIMA${NC}"
    echo -e "${YELLOW}Solo sistemas básicos y telemetría${NC}"
    sleep 2
    
    ros2 launch tadeo_ecar_bringup tadeo_ecar_full_system.launch.py \
        use_simulation:=false \
        navigation_mode:=none \
        slam_mode:=none \
        behavior_mode:=state_machine_only \
        safety_mode:=emergency_only \
        use_rviz:=false
}

# Función para lanzar monitor
launch_monitor() {
    echo -e "${GREEN}📊 Lanzando MONITOR del sistema${NC}"
    sleep 1
    
    python3 /home/thinkpad-wsl/ros2/tadeo-eCar-ws/src/tadeo_ecar_bringup/scripts/system_monitor.py
}

# Función para cleanup al salir
cleanup() {
    echo -e "\n${YELLOW}Deteniendo sistema Tadeo eCar...${NC}"
    # Matar procesos relacionados
    pkill -f "ros2 launch"
    pkill -f "gazebo"
    pkill -f "rviz"
    echo -e "${GREEN}✓ Sistema detenido${NC}"
    exit 0
}

# Trap para cleanup
trap cleanup SIGINT SIGTERM

# Función principal
main() {
    show_banner
    
    # Verificar argumentos
    if [ $# -eq 0 ]; then
        echo -e "${RED}ERROR: Se requiere especificar un modo de lanzamiento${NC}"
        show_help
        exit 1
    fi
    
    case "$1" in
        "sim"|"simulation")
            check_dependencies
            source_workspace
            launch_simulation
            ;;
        "hardware"|"hw")
            check_dependencies
            source_workspace
            launch_hardware
            ;;
        "nav"|"navigation")
            check_dependencies
            source_workspace
            launch_navigation
            ;;
        "slam"|"mapping")
            check_dependencies
            source_workspace
            launch_slam
            ;;
        "minimal"|"min")
            check_dependencies
            source_workspace
            launch_minimal
            ;;
        "monitor"|"mon")
            check_dependencies
            source_workspace
            launch_monitor
            ;;
        "help"|"-h"|"--help")
            show_help
            ;;
        *)
            echo -e "${RED}ERROR: Opción desconocida: $1${NC}"
            show_help
            exit 1
            ;;
    esac
}

# Ejecutar función principal
main "$@"