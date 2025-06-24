# Tadeo eCar - Autonomous Robot Packages

Sistema completo de robot autónomo 4WD4WS desarrollado en ROS2 Humble.

## 📦 Paquetes del Sistema

### Core Packages

**tadeo_ecar_description** 
Descripción URDF del robot, meshes STL, configuración RVIZ y launches base.

**tadeo_ecar_bringup**
Launch files principales para inicializar todo el sistema del robot.

**tadeo_ecar_config**
Archivos de configuración, parámetros y calibración del robot.

### Navigation Stack

**tadeo_ecar_navigation**
Nodos de navegación autónoma, planificación de rutas y evitación de obstáculos.

**tadeo_ecar_slam**
Implementación SLAM para mapeo y localización simultánea.

**tadeo_ecar_localization**
Fusión sensorial y localización del robot en el mapa.

**tadeo_ecar_planning**
Algoritmos de planificación de trayectorias y movimientos.

### Control & Perception

**tadeo_ecar_control**
Controladores de bajo nivel para motores y actuadores 4WD4WS.

**tadeo_ecar_perception**
Procesamiento de sensores: LiDAR, cámara, odometría.

**tadeo_ecar_behavior**
Máquinas de estado y comportamientos autónomos de alto nivel.

**tadeo_ecar_safety**
Sistemas de seguridad, monitoreo y paradas de emergencia.

### Interfaces

**tadeo_ecar_msgs**
Mensajes personalizados específicos del robot.

**tadeo_ecar_interfaces**
Servicios y acciones custom para el sistema.

## 🏗️ Arquitectura

```
tadeo_ecar_bringup
├── Inicializa description
├── Lanza control
├── Activa perception
├── Inicia navigation
└── Habilita safety
```

## 🚀 Uso

```bash
# Inicializar robot completo
ros2 launch tadeo_ecar_bringup robot.launch.py

# Solo simulación
ros2 launch tadeo_ecar_description simulation.launch.py

# Navegación autónoma
ros2 launch tadeo_ecar_navigation autonomous.launch.py
```

---
**Stack:** ROS2 Humble  < /dev/null |  **Robot:** 4WD4WS | **Sensores:** LiDAR + Cámara
