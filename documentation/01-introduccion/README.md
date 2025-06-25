# Capítulo 1: Introducción a ROS2

## Tabla de Contenidos

1. [¿Qué es ROS2?](#qué-es-ros2)
2. [¿Por qué ROS2 para Robótica?](#por-qué-ros2-para-robótica)
3. [Diferencias entre ROS1 y ROS2](#diferencias-entre-ros1-y-ros2)
4. [Instalación y Configuración](#instalación-y-configuración)
5. [Primer Contacto con el eCar](#primer-contacto-con-el-ecar)
6. [Arquitectura del Sistema eCar](#arquitectura-del-sistema-ecar)

## ¿Qué es ROS2?

ROS2 (Robot Operating System 2) es un framework de código abierto para el desarrollo de sistemas robóticos. A pesar de su nombre, ROS2 no es un sistema operativo tradicional, sino un middleware que proporciona:

- **Comunicación entre procesos**: Permite que diferentes componentes del robot se comuniquen
- **Herramientas de desarrollo**: Bibliotecas y utilidades para desarrollo robótico
- **Abstracción de hardware**: Interfaz común para diferentes dispositivos
- **Ecosystem**: Gran cantidad de paquetes y bibliotecas disponibles

### Características Principales

**Distribuido**: Los componentes pueden ejecutarse en diferentes máquinas
```bash
# Nodo en máquina A
ros2 run tadeo_ecar_perception lidar_processor_node

# Nodo en máquina B  
ros2 run tadeo_ecar_control wheel_controller_node
```

**Modular**: Cada funcionalidad es un módulo independiente
```bash
# Cada componente del eCar es un nodo separado
ros2 node list
/lidar_processor_node
/camera_processor_node
/wheel_controller_node
/navigation_controller_node
```

**Multilenguaje**: Soporta C++, Python, y otros lenguajes
```cpp
// C++ Node
class LidarProcessor : public rclcpp::Node {
    // Implementación del procesador LiDAR
};
```

```python
# Python Node
class CameraProcessor(Node):
    # Implementación del procesador de cámara
```

## ¿Por qué ROS2 para Robótica?

### Ventajas en Desarrollo Robótico

**1. Reutilización de Código**
- Los nodos desarrollados para el eCar pueden usarse en otros robots
- Comunidad global compartiendo soluciones

**2. Debugging y Desarrollo**
- Herramientas integradas de visualización y debugging
- Capacidad de ejecutar componentes por separado

**3. Escalabilidad**
- Desde robots simples hasta sistemas multi-robot
- El eCar puede expandirse con nuevos sensores sin reestructurar

**4. Tiempo Real**
- Soporte para sistemas críticos
- Importante para control del eCar a altas velocidades

### Caso de Uso: Robot eCar 4WD4WS

El robot eCar demuestra las ventajas de ROS2:

```
Sistema eCar = 12 paquetes independientes
├── Percepción (LiDAR, Cámara, IMU)
├── Control (4 ruedas direccionales)
├── Navegación (Nav2 stack)
├── Seguridad (Emergency stop, colisiones)
├── Comportamientos (Decision making)
└── Monitoreo (System health)
```

Cada componente puede:
- Desarrollarse independientemente
- Debuggearse por separado
- Reemplazarse sin afectar otros
- Ejecutarse en diferentes máquinas

## Diferencias entre ROS1 y ROS2

### Arquitectura de Comunicación

**ROS1: Master centralizado**
```
roscore (Master)
    ├── Node A
    ├── Node B
    └── Node C
```
Problema: Si roscore falla, todo el sistema se detiene.

**ROS2: Sistema distribuido (DDS)**
```
Node A ←→ Node B
   ↕       ↕
Node C ←→ Node D
```
Ventaja: No hay punto único de falla.

### Ejemplo Práctico con el eCar

**ROS1 (problemático)**:
```bash
# Terminal 1: Requerido antes que nada
roscore

# Terminal 2: Solo después de roscore
rosrun tadeo_ecar_control wheel_controller

# Si roscore se cae, todo se detiene
```

**ROS2 (robusto)**:
```bash
# Terminal 1: Directo, sin dependencias
ros2 run tadeo_ecar_control wheel_controller_node

# Terminal 2: Independiente
ros2 run tadeo_ecar_perception lidar_processor_node

# Los nodos se conectan automáticamente
```

### Quality of Service (QoS)

**ROS1**: Comunicación básica TCP/UDP
**ROS2**: QoS configurable por tópico

```yaml
# Configuración QoS para el eCar
sensor_data:
  reliability: best_effort  # Para sensores de alta frecuencia
  history: keep_last
  depth: 10

control_commands:
  reliability: reliable     # Para comandos críticos
  history: keep_last
  depth: 1
```

## Instalación y Configuración

### Requisitos del Sistema

```bash
# Verificar Ubuntu version
lsb_release -a
# Requerido: Ubuntu 22.04 LTS (Jammy)
```

### Instalación de ROS2 Humble

```bash
# 1. Configurar repositorios
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. Instalar ROS2
sudo apt update
sudo apt install ros-humble-desktop-full

# 3. Configurar entorno
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 4. Instalar herramientas adicionales
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update
```

### Verificación de Instalación

```bash
# Verificar instalación
ros2 --version
# Salida esperada: ros2 cli version: 0.18.x

# Verificar comunicación
ros2 topic list
# Debería mostrar tópicos básicos sin errores
```

### Configuración para el Proyecto eCar

```bash
# Instalar dependencias específicas del eCar
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-behaviortree-cpp-v3
sudo apt install ros-humble-foxglove-bridge
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Primer Contacto con el eCar

### Clonar y Compilar el Proyecto

```bash
# 1. Crear workspace
mkdir -p ~/ros2/ecar-ws/src
cd ~/ros2/ecar-ws/src

# 2. Clonar repositorio (requiere acceso)
git clone https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws.git .

# 3. Instalar dependencias
cd ~/ros2/ecar-ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Compilar
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 5. Configurar entorno
source install/setup.bash
```

### Primer Ejemplo: Verificar Nodos

```bash
# Terminal 1: Listar paquetes del eCar
ros2 pkg list | grep tadeo_ecar
# Salida:
# tadeo_ecar_msgs
# tadeo_ecar_interfaces
# tadeo_ecar_config
# ... (12 paquetes total)

# Terminal 2: Verificar ejecutables
ros2 pkg executables tadeo_ecar_control
# Salida:
# tadeo_ecar_control wheel_controller_node
# tadeo_ecar_control vehicle_dynamics_node
# tadeo_ecar_control four_wheel_steering_controller_node
```

### Primer Nodo del eCar

```bash
# Ejecutar nodo de control de ruedas
ros2 run tadeo_ecar_control wheel_controller_node

# En otro terminal, verificar que está activo
ros2 node list
# Salida: /wheel_controller_node

# Verificar información del nodo
ros2 node info /wheel_controller_node
```

### Primer Tópico del eCar

```bash
# Ver tópicos activos
ros2 topic list
# Ejemplo de salida:
# /cmd_vel
# /odom
# /wheel_states
# /system_health

# Escuchar datos de ruedas
ros2 topic echo /wheel_states

# En otro terminal, publicar comando
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## Arquitectura del Sistema eCar

### Visión General

El sistema eCar está diseñado como un ejemplo completo de robot autónomo usando ROS2:

```
                    eCar 4WD4WS Robot
                         |
            ┌─────────────────────────────┐
            |     ROS2 Node Network       |
            └─────────────────────────────┘
                         |
    ┌────────────────────┼────────────────────┐
    |                    |                    |
┌───▼───┐           ┌────▼────┐          ┌────▼────┐
│Sensors│           │ Control │          │Planning │
│Nodes  │           │ Nodes   │          │ Nodes   │
└───────┘           └─────────┘          └─────────┘
    |                    |                    |
├─LiDAR               ├─Wheels             ├─Global
├─Camera              ├─Steering           ├─Local  
├─IMU                 └─Dynamics           └─Recovery
└─GPS
```

### Flujo de Datos Típico

1. **Sensores → Percepción**
```bash
# LiDAR datos → Procesamiento
/scan → lidar_processor_node → /processed_scan
```

2. **Percepción → Planificación**
```bash
# Obstáculos detectados → Planificador
/obstacles → global_planner_node → /global_path
```

3. **Planificación → Control**
```bash
# Path generado → Controlador
/global_path → controller_node → /cmd_vel
```

4. **Control → Actuadores**
```bash
# Comandos → Ruedas
/cmd_vel → wheel_controller_node → /wheel_commands
```

### Paquetes del Sistema eCar

| Paquete | Propósito | Nodos Principales |
|---------|-----------|-------------------|
| `tadeo_ecar_msgs` | Mensajes personalizados | N/A |
| `tadeo_ecar_interfaces` | Servicios y acciones | N/A |
| `tadeo_ecar_config` | Configuraciones | N/A |
| `tadeo_ecar_control` | Control 4WD4WS | wheel_controller_node |
| `tadeo_ecar_perception` | Sensores | lidar_processor_node |
| `tadeo_ecar_safety` | Seguridad | emergency_stop_node |
| `tadeo_ecar_localization` | Localización | ekf_localization_node |
| `tadeo_ecar_slam` | Mapeo | slam_node |
| `tadeo_ecar_planning` | Planificación | global_planner_node |
| `tadeo_ecar_navigation` | Navegación | navigation_controller_node |
| `tadeo_ecar_behavior` | Comportamientos | behavior_manager_node |
| `tadeo_ecar_bringup` | Sistema completo | N/A (launch files) |

### Ejemplo de Interacción

```bash
# Terminal 1: Iniciar sistema básico
ros2 launch tadeo_ecar_bringup minimal_system.launch.py

# Terminal 2: Verificar nodos activos
ros2 node list

# Terminal 3: Monitorear comunicación
ros2 topic hz /scan
ros2 topic hz /cmd_vel
ros2 topic hz /odom

# Terminal 4: Enviar comando de navegación
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'
```

## Conclusiones del Capítulo

En este capítulo hemos cubierto:

1. **Fundamentos de ROS2**: Middleware para robótica distribuida
2. **Ventajas sobre ROS1**: Sin master, QoS, tiempo real
3. **Instalación completa**: Entorno listo para desarrollo
4. **Primer contacto**: Compilación y ejecución del proyecto eCar
5. **Arquitectura**: Sistema modular de 12 paquetes

### Próximos Pasos

En el siguiente capítulo profundizaremos en:
- Conceptos de workspace y paquetes
- Compilación con colcon
- Variables de entorno
- Organización de proyectos ROS2

### Verificación de Conocimientos

Antes de continuar, asegúrate de poder:

```bash
# 1. Compilar el proyecto sin errores
colcon build

# 2. Ejecutar un nodo del eCar
ros2 run tadeo_ecar_control wheel_controller_node

# 3. Listar tópicos activos
ros2 topic list

# 4. Ver información de un nodo
ros2 node info /wheel_controller_node
```

Si tienes dificultades con alguno de estos comandos, revisa la sección de instalación antes de continuar.

## Referencias

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [DDS Specification](https://www.omg.org/spec/DDS/)
- [Repositorio eCar](https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws)