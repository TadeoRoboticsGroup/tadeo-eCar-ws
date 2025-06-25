# Capítulo 2: Conceptos Básicos de ROS2

## Tabla de Contenidos

1. [Arquitectura Distribuida](#arquitectura-distribuida)
2. [Workspace y Paquetes](#workspace-y-paquetes)
3. [Compilación con Colcon](#compilación-con-colcon)
4. [Variables de Entorno](#variables-de-entorno)
5. [Estructura de Archivos](#estructura-de-archivos)
6. [Herramientas de Desarrollo](#herramientas-de-desarrollo)

## Arquitectura Distribuida

### Concepto de Distribución

ROS2 utiliza una arquitectura distribuida donde múltiples procesos (nodos) se comunican entre sí sin un coordinador central. Esto es fundamental para entender cómo funciona el sistema eCar.

```
Máquina Principal (eCar)          Máquina de Control Remoto
┌─────────────────────┐          ┌──────────────────────┐
│ lidar_processor     │ ←──────→ │ rviz2                │
│ wheel_controller    │          │ rqt_graph            │
│ navigation_node     │          │ teleop_keyboard      │
└─────────────────────┘          └──────────────────────┘
```

### Ventajas para el eCar

**1. Modularidad**
```bash
# Cada funcionalidad es independiente
ros2 run tadeo_ecar_perception lidar_processor_node    # Procesa LiDAR
ros2 run tadeo_ecar_control wheel_controller_node      # Controla ruedas
ros2 run tadeo_ecar_safety emergency_stop_node        # Monitorea seguridad
```

**2. Escalabilidad**
```bash
# Agregar nuevo sensor sin modificar código existente
ros2 run tadeo_ecar_perception camera_processor_node   # Nuevo nodo de cámara
# Se conecta automáticamente al sistema
```

**3. Tolerancia a Fallos**
```bash
# Si un nodo falla, otros continúan
# Ejemplo: Si lidar_processor_node se detiene, wheel_controller_node sigue funcionando
```

### Descubrimiento Automático

Los nodos en ROS2 se descubren automáticamente usando DDS (Data Distribution Service):

```bash
# Terminal 1: Iniciar nodo de control
ros2 run tadeo_ecar_control wheel_controller_node

# Terminal 2: Iniciar nodo de percepción  
ros2 run tadeo_ecar_perception lidar_processor_node

# Los nodos se conectan automáticamente sin configuración adicional
```

### Dominios ROS

Los dominios permiten separar diferentes robots o sistemas:

```bash
# eCar en dominio 0 (default)
export ROS_DOMAIN_ID=0
ros2 run tadeo_ecar_control wheel_controller_node

# Otro robot en dominio 1
export ROS_DOMAIN_ID=1  
ros2 run other_robot_control controller_node

# No hay interferencia entre dominios
```

## Workspace y Paquetes

### Concepto de Workspace

Un workspace es un directorio que contiene uno o más paquetes ROS2. El proyecto eCar es un workspace completo.

```
tadeo-eCar-ws/                    # Workspace root
├── src/                          # Source space
│   ├── tadeo_ecar_control/       # Paquete 1
│   ├── tadeo_ecar_perception/    # Paquete 2
│   └── ...                       # Más paquetes
├── build/                        # Build space (generado)
├── install/                      # Install space (generado)
└── log/                          # Log space (generado)
```

### Anatomía de un Paquete ROS2

Cada paquete tiene una estructura estándar:

```
tadeo_ecar_control/
├── package.xml                   # Metadatos del paquete
├── CMakeLists.txt               # Instrucciones de compilación
├── src/                         # Código fuente C++
│   ├── wheel_controller_node.cpp
│   └── vehicle_dynamics_node.cpp
├── include/                     # Headers C++
│   └── tadeo_ecar_control/
│       └── wheel_controller.hpp
├── launch/                      # Archivos de lanzamiento
│   └── control_system.launch.py
├── config/                      # Archivos de configuración
│   └── control_params.yaml
└── README.md                    # Documentación
```

### package.xml

El archivo `package.xml` define las propiedades del paquete:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>tadeo_ecar_control</name>
  <version>1.0.0</version>
  <description>Vehicle control system for eCar 4WD4WS robot</description>
  
  <maintainer email="ing.marioalvarezvallejo@gmail.com">Semillero Robótica</maintainer>
  <license>Proprietary</license>
  
  <!-- Build dependencies -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- Runtime dependencies -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt

Define cómo compilar el paquete:

```cmake
cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create executables
add_executable(wheel_controller_node src/wheel_controller_node.cpp)
ament_target_dependencies(wheel_controller_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Install targets
install(TARGETS
  wheel_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

### Tipos de Paquetes

**1. Paquetes de Biblioteca (Library)**
```bash
# Ejemplo: tadeo_ecar_msgs
# Contiene definiciones de mensajes, servicios, acciones
# No tiene nodos ejecutables
```

**2. Paquetes de Nodos (Node)**
```bash
# Ejemplo: tadeo_ecar_control
# Contiene nodos ejecutables
# Implementa funcionalidad específica
```

**3. Paquetes de Configuración (Configuration)**
```bash
# Ejemplo: tadeo_ecar_config
# Contiene parámetros y configuraciones
# Archivos launch y configuración YAML
```

**4. Paquetes Meta (Metapackage)**
```bash
# Ejemplo: tadeo_ecar_bringup
# Orquesta otros paquetes
# Launch files del sistema completo
```

## Compilación con Colcon

### ¿Qué es Colcon?

Colcon es la herramienta de compilación estándar para ROS2. Reemplaza a catkin de ROS1.

### Compilación Básica

```bash
# Navegar al workspace
cd ~/ros2/ecar-ws

# Compilar todos los paquetes
colcon build

# Compilar paquetes específicos
colcon build --packages-select tadeo_ecar_control tadeo_ecar_perception

# Compilar con enlaces simbólicos (recomendado para desarrollo)
colcon build --symlink-install
```

### Opciones de Compilación

```bash
# Compilación en paralelo (más rápido)
colcon build --parallel-workers 4

# Compilación con output detallado
colcon build --event-handlers console_direct+

# Continuar compilación aunque algunos paquetes fallen
colcon build --continue-on-error

# Compilar solo paquetes modificados
colcon build --packages-up-to tadeo_ecar_navigation
```

### Verificación de Compilación

```bash
# Verificar éxito de compilación
echo $?
# 0 = éxito, >0 = error

# Ver log detallado
colcon build --event-handlers console_direct+ | tee build.log

# Verificar ejecutables generados
ls install/tadeo_ecar_control/lib/tadeo_ecar_control/
# wheel_controller_node
# vehicle_dynamics_node
```

### Debugging de Problemas de Compilación

```bash
# Limpiar build anterior
rm -rf build/ install/ log/

# Compilar un paquete específico con verbose
colcon build --packages-select tadeo_ecar_control --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

# Ver dependencias no satisfechas
rosdep check --from-paths src --ignore-src
```

## Variables de Entorno

### Variables Esenciales

**ROS_DOMAIN_ID**: Separación de redes
```bash
export ROS_DOMAIN_ID=0    # Dominio por defecto
export ROS_DOMAIN_ID=42   # Dominio personalizado para el eCar
```

**ROS_LOCALHOST_ONLY**: Restricción a localhost
```bash
export ROS_LOCALHOST_ONLY=1   # Solo comunicación local
export ROS_LOCALHOST_ONLY=0   # Comunicación en red (default)
```

**RMW_IMPLEMENTATION**: Middleware DDS
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp      # FastRTPS (default)
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp     # CycloneDX
```

### Configuración del Workspace

Cada vez que abres una terminal nueva:

```bash
# 1. Source ROS2
source /opt/ros/humble/setup.bash

# 2. Source workspace del eCar
source ~/ros2/ecar-ws/install/setup.bash

# Automatizar en ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2/ecar-ws/install/setup.bash" >> ~/.bashrc
```

### Variables Específicas del eCar

```bash
# Configuración para el eCar
export ROS_DOMAIN_ID=10              # Dominio específico eCar
export ECAR_CONFIG_PATH="$HOME/ros2/ecar-ws/src/tadeo_ecar_config"
export ECAR_MAPS_PATH="$HOME/ecar_maps"

# En archivo launch
os.environ['ECAR_CONFIG_PATH'] = config_path
```

### Verificación de Variables

```bash
# Verificar configuración ROS2
env | grep ROS
# ROS_VERSION=2
# ROS_PYTHON_VERSION=3
# ROS_DOMAIN_ID=10

# Verificar que workspace está configurado
ros2 pkg list | grep tadeo_ecar
# Debe mostrar todos los paquetes del eCar
```

## Estructura de Archivos

### Convenciones de Nombres

**Paquetes**: snake_case con prefijo
```
tadeo_ecar_control
tadeo_ecar_perception
tadeo_ecar_navigation
```

**Nodos**: descriptive_name_node
```cpp
// C++
class WheelControllerNode : public rclcpp::Node
{
    WheelControllerNode() : Node("wheel_controller_node")
};
```

**Tópicos**: snake_case jerárquico
```
/ecar/sensors/lidar/scan
/ecar/control/cmd_vel
/ecar/navigation/goal_pose
```

### Organización de Código C++

```cpp
// include/tadeo_ecar_control/wheel_controller.hpp
#ifndef TADEO_ECAR_CONTROL__WHEEL_CONTROLLER_HPP_
#define TADEO_ECAR_CONTROL__WHEEL_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace tadeo_ecar_control
{
class WheelController : public rclcpp::Node
{
public:
    WheelController();
    
private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
};
}  // namespace tadeo_ecar_control

#endif  // TADEO_ECAR_CONTROL__WHEEL_CONTROLLER_HPP_
```

### Organización de Launch Files

```python
# launch/control_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Create nodes
    wheel_controller_node = Node(
        package='tadeo_ecar_control',
        executable='wheel_controller_node',
        name='wheel_controller_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wheel_base': 1.2,
            'track_width': 0.8
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        wheel_controller_node
    ])
```

### Archivos de Configuración YAML

```yaml
# config/control_params.yaml
wheel_controller_node:
  ros__parameters:
    # Vehicle parameters
    wheel_base: 1.2        # meters
    track_width: 0.8       # meters
    wheel_radius: 0.15     # meters
    
    # Control parameters
    max_linear_velocity: 2.0   # m/s
    max_angular_velocity: 1.0  # rad/s
    
    # PID parameters
    velocity_pid:
      kp: 1.0
      ki: 0.1
      kd: 0.05
```

## Herramientas de Desarrollo

### CLI Tools Básicas

```bash
# ros2 node - Gestión de nodos
ros2 node list                    # Listar nodos activos
ros2 node info /wheel_controller_node  # Info detallada del nodo

# ros2 topic - Gestión de tópicos
ros2 topic list                   # Listar tópicos
ros2 topic echo /cmd_vel          # Escuchar tópico
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'

# ros2 service - Gestión de servicios
ros2 service list                 # Listar servicios
ros2 service call /emergency_stop std_srvs/srv/Trigger

# ros2 param - Gestión de parámetros
ros2 param list                   # Listar parámetros
ros2 param get /wheel_controller_node wheel_base
ros2 param set /wheel_controller_node max_velocity 1.5
```

### Herramientas de Introspección

```bash
# rqt_graph - Visualizar conexiones
rqt_graph

# rqt_console - Ver logs
rqt_console

# rqt_plot - Graficar datos en tiempo real
rqt_plot /ecar/sensors/battery_voltage

# rqt_reconfigure - Configurar parámetros dinámicos
rqt_reconfigure
```

### Debugging con GDB

```bash
# Compilar con símbolos de debug
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Ejecutar con gdb
ros2 run --prefix 'gdb -ex run --args' tadeo_ecar_control wheel_controller_node
```

### Profiling

```bash
# Compilar con profiling
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Usar perf (Linux)
perf record ros2 run tadeo_ecar_control wheel_controller_node
perf report

# Usar valgrind para memory leaks
valgrind --tool=memcheck ros2 run tadeo_ecar_control wheel_controller_node
```

### Automatización con Scripts

```bash
#!/bin/bash
# scripts/build_ecar.sh

set -e  # Exit on error

echo "Building eCar workspace..."

# Source ROS2
source /opt/ros/humble/setup.bash

# Clean previous build
rm -rf build/ install/ log/

# Build with optimizations
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)

# Source new build
source install/setup.bash

echo "Build completed successfully!"

# Run basic tests
echo "Running basic tests..."
ros2 pkg list | grep tadeo_ecar | wc -l  # Should be 12
```

## Ejercicios Prácticos

### Ejercicio 1: Crear un Paquete Simple

```bash
# 1. Navegar al workspace
cd ~/ros2/ecar-ws/src

# 2. Crear nuevo paquete
ros2 pkg create --build-type ament_cmake my_ecar_test

# 3. Modificar package.xml para agregar dependencias
# 4. Crear un nodo simple que publique velocidad
# 5. Compilar y probar
```

### Ejercicio 2: Explorar el Sistema eCar

```bash
# 1. Compilar workspace completo
colcon build

# 2. Listar todos los paquetes
ros2 pkg list | grep tadeo_ecar

# 3. Ejecutar nodo de control
ros2 run tadeo_ecar_control wheel_controller_node

# 4. En otra terminal, ver tópicos
ros2 topic list

# 5. Publicar comando y observar respuesta
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

### Ejercicio 3: Configurar Variables de Entorno

```bash
# 1. Crear script de configuración personalizado
cat > ~/setup_ecar.sh << 'EOF'
#!/bin/bash
export ROS_DOMAIN_ID=10
export ECAR_CONFIG_PATH="$HOME/ros2/ecar-ws/src/tadeo_ecar_config"
source /opt/ros/humble/setup.bash
source ~/ros2/ecar-ws/install/setup.bash
echo "eCar environment configured!"
EOF

# 2. Hacer ejecutable
chmod +x ~/setup_ecar.sh

# 3. Usar en nuevas terminales
source ~/setup_ecar.sh
```

## Conclusiones del Capítulo

En este capítulo hemos aprendido:

1. **Arquitectura distribuida**: Nodos independientes sin coordinador central
2. **Workspace structure**: Organización de paquetes y archivos
3. **Colcon**: Herramienta de compilación moderna
4. **Variables de entorno**: Configuración del sistema ROS2
5. **Herramientas CLI**: Introspección y debugging

### Checklist de Conocimientos

Antes de continuar, deberías poder:

- [ ] Explicar la diferencia entre workspace y paquete
- [ ] Compilar el proyecto eCar sin errores
- [ ] Configurar variables de entorno correctamente
- [ ] Usar herramientas CLI básicas
- [ ] Navegar por la estructura de archivos

### Próximo Capítulo

En el Capítulo 3 estudiaremos en profundidad:
- Concepto de nodo en ROS2
- Lifecycle de nodos
- Creación de nodos en C++ y Python
- Ejemplos prácticos con el eCar

## Referencias

- [ROS2 Workspace Tutorial](https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-A-Workspace.html)
- [Colcon Documentation](https://colcon.readthedocs.io/)
- [ROS2 Environment Variables](https://docs.ros.org/en/humble/Concepts/About-Environment-Variables.html)