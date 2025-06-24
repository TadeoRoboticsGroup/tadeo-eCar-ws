# Tadeo eCar - Autonomous 4WD4WS Robot

Robot autónomo 4WD4WS (4-wheel drive, 4-wheel steering) con ROS2 Humble para desarrollo en Docker/WSL.

## 🚗 Descripción del Robot

- **Tipo:** 4WD4WS (tracción y dirección en 4 ruedas)
- **Sensores:** LiDAR, Cámara RGB
- **Simulación:** Gazebo Ignition Fortress
- **Visualización:** RVIZ2 con mesh STL personalizados

## 📁 Estructura del Paquete

```
tadeo_ecar_description/
├── launch/                    # Archivos de lanzamiento
│   ├── rviz_display.launch.py    # Solo visualización RVIZ
│   └── simulation.launch.py      # Simulación completa
├── urdf/                      # Descripción del robot
│   ├── tadeo_ecar.urdf.xacro     # Archivo principal
│   ├── tadeo_ecar_base.xacro     # Chasis y sensores
│   ├── tadeo_ecar_wheels.xacro   # Ruedas y suspensión
│   ├── materials.xacro           # Materiales visuales
│   └── gazebo_ignition.xacro     # Plugins Gazebo
├── meshes/                    # Modelos 3D STL
│   ├── chassis/               # Chasis del vehículo
│   ├── wheels/                # Ruedas
│   └── suspension/            # Suspensión
├── rviz/                      # Configuración RVIZ
├── worlds/                    # Mundos de simulación
└── config/                    # Parámetros
```

## 🚀 Launches Disponibles

### 1. RVIZ Display (Solo Visualización)
```bash
ros2 launch tadeo_ecar_description rviz_display.launch.py
```
- ✅ Visualización 3D con mesh STL
- ✅ Joint State Publisher GUI (mover articulaciones)
- ✅ Ideal para desarrollo y debug

### 2. Simulation (Simulación Completa)
```bash
ros2 launch tadeo_ecar_description simulation.launch.py
```
- ✅ Gazebo server (física)
- ✅ Robot spawneado en simulación
- ✅ Bridges ROS-Gazebo activos
- ✅ RVIZ2 con todos los sensores
- ✅ Listo para navegación autónoma

### Ver Gazebo GUI (Opcional)
```bash
ign gazebo -g
```

## 🔗 Tópicos ROS2

### Sensores
- `/scan` - Datos LiDAR (sensor_msgs/LaserScan)
- `/camera` - Imagen RGB (sensor_msgs/Image)
- `/camera_info` - Info cámara (sensor_msgs/CameraInfo)

### Control
- `/cmd_vel` - Comandos velocidad (geometry_msgs/Twist)
- `/odom` - Odometría (nav_msgs/Odometry)

### Estado del Robot
- `/joint_states` - Estados articulaciones (sensor_msgs/JointState)
- `/tf` - Transformaciones (tf2_msgs/TFMessage)
- `/robot_description` - Descripción URDF (std_msgs/String)

## 🤖 Configuración del Robot

### Dimensiones
- **Wheelbase:** 1.058m
- **Track Width:** 0.55m
- **Chasis:** 1.2m x 0.8m x 0.4m

### Sensores
- **LiDAR:** Montado en frente (0.6m, 0, altura_chasis)
- **Cámara:** Montada en frente (0.476m, 0, altura_superior)

### Articulaciones
- **4 joints de dirección:** front/rear_left/right_suspension_joint
- **4 joints de tracción:** front/rear_left/right_wheel_joint

## 🔧 Para Navegación Autónoma

### Paquetes Requeridos
```bash
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-robot-localization
```

### Uso con Nav2
```bash
# Terminal 1: Simulación
ros2 launch tadeo_ecar_description simulation.launch.py

# Terminal 2: Navegación (ejemplo)
ros2 launch nav2_bringup navigation_launch.py
```

### Bridges Configurados
- `/clock` - Sincronización temporal
- `/cmd_vel` - Control movimiento
- `/odom` - Odometría para localización
- `/tf` - Transformaciones
- `/scan` - LiDAR para navegación
- `/camera` - Visión para percepción

## 🐛 Troubleshooting

### Mesh no cargan
```bash
# Limpiar y reconstruir
rm -rf build log install
colcon build --packages-select tadeo_ecar_description
```

### Gazebo no se ve
```bash
# Ejecutar en terminales separadas
ros2 launch tadeo_ecar_description simulation.launch.py
ign gazebo -g
```

### Sensores no aparecen en RVIZ
- Usar "Add" → "LaserScan" para LiDAR
- Usar "Add" → "Image" para cámara
- Verificar tópicos: `ros2 topic list`

## 📋 Desarrollo

### Modificar geometría
- Editar archivos en `urdf/`
- Limpiar build: `rm -rf build log install`
- Reconstruir: `colcon build --packages-select tadeo_ecar_description`

### Agregar sensores
- Modificar `urdf/tadeo_ecar_base.xacro`
- Agregar plugins en `urdf/gazebo_ignition.xacro`
- Configurar bridges en `launch/simulation.launch.py`

---
**Autor:** Desarrollo para robot autónomo Tadeo eCar  
**ROS2:** Humble  
**Simulador:** Gazebo Ignition Fortress
