# 🤖 Robot Autónomo eCar 4WD4WS

## 🚀 **La Revolución Robótica Empieza Aquí**

**ʙɪᴇɴᴠᴇɴɪᴅᴏꜱ ᴀʟ ꜱᴇᴍɪʟʟᴇʀᴏ ᴅᴇ ʀᴏʙÓᴛɪᴄᴀ** 💻💖☕

> Plataforma robótica autónoma para logística universitaria en interiores. Robot eléctrico omnidireccional 4WD 4WS con capacidades de navegación autónoma, SLAM, visión computacional y planificación de trayectorias usando ROS2 Humble. Diseñado para transporte de materiales en ambientes universitarios.

---

## 📋 **Descripción del Proyecto**

El **Robot Autónomo eCar 4WD4WS** es una plataforma robótica de vanguardia desarrollada por el **Semillero de Robótica** para revolucionar la logística en ambientes universitarios. Este sistema integra tecnologías avanzadas de robótica móvil, inteligencia artificial y navegación autónoma para crear un robot completamente autónomo capaz de transportar materiales de forma eficiente y segura.

### 🎯 **Características Principales**

- **🚗 Sistema 4WD4WS**: Tracción en las 4 ruedas con dirección en las 4 ruedas para máxima maniobrabilidad
- **🧠 Navegación Autónoma**: Sistema completo de navegación con planificación de rutas y evitación de obstáculos
- **🗺️ SLAM Avanzado**: Simultaneous Localization and Mapping para crear mapas del entorno en tiempo real
- **👁️ Visión Computacional**: Procesamiento de imágenes y detección de objetos
- **🛡️ Sistema de Seguridad Multicapa**: Múltiples niveles de protección para operación segura
- **🤖 Comportamientos Inteligentes**: Árboles de comportamiento para toma de decisiones complejas
- **📊 Monitoreo en Tiempo Real**: Sistema completo de telemetría y diagnósticos

---

## 🏗️ **Arquitectura del Sistema**

### 📦 **Paquetes del Sistema (12 Total)**

El sistema está organizado en 12 paquetes especializados que trabajan en conjunto:

#### 🔧 **1. tadeo_ecar_msgs**
**Mensajes Personalizados del Sistema**
- Mensajes de estado del robot y sistema
- Información de salud de componentes
- Estados de navegación y seguridad
- Telemetría de ruedas y sensores

#### 🔌 **2. tadeo_ecar_interfaces**
**Servicios y Acciones del Sistema**
- Servicios de calibración y configuración
- Acciones de navegación y acoplamiento
- Interfaces de control y monitoreo
- Comunicación entre subsistemas

#### ⚙️ **3. tadeo_ecar_config**
**Configuraciones Centralizadas**
- Parámetros del robot y controladores
- Configuración de navegación (Nav2)
- Parámetros de sensores y hardware
- Archivos de lanzamiento básicos

#### 🎮 **4. tadeo_ecar_control**
**Sistema de Control 4WD4WS**
- Control cinemático 4WD4WS avanzado
- Dinámica vehicular y estabilidad
- Controladores PID para ruedas
- Gestión de actuadores y motores

#### 📡 **5. tadeo_ecar_perception**
**Procesamiento de Sensores**
- Procesamiento de LiDAR 360°
- Visión computacional con cámaras
- Fusión de datos de IMU
- Calibración automática de sensores

#### 🛡️ **6. tadeo_ecar_safety**
**Sistema de Seguridad Multicapa**
- Parada de emergencia automática
- Evitación de colisiones en tiempo real
- Monitoreo de estado del sistema
- Watchdog para componentes críticos

#### 🎯 **7. tadeo_ecar_localization**
**Localización y Fusión Sensorial**
- Filtro de Kalman Extendido (EKF)
- Filtro de Kalman sin scent (UKF)
- Fusión de odometría y sensores
- Corrección de deriva y localización precisa

#### 🗺️ **8. tadeo_ecar_slam**
**SLAM y Mapeo**
- SLAM con SLAM Toolbox
- SLAM con Cartographer
- Gestión inteligente de mapas
- Detección de bucles cerrados

#### 🛣️ **9. tadeo_ecar_planning**
**Planificación de Trayectorias**
- Planificación global con OMPL
- Planificador local adaptativo
- Optimización de trayectorias
- Algoritmos RRT*, PRM, EST

#### 🧭 **10. tadeo_ecar_navigation**
**Navegación Autónoma**
- Integración completa con Nav2
- Ejecución de misiones complejas
- Gestión de waypoints
- Monitoreo de navegación

#### 🧠 **11. tadeo_ecar_behavior**
**Comportamientos Inteligentes**
- Árboles de comportamiento (BehaviorTree.CPP)
- Máquina de estados jerárquica
- Comportamientos autónomos adaptativos
- Monitoreo de rendimiento

#### 🚀 **12. tadeo_ecar_bringup**
**Sistema de Arranque Integral**
- Lanzamiento de sistema completo
- Configuraciones para múltiples escenarios
- Scripts de utilidad y monitoreo
- Integración de todos los subsistemas

---

## 🛠️ **Tecnologías Utilizadas**

### 🤖 **Frameworks y Middleware**
- **ROS2 Humble**: Sistema operativo robótico de última generación
- **Nav2**: Stack de navegación autónoma
- **BehaviorTree.CPP**: Motor de árboles de comportamiento
- **SLAM Toolbox**: SLAM de código abierto
- **Cartographer**: SLAM avanzado de Google

### 🧮 **Algoritmos y Librerías**
- **OMPL**: Librería de planificación de movimiento
- **OpenCV**: Visión computacional
- **PCL**: Procesamiento de nubes de puntos
- **Eigen**: Álgebra lineal de alto rendimiento
- **yaml-cpp**: Manejo de configuraciones

### 📊 **Visualización y Monitoreo**
- **RViz2**: Visualización 3D avanzada
- **Foxglove Studio**: Monitoreo web en tiempo real
- **Groot**: Debugging de árboles de comportamiento
- **PlotJuggler**: Análisis de datos en tiempo real

### 🔧 **Herramientas de Desarrollo**
- **Colcon**: Sistema de construcción
- **Rosbag2**: Grabación y reproducción de datos
- **Launch**: Sistema de lanzamiento flexible
- **Lifecycle**: Gestión del ciclo de vida de nodos

---

## 🎯 **Casos de Uso**

### 🏫 **Logística Universitaria**
- Transporte de materiales entre laboratorios
- Distribución de equipos y suministros
- Entrega de documentos y paquetes
- Soporte en bibliotecas y centros de cómputo

### 🔬 **Investigación y Desarrollo**
- Plataforma para investigación en robótica móvil
- Testing de algoritmos de navegación
- Desarrollo de aplicaciones de IA
- Educación en robótica avanzada

### 🏭 **Aplicaciones Industriales**
- Inspección de instalaciones
- Mapeo de entornos industriales
- Monitoreo de seguridad
- Automatización de procesos

---

## 🚀 **Inicio Rápido**

### 📋 **Requisitos Previos**

#### Sistema Operativo
```bash
Ubuntu 22.04 LTS (recomendado)
ROS2 Humble Hawksbill
```

#### Dependencias
```bash
# Instalar ROS2 Humble
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop-full
```

#### Dependencias del Proyecto
```bash
# Herramientas de construcción
sudo apt install python3-colcon-common-extensions

# Navegación
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox

# Comportamientos
sudo apt install ros-humble-behaviortree-cpp-v3

# Visualización
sudo apt install ros-humble-rviz2 ros-humble-foxglove-bridge
```

### 🏗️ **Instalación**

```bash
# 1. Crear workspace
mkdir -p ~/ros2/ecar-ws/src
cd ~/ros2/ecar-ws/src

# 2. Clonar repositorio (requiere acceso)
git clone https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws.git .

# 3. Instalar dependencias
cd ~/ros2/ecar-ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Compilar proyecto
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 5. Source del workspace
source install/setup.bash
```

### ▶️ **Ejecución del Sistema**

#### 🎮 **Modo Simulación (Recomendado para Empezar)**
```bash
# Sistema completo en simulación
./src/tadeo_ecar_bringup/scripts/start_system.sh sim
```

#### 🤖 **Modo Hardware Real**
```bash
# Sistema en hardware real (requiere robot físico)
./src/tadeo_ecar_bringup/scripts/start_system.sh hardware
```

#### 🗺️ **Solo Navegación**
```bash
# Solo sistema de navegación (requiere mapa existente)
./src/tadeo_ecar_bringup/scripts/start_system.sh nav
```

#### 📊 **Monitor del Sistema**
```bash
# Monitor en tiempo real
./src/tadeo_ecar_bringup/scripts/start_system.sh monitor
```

---

## 🎮 **Control del Robot**

### 🕹️ **Control Manual**
```bash
# Teleoperación con teclado
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 🎯 **Navegación por Objetivos**
```bash
# Enviar objetivo de navegación
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "
{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

### 🤖 **Control de Comportamientos**
```bash
# Cambiar modo de comportamiento
ros2 topic pub /behavior_command std_msgs/String "data: 'start_tree:patrol_behavior'"

# Cambiar estado del robot
ros2 topic pub /state_command std_msgs/String "data: 'TRANSITION:NAVIGATING'"
```

---

## 📊 **Monitoreo y Diagnósticos**

### 📈 **Tópicos Principales**
```bash
# Estado del robot
ros2 topic echo /robot_state

# Salud del sistema
ros2 topic echo /system_health

# Métricas de navegación
ros2 topic echo /navigation_metrics

# Estado de batería
ros2 topic echo /battery_state
```

### 🔍 **Diagnósticos Avanzados**
```bash
# Diagnósticos agregados
ros2 topic echo /diagnostics_agg

# Salud de componentes específicos
ros2 topic echo /behavior/manager_health
ros2 topic echo /safety/monitor_health
```

### 📊 **Visualización**
```bash
# RViz para visualización 3D
rviz2 -d src/tadeo_ecar_bringup/rviz/tadeo_ecar_full_system.rviz

# Foxglove para monitoreo web (puerto 8765)
# Abrir navegador en http://localhost:8765
```

---

## 🛠️ **Configuración Avanzada**

### ⚙️ **Parámetros del Sistema**
Los parámetros principales se encuentran en:
- `tadeo_ecar_config/config/robot_params.yaml`
- `tadeo_ecar_bringup/config/system_params.yaml`
- `tadeo_ecar_bringup/params/hardware_params.yaml`

### 🎛️ **Personalización de Comportamientos**
```bash
# Editar árboles de comportamiento
nano src/tadeo_ecar_behavior/behavior_trees/main_behavior.xml

# Modificar parámetros de navegación
nano src/tadeo_ecar_config/config/nav2_params.yaml
```

### 🔧 **Calibración de Hardware**
```bash
# Calibración automática
ros2 launch tadeo_ecar_bringup tadeo_ecar_hardware.launch.py auto_calibrate:=true

# Calibración manual de ruedas
ros2 run tadeo_ecar_control wheel_calibration_node
```

---

## 🧪 **Testing y Desarrollo**

### 🔬 **Ejecutar Tests**
```bash
# Tests unitarios
colcon test --packages-select tadeo_ecar_control tadeo_ecar_perception

# Ver resultados
colcon test-result --verbose
```

### 🐛 **Debugging**
```bash
# Modo debug con logs detallados
ros2 launch tadeo_ecar_bringup tadeo_ecar_full_system.launch.py log_level:=debug

# Groot para debugging de comportamientos
ros2 launch tadeo_ecar_behavior behavior_system.launch.py enable_groot:=true
```

### 📝 **Grabación de Datos**
```bash
# Grabar datos importantes
ros2 bag record /scan /camera/image_raw /odom /tf /cmd_vel /robot_state

# Reproducir datos grabados
ros2 bag play <nombre_del_bag>
```

---

## 📁 **Estructura del Proyecto**

```
tadeo-eCar-ws/src/
├── 📦 tadeo_ecar_msgs/              # Mensajes personalizados
│   ├── msg/                         # Definiciones de mensajes
│   └── README.md                    # Documentación de mensajes
├── 🔌 tadeo_ecar_interfaces/        # Servicios y acciones
│   ├── srv/                         # Servicios del sistema
│   ├── action/                      # Acciones para tareas largas
│   └── README.md                    # Documentación de interfaces
├── ⚙️ tadeo_ecar_config/            # Configuraciones centralizadas
│   ├── config/                      # Archivos de configuración
│   ├── launch/                      # Launches básicos
│   └── README.md                    # Guía de configuración
├── 🎮 tadeo_ecar_control/           # Sistema de control 4WD4WS
│   ├── src/                         # Nodos de control C++
│   ├── include/                     # Headers del sistema
│   ├── config/                      # Parámetros de control
│   └── README.md                    # Documentación de control
├── 📡 tadeo_ecar_perception/        # Procesamiento de sensores
│   ├── src/                         # Nodos de percepción C++
│   ├── config/                      # Configuración de sensores
│   └── README.md                    # Guía de percepción
├── 🛡️ tadeo_ecar_safety/            # Sistema de seguridad
│   ├── src/                         # Nodos de seguridad C++
│   ├── config/                      # Parámetros de seguridad
│   └── README.md                    # Manual de seguridad
├── 🎯 tadeo_ecar_localization/      # Localización y fusión
│   ├── src/                         # Nodos de localización C++
│   ├── include/                     # Headers de algoritmos
│   └── README.md                    # Guía de localización
├── 🗺️ tadeo_ecar_slam/              # SLAM y mapeo
│   ├── src/                         # Nodos de SLAM C++
│   ├── config/                      # Configuración de SLAM
│   └── README.md                    # Manual de SLAM
├── 🛣️ tadeo_ecar_planning/          # Planificación de trayectorias
│   ├── src/                         # Planificadores C++
│   ├── include/                     # Headers de planning
│   └── README.md                    # Guía de planificación
├── 🧭 tadeo_ecar_navigation/        # Navegación autónoma
│   ├── src/                         # Nodos de navegación C++
│   ├── config/                      # Parámetros de navegación
│   ├── rviz/                        # Configuraciones de RViz
│   └── README.md                    # Manual de navegación
├── 🧠 tadeo_ecar_behavior/          # Comportamientos inteligentes
│   ├── src/                         # Nodos de comportamiento C++
│   ├── behavior_trees/              # Árboles XML
│   ├── config/                      # Parámetros de comportamiento
│   └── README.md                    # Guía de comportamientos
├── 🚀 tadeo_ecar_bringup/           # Sistema de arranque
│   ├── launch/                      # Launches del sistema completo
│   ├── config/                      # Configuraciones globales
│   ├── scripts/                     # Scripts de utilidad
│   ├── rviz/                        # Configuraciones de visualización
│   └── README.md                    # Manual de sistema completo
└── 📄 README.md                     # Este archivo
```

---

## 🤝 **Contribución**

### 👥 **Cómo Contribuir**

1. **Fork del repositorio**
2. **Crear rama para nueva funcionalidad**
   ```bash
   git checkout -b feature/nueva-funcionalidad
   ```
3. **Realizar cambios con commits descriptivos**
   ```bash
   git commit -m "feat: añadir nueva funcionalidad de navegación"
   ```
4. **Push y crear Pull Request**
   ```bash
   git push origin feature/nueva-funcionalidad
   ```

### 📝 **Estándares de Código**

- **C++17** para nodos ROS2
- **Python 3.8+** para scripts y launches
- **Documentación** en español para README
- **Comentarios** en código en inglés
- **Naming conventions** siguiendo ROS2 standards

### 🧪 **Testing**

- Todos los nodos deben incluir tests unitarios
- Tests de integración para sistemas complejos
- Documentación de casos de uso
- Validación en simulación antes de hardware

---

## 🔧 **Solución de Problemas**

### ❓ **Problemas Comunes**

#### 🚫 **Sistema No Inicia**
```bash
# Verificar dependencias
ros2 pkg list | grep tadeo_ecar

# Recompilar workspace
cd ~/ros2/ecar-ws
colcon build --symlink-install
source install/setup.bash
```

#### 🔗 **Problemas de Comunicación**
```bash
# Verificar nodos activos
ros2 node list

# Verificar tópicos
ros2 topic list

# Verificar conectividad
ros2 node info /nombre_del_nodo
```

#### 🐌 **Rendimiento Lento**
```bash
# Monitor del sistema
./src/tadeo_ecar_bringup/scripts/system_monitor.py

# Verificar uso de recursos
htop

# Ajustar frecuencias en config/system_params.yaml
```

#### 🤖 **Robot No Responde**
```bash
# Verificar estado de emergencia
ros2 topic echo /emergency_stop

# Resetear sistema de comportamientos
ros2 topic pub /behavior_command std_msgs/String "data: 'reset'"

# Verificar hardware (modo hardware)
ros2 topic echo /system_health
```

### 📞 **Soporte Técnico**

Para soporte técnico especializado:
- **Email**: ing.marioalvarezvallejo@gmail.com
- **Documentación**: Consultar README de cada paquete
- **Issues**: Usar sistema de issues del repositorio

---

## 📚 **Documentación Adicional**

### 📖 **Manuales Detallados**
Cada paquete incluye documentación específica en su directorio:

- **Control**: `tadeo_ecar_control/README.md` - Cinemática 4WD4WS y control
- **Percepción**: `tadeo_ecar_perception/README.md` - Sensores y procesamiento
- **Navegación**: `tadeo_ecar_navigation/README.md` - Nav2 y planificación
- **Comportamientos**: `tadeo_ecar_behavior/README.md` - BehaviorTree y IA
- **Sistema**: `tadeo_ecar_bringup/README.md` - Lanzamiento y configuración

### 🎓 **Recursos de Aprendizaje**

#### 📝 **Tutoriales Recomendados**
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Nav2 Documentation](https://navigation.ros.org/)
- [BehaviorTree.CPP Guide](https://www.behaviortree.dev/)

#### 📚 **Libros Recomendados**
- "Programming Robots with ROS" - Morgan Quigley
- "Probabilistic Robotics" - Sebastian Thrun
- "Planning Algorithms" - Steven LaValle

#### 🎥 **Videos y Cursos**
- [ROS2 Course](https://www.youtube.com/ros)
- [Robot Operating System](https://www.edx.org/course/robot-operating-system-ros)

---

## 🏆 **Reconocimientos**

### 👨‍💻 **Equipo de Desarrollo**

**Desarrollado por el Semillero de Robótica** 🤖

- **Líder del Proyecto**: Mario Álvarez Vallejo
- **Email**: ing.marioalvarezvallejo@gmail.com
- **Organización**: Semillero de Robótica

### 🙏 **Agradecimientos**

- **ROS2 Community** por el increíble framework
- **Open Source Robotics Foundation** por las herramientas
- **Nav2 Team** por el stack de navegación
- **BehaviorTree.CPP Team** por el motor de comportamientos

### 💖 **Apoyo al Proyecto**

Si te ha gustado este proyecto, puedes apoyarnos:

[![Buy Me A Coffee](https://img.shields.io/badge/-buy_me_a%C2%A0coffee-gray?logo=buy-me-a-coffee)](https://www.buymeacoffee.com/semillerorobotica)

**¡ʜᴇʏ ᴛÚ! ꜱɪ ᴛᴇ ʜᴀ ɢᴜꜱᴛᴀᴅᴏ ʟᴏ ǫᴜᴇ ʜᴀᴄᴇᴍᴏꜱ ɪɴᴠɪᴛᴀɴᴏꜱ ᴜɴ ᴄᴀꜰÉ!** ☕

---

## 📄 **Licencia**

```
Repositorio Privado - Semillero de Robótica

Copyright (c) 2024 Semillero de Robótica
Todos los derechos reservados.

Este código es propiedad del Semillero de Robótica y está protegido por 
leyes de derechos de autor. No se permite la reproducción, distribución,
o modificación sin autorización expresa del propietario.

Para consultas sobre licenciamiento, contactar:
ing.marioalvarezvallejo@gmail.com
```

---

## 🔄 **Versioning**

**Versión Actual**: 1.0.0

### 📈 **Historial de Versiones**
- **v1.0.0** (2024) - Lanzamiento inicial con sistema completo
  - 12 paquetes integrados
  - Navegación autónoma completa
  - Sistema de comportamientos avanzado
  - Soporte para simulación y hardware

### 🚀 **Roadmap Futuro**
- **v1.1.0** - Mejoras en planificación y optimización
- **v1.2.0** - Integración con IA y machine learning
- **v2.0.0** - Soporte multi-robot y coordinación

---

## 📬 **Contacto**

### 🏢 **Organización**
**Semillero de Robótica**

### 👨‍💻 **Desarrollador Principal**
- **Nombre**: Mario Álvarez Vallejo
- **Email**: ing.marioalvarezvallejo@gmail.com
- **Especialidad**: Robótica Móvil y Sistemas Autónomos

### 🌐 **Enlaces**
- **GitHub Organización**: [TadeoRoboticsGroup](http://github.com/TadeoRoboticsGroup)
- **Repositorio**: [tadeo-eCar-ws](https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws)
- **Sitio Web**: [Semillero de Robótica](https://tadeoroboticsgroup.github.io/TadeoRoboticsGroup/)
- **Issues**: [Reportar Problemas](https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws/issues)

---

**🤖 ᴀǫᴜí ᴘᴏᴅʀáɴ ᴀᴘʀᴇɴᴅᴇʀ, ᴇxᴘᴇʀɪᴍᴇɴᴛᴀʀ ʏ ᴅᴀʀ ʀɪᴇɴᴅᴀ ꜱᴜᴇʟᴛᴀ ᴀ ꜱᴜ ᴄʀᴇᴀᴛɪᴠɪᴅᴀᴅ ᴇɴ ᴇʟ ᴇᴍᴏᴄɪᴏɴᴀɴᴛᴇ ᴍᴜɴᴅᴏ ᴅᴇ ʟᴀ ʀᴏʙóᴛɪᴄᴀ** 💻💖☕

---

*¡La revolución robótica está en tus manos! 🚀🤖*