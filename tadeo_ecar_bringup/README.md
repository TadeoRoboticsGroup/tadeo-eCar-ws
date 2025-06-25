# Sistema de Arranque Tadeo eCar

## Descripción General

El paquete `tadeo_ecar_bringup` es el punto de entrada principal para el sistema robótico autónomo Tadeo eCar 4WD4WS. Este paquete integra todos los subsistemas del robot y proporciona múltiples configuraciones de lanzamiento para diferentes escenarios de operación.

## Características Principales

### 🚀 Lanzamiento Integral
- **Sistema completo** con todos los subsistemas integrados
- **Múltiples modos** de operación (simulación, hardware, desarrollo)
- **Configuración flexible** mediante parámetros de lanzamiento
- **Scripts de utilidad** para operación simplificada

### 🎯 Modos de Operación
- **Simulación completa** con Gazebo/Ignition
- **Hardware real** con drivers y calibración
- **Solo navegación** para operación con mapas existentes
- **SLAM** para generación de mapas
- **Configuración mínima** para desarrollo y testing

### 📊 Monitoreo y Diagnóstico
- **Monitor del sistema** en tiempo real
- **Diagnósticos avanzados** de componentes
- **Logging automático** para análisis posterior
- **Visualización integrada** con RViz y Foxglove

## Arquitectura del Sistema

```
tadeo_ecar_bringup/
├── launch/                          # Archivos de lanzamiento
│   ├── tadeo_ecar_full_system.launch.py    # Sistema completo
│   ├── tadeo_ecar_simulation.launch.py     # Modo simulación
│   └── tadeo_ecar_hardware.launch.py       # Modo hardware
├── config/                          # Configuraciones globales
│   └── system_params.yaml          # Parámetros del sistema
├── params/                          # Parámetros específicos
│   └── hardware_params.yaml        # Configuración de hardware
├── rviz/                           # Configuraciones de RViz
│   └── tadeo_ecar_full_system.rviz # Vista principal
├── scripts/                        # Scripts de utilidad
│   ├── system_monitor.py           # Monitor del sistema
│   └── start_system.sh             # Script de inicio rápido
└── README.md
```

## Subsistemas Integrados

### 1. **tadeo_ecar_description**
- Modelo URDF del robot
- Configuración de simulación
- Publicador de estado del robot

### 2. **tadeo_ecar_perception**
- Procesamiento de sensores (LiDAR, cámaras, IMU)
- Fusión sensorial
- Calibración automática

### 3. **tadeo_ecar_control**
- Control 4WD4WS
- Cinemática del vehículo
- Controladores de ruedas

### 4. **tadeo_ecar_safety**
- Sistema de seguridad multicapa
- Parada de emergencia
- Monitoreo de colisiones
- Watchdog del sistema

### 5. **tadeo_ecar_localization**
- Localización con EKF/UKF
- Fusión de odometría
- Corrección de deriva

### 6. **tadeo_ecar_slam**
- SLAM con SLAM Toolbox
- SLAM con Cartographer
- Gestión de mapas

### 7. **tadeo_ecar_planning**
- Planificación global con OMPL
- Planificación local
- Optimización de trayectorias

### 8. **tadeo_ecar_navigation**
- Integración con Nav2
- Ejecución de misiones
- Gestión de waypoints

### 9. **tadeo_ecar_behavior**
- Árboles de comportamiento (BehaviorTree.CPP)
- Máquina de estados
- Comportamientos autónomos

## Uso del Sistema

### 1. Lanzamiento Rápido

#### Script de Inicio Simplificado
```bash
# Simulación completa
./src/tadeo_ecar_bringup/scripts/start_system.sh sim

# Hardware real
./src/tadeo_ecar_bringup/scripts/start_system.sh hardware

# Solo navegación
./src/tadeo_ecar_bringup/scripts/start_system.sh nav

# SLAM para mapeo
./src/tadeo_ecar_bringup/scripts/start_system.sh slam

# Configuración mínima
./src/tadeo_ecar_bringup/scripts/start_system.sh minimal

# Monitor del sistema
./src/tadeo_ecar_bringup/scripts/start_system.sh monitor
```

### 2. Lanzamiento Manual Detallado

#### Sistema Completo
```bash
ros2 launch tadeo_ecar_bringup tadeo_ecar_full_system.launch.py \
    use_simulation:=false \
    use_rviz:=true \
    navigation_mode:=full \
    behavior_mode:=full \
    safety_mode:=full \
    slam_mode:=slam_toolbox
```

#### Simulación
```bash
ros2 launch tadeo_ecar_bringup tadeo_ecar_simulation.launch.py \
    world_file:=empty_world_ignition.sdf \
    enable_navigation:=true \
    enable_slam:=true \
    use_rviz:=true
```

#### Hardware Real
```bash
ros2 launch tadeo_ecar_bringup tadeo_ecar_hardware.launch.py \
    auto_calibrate:=true \
    enable_safety:=true \
    use_foxglove:=true \
    record_data:=false
```

### 3. Parámetros de Lanzamiento

#### Parámetros Globales
- `namespace`: Namespace del robot (default: '')
- `use_sim_time`: Usar tiempo de simulación (default: false)
- `robot_name`: Nombre del robot (default: 'tadeo_ecar')
- `log_level`: Nivel de logging (default: 'info')

#### Parámetros de Modo
- `use_simulation`: Activar modo simulación (default: false)
- `use_rviz`: Lanzar RViz (default: true)
- `use_foxglove`: Lanzar Foxglove Bridge (default: false)
- `autostart`: Autostart de lifecycle managers (default: true)

#### Parámetros de Subsistemas
- `navigation_mode`: Modo navegación [full, localization_only, planning_only, none]
- `slam_mode`: Modo SLAM [none, slam_toolbox, cartographer]
- `behavior_mode`: Modo comportamientos [full, state_machine_only, none]
- `safety_mode`: Modo seguridad [full, emergency_only, none]

## Monitoreo del Sistema

### Monitor en Tiempo Real
```bash
# Lanzar monitor
python3 src/tadeo_ecar_bringup/scripts/system_monitor.py
```

El monitor muestra:
- **Estado del robot** (batería, estado actual)
- **Recursos del sistema** (CPU, RAM, disco)
- **Salud de componentes** (todos los subsistemas)
- **Procesos ROS2** activos

### Tópicos de Monitoreo
```bash
# Estado del robot
rostopic echo /robot_state

# Salud del sistema
rostopic echo /system_health

# Estado de batería
rostopic echo /battery_state

# Métricas de comportamientos
rostopic echo /behavior_metrics

# Alertas del sistema
rostopic echo /system_alerts
```

### Diagnósticos
```bash
# Ver diagnósticos agregados
rostopic echo /diagnostics_agg

# Ver salud de componentes específicos
rostopic echo /behavior/manager_health
rostopic echo /safety/monitor_health
rostopic echo /navigation/health
```

## Configuración del Sistema

### Archivo Principal de Configuración
**Ubicación:** `config/system_params.yaml`

#### Secciones principales:
- **robot_description**: Dimensiones y capacidades
- **tf_configuration**: Configuración de frames
- **topic_configuration**: Mapeo de tópicos
- **qos_configuration**: Perfiles de calidad de servicio
- **safety_configuration**: Límites y zonas de seguridad
- **monitoring_configuration**: Configuración de monitoreo

### Configuración de Hardware
**Ubicación:** `params/hardware_params.yaml`

#### Componentes configurables:
- **Motores**: PID, límites, comunicación serie
- **Encoders**: Resolución, calibración
- **LiDAR**: Parámetros de escaneo, filtrado
- **Cámara**: Resolución, calibración
- **IMU**: Calibración, filtrado
- **Monitoreo**: Batería, temperatura, voltajes

## Escenarios de Uso

### 1. Desarrollo y Testing
```bash
# Configuración mínima para desarrollo
ros2 launch tadeo_ecar_bringup tadeo_ecar_full_system.launch.py \
    navigation_mode:=none \
    slam_mode:=none \
    behavior_mode:=state_machine_only \
    safety_mode:=emergency_only \
    use_rviz:=false \
    log_level:=debug
```

### 2. Simulación de Desarrollo
```bash
# Simulación con todas las funcionalidades
ros2 launch tadeo_ecar_bringup tadeo_ecar_simulation.launch.py \
    enable_navigation:=true \
    enable_slam:=true \
    slam_mode:=slam_toolbox \
    use_rviz:=true \
    simulator:=ignition
```

### 3. Generación de Mapas
```bash
# SLAM para crear mapas del entorno
ros2 launch tadeo_ecar_bringup tadeo_ecar_full_system.launch.py \
    use_simulation:=false \
    navigation_mode:=localization_only \
    slam_mode:=slam_toolbox \
    behavior_mode:=state_machine_only \
    use_rviz:=true
```

### 4. Navegación Autónoma
```bash
# Navegación con mapa conocido
ros2 launch tadeo_ecar_bringup tadeo_ecar_full_system.launch.py \
    use_simulation:=false \
    navigation_mode:=full \
    slam_mode:=none \
    behavior_mode:=full \
    safety_mode:=full \
    use_rviz:=true
```

### 5. Operación en Hardware Real
```bash
# Configuración completa para hardware
ros2 launch tadeo_ecar_bringup tadeo_ecar_hardware.launch.py \
    auto_calibrate:=true \
    enable_safety:=true \
    enable_navigation:=true \
    battery_monitoring:=true \
    use_foxglove:=true \
    record_data:=true
```

## Integración con Herramientas Externas

### RViz
- **Configuración**: `rviz/tadeo_ecar_full_system.rviz`
- **Displays**: Robot, sensores, navegación, comportamientos
- **Tools**: Pose inicial, objetivos, puntos de interés

### Foxglove Studio
```bash
# Activar Foxglove Bridge
ros2 launch tadeo_ecar_bringup tadeo_ecar_full_system.launch.py \
    use_foxglove:=true
```
- **Puerto**: 8765
- **Interfaz web**: Monitoreo remoto y control

### Groot (BehaviorTree Debugger)
```bash
# Activar Groot para debugging de comportamientos
ros2 launch tadeo_ecar_behavior behavior_system.launch.py \
    enable_groot:=true \
    groot_port:=1666
```

## Grabación y Análisis de Datos

### Grabación Automática
```bash
# Grabar datos importantes
ros2 launch tadeo_ecar_bringup tadeo_ecar_hardware.launch.py \
    record_data:=true
```

### Grabación Manual
```bash
# Grabar tópicos específicos
ros2 bag record /scan /camera/image_raw /odom /tf /cmd_vel \
    /robot_state /system_health /battery_state
```

### Análisis de Logs
```bash
# Ver logs del sistema
tail -f /tmp/tadeo_ecar_logs/*.log

# Analizar datos grabados
ros2 bag info <bag_file>
ros2 bag play <bag_file>
```

## Solución de Problemas

### Problemas Comunes

#### 1. Sistema No Inicia
```bash
# Verificar dependencias
ros2 pkg list | grep tadeo_ecar

# Verificar compilation
colcon build --packages-select tadeo_ecar_bringup

# Verificar permisos de scripts
chmod +x src/tadeo_ecar_bringup/scripts/*
```

#### 2. Nodos No Se Comunican
```bash
# Verificar tópicos
ros2 topic list

# Verificar conectividad
ros2 node list
ros2 node info <node_name>

# Verificar QoS
ros2 topic info <topic_name> --verbose
```

#### 3. Rendimiento Degradado
```bash
# Verificar recursos del sistema
./src/tadeo_ecar_bringup/scripts/system_monitor.py

# Verificar frecuencias de publicación
ros2 topic hz <topic_name>

# Ajustar parámetros de QoS en system_params.yaml
```

#### 4. Problemas de Simulación
```bash
# Verificar Gazebo/Ignition
gazebo --version
ign gazebo --version

# Relanzar simulación
ros2 launch tadeo_ecar_bringup tadeo_ecar_simulation.launch.py
```

#### 5. Errores de Hardware
```bash
# Verificar dispositivos
ls /dev/tty*
ls /dev/video*

# Verificar permisos
sudo usermod -a -G dialout $USER

# Verificar configuración en hardware_params.yaml
```

### Logs de Diagnóstico
```bash
# Logs en tiempo real
ros2 launch tadeo_ecar_bringup tadeo_ecar_full_system.launch.py \
    log_level:=debug

# Logs de componentes específicos
ros2 node info /behavior_manager_node
ros2 service call /behavior_manager_node/get_logger_level \
    rcl_interfaces/srv/GetLoggerLevels
```

## Desarrollo y Extensión

### Añadir Nuevos Subsistemas
1. Crear paquete con dependencias
2. Añadir en `package.xml` como `exec_depend`
3. Incluir launch file en `tadeo_ecar_full_system.launch.py`
4. Actualizar configuración en `system_params.yaml`

### Crear Nuevos Modos de Operación
1. Copiar launch file existente
2. Modificar parámetros de lanzamiento
3. Añadir en script `start_system.sh`
4. Documentar en README

### Personalizar Configuraciones
1. Modificar archivos YAML en `config/` y `params/`
2. Crear perfiles específicos para diferentes robots
3. Usar parámetros de lanzamiento para selección

## Mantenimiento

### Actualizaciones del Sistema
```bash
# Actualizar workspace
cd /home/thinkpad-wsl/ros2/tadeo-eCar-ws
git pull
colcon build

# Verificar integridad
ros2 launch tadeo_ecar_bringup tadeo_ecar_full_system.launch.py \
    navigation_mode:=none slam_mode:=none behavior_mode:=none
```

### Calibración Periódica
```bash
# Calibración automática de hardware
ros2 launch tadeo_ecar_bringup tadeo_ecar_hardware.launch.py \
    auto_calibrate:=true
```

### Limpieza de Logs
```bash
# Limpiar logs antiguos
find /tmp/tadeo_ecar_logs -name "*.log" -mtime +7 -delete
```

## Rendimiento y Optimización

### Configuración para Producción
```yaml
# En system_params.yaml
monitoring_configuration:
  monitoring_frequencies:
    system_health: 1.0      # Reducir frecuencia
  logging:
    log_level: "WARN"       # Solo warnings y errores
    log_to_file: false      # Deshabilitar logging a archivo
```

### Configuración para Desarrollo
```yaml
# En system_params.yaml
monitoring_configuration:
  monitoring_frequencies:
    system_health: 5.0      # Mayor frecuencia
  logging:
    log_level: "DEBUG"      # Información detallada
    log_to_file: true       # Habilitar logging completo
```

## Contacto y Soporte

### 📞 **Soporte Técnico**
- **Email**: ing.marioalvarezvallejo@gmail.com
- **GitHub**: [TadeoRoboticsGroup](http://github.com/TadeoRoboticsGroup)
- **Issues**: [Reportar Problemas](https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws/issues)
- **Documentación**: [Sitio Web](https://tadeoroboticsgroup.github.io/TadeoRoboticsGroup/)

Para problemas técnicos, nuevas funcionalidades o contribuciones al sistema de arranque, usar el sistema de issues del repositorio.

## Licencia

**Repositorio Privado - Semillero de Robótica**

Copyright (c) 2024 Semillero de Robótica. Todos los derechos reservados.

Desarrollado como parte del proyecto eCar para investigación en robótica autónoma. Sistema integrado optimizado para robots móviles 4WD4WS con capacidades de navegación autónoma y comportamientos inteligentes.

### 🌐 **Enlaces del Proyecto**
- **Organización**: [TadeoRoboticsGroup](http://github.com/TadeoRoboticsGroup)
- **Repositorio**: [tadeo-eCar-ws](https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws)
- **Sitio Web**: [Semillero de Robótica](https://tadeoroboticsgroup.github.io/TadeoRoboticsGroup/)