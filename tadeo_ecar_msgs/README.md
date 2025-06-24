# Tadeo eCar Messages

Mensajes personalizados para el robot autónomo Tadeo eCar 4WD4WS.

## 📋 Descripción

Este paquete define mensajes ROS2 específicos para el sistema Tadeo eCar, proporcionando interfaces de comunicación estructuradas entre los diferentes componentes del robot.

## 📨 Mensajes Disponibles

### RobotStatus.msg
Estado operacional completo del robot:
- Estados: IDLE, MANUAL, AUTONOMOUS, EMERGENCY, MAINTENANCE
- Sistema: ready, moving, charging, emergency_stop
- Batería: voltage, percentage, temperature
- Rendimiento: CPU, memory, network quality

### WheelStates.msg
Estados individuales de las 4 ruedas (4WD4WS):
- Velocidades: front_left/right, rear_left/right (rad/s)
- Ángulos de dirección: steering angles (rad)
- Torques: individual wheel torques (Nm)
- Deslizamiento: slip detection per wheel
- Posiciones: encoder positions (rad)

### SystemHealth.msg
Monitoreo de salud del sistema:
- Estados: HEALTHY, WARNING, ERROR, CRITICAL
- Componentes: CPU, memory, storage, network
- Sensores: LiDAR, camera, IMU, GPS
- Motores: status per motor controller
- Temperaturas: CPU, GPU, motors
- Diagnósticos: error codes, messages, uptime

### SafetyStatus.msg
Estado del sistema de seguridad:
- Estados de seguridad: SAFE, WARNING, DANGER, EMERGENCY
- Paradas de emergencia: hardware, software, remote
- Detección de colisiones: obstacles, distances
- Límites de velocidad: current, safety limits
- Monitoreo de sensores: operational status
- Zonas de seguridad: safe zones, boundaries
- Timeouts: communication, control, sensor
- Acciones: braking, speed reduction, replanning

### NavigationState.msg
Estado del sistema de navegación:
- Modos: IDLE, PLANNING, EXECUTING, PAUSED, COMPLETED, FAILED
- Objetivos: current goal, distance, time estimation
- Rutas: path points, waypoints, completion
- Localización: confidence, map matching
- Planificación: path availability, status
- Métricas: speed, length, deviation
- Obstáculos: static/dynamic detection

## 🔧 Uso

### Publicar estado del robot
```cpp
#include "tadeo_ecar_msgs/msg/robot_status.hpp"

auto status_msg = tadeo_ecar_msgs::msg::RobotStatus();
status_msg.header.stamp = this->now();
status_msg.robot_id = "tadeo_ecar_01";
status_msg.operational_state = tadeo_ecar_msgs::msg::RobotStatus::AUTONOMOUS;
status_msg.is_ready = true;
status_msg.battery_percentage = 85.5;

robot_status_pub_->publish(status_msg);
```

## 📦 Dependencias

- std_msgs - Mensajes estándar
- geometry_msgs - Mensajes de geometría  
- sensor_msgs - Mensajes de sensores
- rosidl_default_generators - Generación de interfaces

## 🏗️ Compilación

```bash
# Desde el workspace
colcon build --packages-select tadeo_ecar_msgs

# Solo este paquete
colcon build --packages-up-to tadeo_ecar_msgs
```

---
**Versión:** 1.0.0  
**Compatible:** ROS2 Humble  
**Licencia:** MIT
