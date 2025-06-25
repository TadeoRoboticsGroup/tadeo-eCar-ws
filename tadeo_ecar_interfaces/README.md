# Tadeo eCar Interfaces

Servicios y acciones para el robot autónomo Tadeo eCar 4WD4WS.

## 📋 Descripción

Este paquete define servicios ROS2 y acciones específicas para el sistema Tadeo eCar, proporcionando interfaces de comunicación síncronas y asíncronas para control y monitoreo del robot.

## 🔧 Servicios Disponibles

### SetRobotMode.srv
Cambio de modo operacional del robot:
- **Request:** modo solicitado (IDLE, MANUAL, AUTONOMOUS, EMERGENCY, MAINTENANCE)
- **Response:** éxito, mensaje, modo actual, tiempo de cambio
- **Uso:** Control de estados del robot

### CalibrateWheels.srv  
Calibración de controladores de ruedas:
- **Request:** tipo de calibración (ALL_WHEELS, FRONT_WHEELS, REAR_WHEELS, etc.)
- **Response:** resultado, duración, offsets de ruedas, límites de dirección
- **Uso:** Calibración del sistema 4WD4WS

### GetSystemStatus.srv
Estado completo del sistema:
- **Request:** inclusión de diagnósticos, métricas, historial
- **Response:** status completo, health, safety, wheel states
- **Uso:** Monitoreo integral del robot

### EmergencyStop.srv
Control de parada de emergencia:
- **Request:** activar/desactivar, razón, nivel de prioridad
- **Response:** éxito, tiempo de parada, status de sistemas
- **Uso:** Seguridad del robot

### SaveMap.srv
Guardado de mapas:
- **Request:** nombre, descripción, compresión, directorio
- **Response:** éxito, ruta del archivo, tamaño, resolución
- **Uso:** Gestión de mapas SLAM

## 🎯 Acciones Disponibles

### NavigateToGoal.action
Navegación autónoma a objetivo:
- **Goal:** pose objetivo, tolerancias, velocidad máxima, planner
- **Result:** éxito, errores finales, tiempo, distancia recorrida
- **Feedback:** pose actual, distancia restante, comportamiento actual
- **Uso:** Navegación punto a punto

### FollowPath.action
Seguimiento de trayectoria:
- **Goal:** path completo, velocidad, tolerancia, reversión
- **Result:** completación, velocidad promedio, desviación máxima
- **Feedback:** waypoint actual, progreso, error cross-track
- **Uso:** Ejecución de rutas predefinidas

### Dock.action
Acoplamiento autónomo:
- **Goal:** pose del dock, ID, velocidad, precisión, guía visual
- **Result:** éxito, errores finales, conexión de carga
- **Feedback:** fase actual (APPROACHING, ALIGNING, etc.), distancia
- **Uso:** Estacionamiento y carga automática

### Calibrate.action
Calibración del sistema:
- **Goal:** tipo (SENSORS, ACTUATORS, NAVIGATION, FULL_SYSTEM)
- **Result:** componentes calibrados/fallidos, precisión, tiempo
- **Feedback:** componente actual, progreso, tiempo estimado
- **Uso:** Calibración completa del robot

## 🔧 Uso

### Llamar servicio de modo
```cpp
#include "tadeo_ecar_interfaces/srv/set_robot_mode.hpp"

auto client = create_client<tadeo_ecar_interfaces::srv::SetRobotMode>("set_robot_mode");
auto request = std::make_shared<tadeo_ecar_interfaces::srv::SetRobotMode::Request>();

request->requested_mode = tadeo_ecar_interfaces::srv::SetRobotMode::Request::AUTONOMOUS;
request->reason = "Starting autonomous mission";
request->force_change = false;

auto future = client->async_send_request(request);
```

### Cliente de acción de navegación
```cpp
#include "tadeo_ecar_interfaces/action/navigate_to_goal.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigateToGoal = tadeo_ecar_interfaces::action::NavigateToGoal;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToGoal>;

auto action_client = rclcpp_action::create_client<NavigateToGoal>(this, "navigate_to_goal");

auto goal_msg = NavigateToGoal::Goal();
goal_msg.target_pose.pose.position.x = 5.0;
goal_msg.target_pose.pose.position.y = 2.0;
goal_msg.tolerance_distance = 0.1;
goal_msg.max_speed = 1.0;

auto future = action_client->async_send_goal(goal_msg);
```

## 📦 Dependencias

- `std_msgs` - Mensajes estándar
- `geometry_msgs` - Mensajes de geometría
- `sensor_msgs` - Mensajes de sensores  
- `nav_msgs` - Mensajes de navegación
- `tadeo_ecar_msgs` - Mensajes personalizados del robot
- `rosidl_default_generators` - Generación de interfaces

## 🏗️ Compilación

```bash
# Compilar este paquete (requiere tadeo_ecar_msgs)
colcon build --packages-select tadeo_ecar_interfaces

# Compilar con dependencias
colcon build --packages-up-to tadeo_ecar_interfaces
```

## 🔍 Verificación

```bash
# Listar servicios generados
ros2 interface list  < /dev/null |  grep tadeo_ecar_interfaces

# Ver definición de servicio
ros2 interface show tadeo_ecar_interfaces/srv/SetRobotMode

# Ver definición de acción
ros2 interface show tadeo_ecar_interfaces/action/NavigateToGoal

# Llamar servicio de prueba
ros2 service call /set_robot_mode tadeo_ecar_interfaces/srv/SetRobotMode "{requested_mode: 2, reason: test, force_change: false}"
```

## 📋 Servicios y Acciones Recomendados

### Servicios
| Servicio | Endpoint | Uso |
|----------|----------|-----|
| SetRobotMode | `/set_robot_mode` | Control de estados |
| CalibrateWheels | `/calibrate_wheels` | Calibración 4WD4WS |
| GetSystemStatus | `/get_system_status` | Monitoreo |
| EmergencyStop | `/emergency_stop` | Seguridad |
| SaveMap | `/save_map` | Gestión mapas |

### Acciones  
| Acción | Endpoint | Duración |
|--------|----------|----------|
| NavigateToGoal | `/navigate_to_goal` | Segundos-Minutos |
| FollowPath | `/follow_path` | Segundos-Minutos |
| Dock | `/dock` | 30-120 segundos |
| Calibrate | `/calibrate` | Minutos |

## 🔄 Flujo de Trabajo Típico

1. **Inicialización:** GetSystemStatus → SetRobotMode(AUTONOMOUS)
2. **Navegación:** NavigateToGoal → FollowPath
3. **Docking:** Dock action para estacionamiento
4. **Mantenimiento:** Calibrate → SaveMap

---
**Versión:** 1.0.0  
**Compatible:** ROS2 Humble  
**Licencia:** MIT

## Contacto y Soporte

### 📞 **Soporte Técnico**
- **Email**: ing.marioalvarezvallejo@gmail.com
- **GitHub**: [TadeoRoboticsGroup](http://github.com/TadeoRoboticsGroup)
- **Issues**: [Reportar Problemas](https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws/issues)
- **Sitio Web**: [Semillero de Robótica](https://tadeoroboticsgroup.github.io/TadeoRoboticsGroup/)

### 🌐 **Enlaces del Proyecto**
- **Organización**: [TadeoRoboticsGroup](http://github.com/TadeoRoboticsGroup)
- **Repositorio**: [tadeo-eCar-ws](https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws)
- **Sitio Web**: [Semillero de Robótica](https://tadeoroboticsgroup.github.io/TadeoRoboticsGroup/)
