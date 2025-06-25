# Sistema de Navegación Tadeo eCar

## Descripción General

El paquete `tadeo_ecar_navigation` implementa un sistema completo de navegación autónoma para el robot Tadeo eCar 4WD4WS. Este sistema incluye control de navegación avanzado, ejecución de misiones, gestión interactiva de waypoints y monitoreo en tiempo real del rendimiento de navegación.

## Características Principales

### 🎯 Capacidades de Navegación
- **Navegación autónoma** con integración completa de Nav2
- **Control 4WD4WS** (4 ruedas motrices, 4 ruedas direccionales)
- **Planificación de rutas** con evitación dinámica de obstáculos
- **Seguimiento de trayectorias** con control predictivo
- **Recuperación automática** ante situaciones de bloqueo

### 🗺️ Gestión de Misiones
- **Creación interactiva** de waypoints mediante RViz
- **Ejecución secuencial** de misiones complejas
- **Almacenamiento persistente** en formato YAML
- **Control de ejecución** (iniciar, pausar, reanudar, abortar)
- **Misiones cíclicas** con bucles automáticos

### 📊 Monitoreo y Análisis
- **Métricas de rendimiento** en tiempo real
- **Análisis de calidad** de navegación
- **Detección de proximidad** a obstáculos
- **Evaluación de suavidad** de trayectorias
- **Alertas de seguridad** automáticas

### 🔒 Seguridad Avanzada
- **Monitoreo continuo** de sistemas críticos
- **Parada de emergencia** automática
- **Límites de velocidad** configurables
- **Detección de estancamiento** del robot
- **Alertas graduales** por niveles de criticidad

## Arquitectura del Sistema

```
tadeo_ecar_navigation/
├── src/
│   ├── navigation_controller_node.cpp    # Control principal de navegación
│   ├── mission_executor_node.cpp         # Ejecutor de misiones
│   ├── waypoint_manager_node.cpp         # Gestor de waypoints
│   └── navigation_monitor_node.cpp       # Monitor de rendimiento
├── include/tadeo_ecar_navigation/
│   └── navigation_types.hpp              # Tipos y estructuras de datos
├── config/
│   └── navigation_params.yaml            # Parámetros de configuración
├── launch/
│   └── navigation_system.launch.py       # Lanzamiento del sistema completo
├── rviz/
│   └── navigation.rviz                   # Configuración de visualización
└── README.md
```

## Nodos Principales

### 1. Navigation Controller Node
**Archivo:** `navigation_controller_node.cpp`
**Función:** Control principal del sistema de navegación

#### Funcionalidades:
- Interfaz con Nav2 para navegación global y local
- Control de velocidad con límites de seguridad
- Gestión de modos de navegación (manual, autónomo, emergencia)
- Seguimiento de misiones con múltiples waypoints
- Comportamientos de recuperación automática

#### Tópicos Principales:
```bash
# Suscripciones
/move_base_simple/goal          # Objetivos de navegación
/odom                          # Odometría del robot
/mission_path                  # Rutas de misión
/navigation_mode               # Modo de navegación
/emergency_stop                # Parada de emergencia

# Publicaciones
/cmd_vel                       # Comandos de velocidad
/navigation_status             # Estado de navegación
/current_goal                  # Objetivo actual
/navigation_markers            # Marcadores de visualización
```

#### Servicios y Acciones:
```bash
/navigate_to_pose              # Servicio de navegación a pose
/follow_path                   # Acción de seguimiento de ruta
```

### 2. Mission Executor Node
**Archivo:** `mission_executor_node.cpp`
**Función:** Ejecución y gestión de misiones complejas

#### Funcionalidades:
- Carga automática de misiones desde archivos YAML
- Control de ejecución (inicio, pausa, reanudación, parada)
- Seguimiento de progreso con métricas detalladas
- Reintentos automáticos en caso de fallos
- Logging completo de ejecución

#### Comandos de Control:
```bash
# Comandos de misión
rostopic pub /mission_command std_msgs/String "data: 'START'"
rostopic pub /mission_command std_msgs/String "data: 'PAUSE'"
rostopic pub /mission_command std_msgs/String "data: 'RESUME'"
rostopic pub /mission_command std_msgs/String "data: 'STOP'"
rostopic pub /mission_command std_msgs/String "data: 'LOAD:1'"
```

#### Estados de Misión:
- **IDLE:** Esperando comandos
- **LOADED:** Misión cargada y lista
- **EXECUTING:** Ejecutando misión activamente
- **PAUSED:** Misión pausada temporalmente
- **COMPLETED:** Misión completada exitosamente
- **FAILED:** Misión falló y requiere intervención
- **STOPPED:** Misión detenida por usuario

### 3. Waypoint Manager Node
**Archivo:** `waypoint_manager_node.cpp`
**Función:** Gestión interactiva de waypoints y creación de misiones

#### Funcionalidades:
- Creación interactiva mediante clics en RViz
- Edición visual con marcadores 3D
- Optimización automática de rutas
- Almacenamiento automático y manual
- Validación de waypoints

#### Modo de Edición:
```bash
# Activar modo de edición
rostopic pub /waypoint_command std_msgs/String "data: 'START_EDITING'"

# Finalizar edición y crear misión
rostopic pub /waypoint_command std_msgs/String "data: 'STOP_EDITING'"

# Comandos adicionales
rostopic pub /waypoint_command std_msgs/String "data: 'CLEAR_ALL'"
rostopic pub /waypoint_command std_msgs/String "data: 'UNDO_LAST'"
rostopic pub /waypoint_command std_msgs/String "data: 'OPTIMIZE_PATH'"
rostopic pub /waypoint_command std_msgs/String "data: 'SAVE_MISSION'"
```

#### Creación de Waypoints:
1. **Modo Interactivo:** Usar "Publish Point" en RViz
2. **Modo Objetivo:** Usar "2D Nav Goal" en RViz  
3. **Modo Servicio:** Llamar al servicio `/add_waypoint_service`

### 4. Navigation Monitor Node
**Archivo:** `navigation_monitor_node.cpp`
**Función:** Monitoreo en tiempo real del rendimiento y seguridad

#### Métricas Monitoreadas:
- **Calidad de Navegación:** Puntuación global de 0-100%
- **Proximidad a Obstáculos:** Distancia mínima detectada
- **Desviación de Ruta:** Diferencia con ruta planificada
- **Suavidad de Velocidad:** Análisis de aceleraciones
- **Frescura de Datos:** Validez temporal de sensores

#### Alertas de Seguridad:
- **CRITICAL:** Emergencias que requieren parada inmediata
- **HIGH:** Situaciones peligrosas que necesitan atención
- **MEDIUM:** Advertencias de rendimiento degradado
- **INFO:** Información general del sistema

#### Tópicos de Monitoreo:
```bash
/navigation_metrics            # Métricas de rendimiento
/safety_alerts                # Alertas de seguridad
/navigation_quality           # Puntuación de calidad (0-1)
/monitor_visualization        # Visualización del monitor
```

## Configuración y Parámetros

### Archivo de Configuración Principal
**Ubicación:** `config/navigation_params.yaml`

#### Parámetros del Controlador de Navegación:
```yaml
navigation_controller_node:
  ros__parameters:
    control_frequency: 20.0      # Hz - Frecuencia de control
    goal_tolerance: 0.3          # metros - Tolerancia de objetivo
    angle_tolerance: 0.2         # radianes - Tolerancia angular
    max_linear_velocity: 1.5     # m/s - Velocidad máxima
    max_angular_velocity: 1.0    # rad/s - Velocidad angular máxima
    use_nav2_stack: true         # Usar Nav2 completo
    enable_safety_monitoring: true
```

#### Parámetros del Ejecutor de Misiones:
```yaml
mission_executor_node:
  ros__parameters:
    execution_frequency: 5.0     # Hz - Frecuencia de ejecución
    max_mission_attempts: 3      # Intentos máximos por misión
    waypoint_timeout: 60.0       # segundos - Timeout por waypoint
    mission_timeout: 1800.0      # segundos - Timeout total de misión
    enable_mission_logging: true # Habilitar logging detallado
```

#### Parámetros del Gestor de Waypoints:
```yaml
waypoint_manager_node:
  ros__parameters:
    update_frequency: 10.0       # Hz - Frecuencia de actualización
    max_waypoints_per_mission: 50    # Máximo waypoints por misión
    default_waypoint_tolerance: 0.3  # metros - Tolerancia por defecto
    auto_save_interval: 30.0     # segundos - Auto-guardado
    enable_interactive_markers: true # Marcadores interactivos
```

#### Parámetros del Monitor:
```yaml
navigation_monitor_node:
  ros__parameters:
    monitor_frequency: 10.0      # Hz - Frecuencia de monitoreo
    safety_distance_threshold: 0.5   # metros - Distancia segura
    alert_threshold_high: 0.8    # Umbral alto de calidad
    alert_threshold_medium: 0.6  # Umbral medio de calidad
    performance_logging_enabled: true
```

## Uso del Sistema

### 1. Lanzamiento Completo del Sistema
```bash
# Lanzar sistema completo de navegación
ros2 launch tadeo_ecar_navigation navigation_system.launch.py

# Con parámetros personalizados
ros2 launch tadeo_ecar_navigation navigation_system.launch.py \
    use_sim_time:=true \
    enable_monitor:=true \
    enable_waypoint_manager:=true \
    use_rviz:=true \
    log_level:=info
```

### 2. Creación de Misiones Paso a Paso

#### Paso 1: Activar Modo de Edición
```bash
rostopic pub /waypoint_command std_msgs/String "data: 'START_EDITING'"
```

#### Paso 2: Agregar Waypoints
- Abrir RViz con la configuración proporcionada
- Usar herramienta "Publish Point" para crear waypoints
- Los waypoints aparecerán como esferas naranjas numeradas

#### Paso 3: Optimizar Ruta (Opcional)
```bash
rostopic pub /waypoint_command std_msgs/String "data: 'OPTIMIZE_PATH'"
```

#### Paso 4: Finalizar y Guardar Misión
```bash
rostopic pub /waypoint_command std_msgs/String "data: 'STOP_EDITING'"
rostopic pub /waypoint_command std_msgs/String "data: 'SAVE_MISSION'"
```

### 3. Ejecución de Misiones

#### Cargar Misión Específica:
```bash
rostopic pub /mission_command std_msgs/String "data: 'LOAD:1'"
```

#### Iniciar Ejecución:
```bash
rostopic pub /mission_command std_msgs/String "data: 'START'"
```

#### Control Durante Ejecución:
```bash
# Pausar misión
rostopic pub /mission_command std_msgs/String "data: 'PAUSE'"

# Reanudar misión
rostopic pub /mission_command std_msgs/String "data: 'RESUME'"

# Detener misión
rostopic pub /mission_command std_msgs/String "data: 'STOP'"

# Abortar misión (emergencia)
rostopic pub /mission_command std_msgs/String "data: 'ABORT'"
```

### 4. Monitoreo y Diagnóstico

#### Verificar Estado del Sistema:
```bash
# Estado de navegación
rostopic echo /navigation_status

# Progreso de misión
rostopic echo /mission_progress

# Métricas de rendimiento
rostopic echo /navigation_metrics

# Alertas de seguridad
rostopic echo /safety_alerts

# Calidad de navegación (0.0 - 1.0)
rostopic echo /navigation_quality
```

#### Verificar Salud de Componentes:
```bash
# Salud del controlador
rostopic echo /navigation/controller_health

# Salud del ejecutor de misiones
rostopic echo /navigation/mission_health

# Salud del gestor de waypoints
rostopic echo /navigation/waypoint_health

# Salud del monitor
rostopic echo /navigation/monitor_health
```

## Integración con Otros Sistemas

### Dependencias de Paquetes Tadeo eCar:
- **tadeo_ecar_msgs:** Mensajes personalizados de salud del sistema
- **tadeo_ecar_interfaces:** Servicios y acciones de navegación
- **tadeo_ecar_config:** Configuraciones globales del robot
- **tadeo_ecar_perception:** Datos de sensores para navegación
- **tadeo_ecar_planning:** Planificación de rutas y trayectorias
- **tadeo_ecar_safety:** Sistema de seguridad y paradas de emergencia

### Dependencias Externas:
- **Nav2:** Stack completo de navegación de ROS2
- **AMCL:** Localización Monte Carlo adaptativa
- **tf2:** Transformadas entre marcos de referencia
- **yaml-cpp:** Procesamiento de archivos de configuración
- **Eigen3:** Operaciones matemáticas y vectoriales

## Estructura de Archivos de Misión

### Formato YAML de Misiones:
```yaml
# Ejemplo: mission_1.yaml
id: 1
name: "Misión de Patrullaje"
description: "Ruta de patrullaje por el área principal"
loop_mission: true
max_attempts: 3
total_distance: 25.4
waypoints:
  - id: 0
    name: "WP_Start"
    pose:
      x: 0.0
      y: 0.0
      theta: 0.0
    tolerance: 0.3
    max_velocity: 1.0
    is_mandatory: true
  - id: 1
    name: "WP_Corner1"
    pose:
      x: 5.0
      y: 0.0
      theta: 1.57
    tolerance: 0.3
    max_velocity: 0.8
    is_mandatory: true
  # ... más waypoints
```

### Metadatos de Waypoints:
Los waypoints pueden incluir metadatos personalizados para acciones específicas:
```yaml
metadata:
  wait_time: "5.0"          # Esperar 5 segundos en el waypoint
  action: "take_photo"      # Ejecutar acción personalizada
  priority: "high"          # Prioridad del waypoint
  sensor_check: "true"      # Verificar sensores en este punto
```

## Visualización en RViz

### Elementos Visualizados:
1. **Mapa del entorno** con obstáculos estáticos
2. **Costmaps global y local** con información de obstáculos
3. **Rutas planificadas** (global y local)
4. **Waypoints activos** con numeración secuencial
5. **Objetivo actual** con flecha direccional
6. **Trayectoria de misión** completa
7. **Estado del robot** con pose y velocidad
8. **Indicadores de calidad** de navegación

### Herramientas Interactivas:
- **2D Pose Estimate:** Establecer pose inicial del robot
- **2D Nav Goal:** Enviar objetivo de navegación directa
- **Publish Point:** Crear nuevos waypoints
- **Measure:** Medir distancias en el mapa

## Solución de Problemas

### Problemas Comunes y Soluciones:

#### 1. Robot No Responde a Objetivos
```bash
# Verificar estado de navegación
rostopic echo /navigation_status

# Verificar modo de navegación
rostopic pub /navigation_mode std_msgs/String "data: 'AUTONOMOUS'"

# Verificar parada de emergencia
rostopic pub /emergency_stop std_msgs/Bool "data: false"
```

#### 2. Misión No Se Ejecuta
```bash
# Verificar si la misión está cargada
rostopic echo /mission_status

# Verificar waypoints disponibles
rostopic echo /waypoint_list

# Cargar misión manualmente
rostopic pub /mission_command std_msgs/String "data: 'LOAD:1'"
```

#### 3. Calidad de Navegación Baja
```bash
# Monitorear métricas detalladas
rostopic echo /navigation_metrics

# Verificar alertas de seguridad
rostopic echo /safety_alerts

# Revisar datos de sensores
rostopic echo /scan
rostopic echo /odom
```

#### 4. Waypoints No Se Crean
```bash
# Verificar modo de edición
rostopic echo /waypoint_status

# Activar modo de edición
rostopic pub /waypoint_command std_msgs/String "data: 'START_EDITING'"

# Verificar marcos de referencia
rostopic echo /tf
```

### Logs de Diagnóstico:
```bash
# Logs del sistema en tiempo real
ros2 node list
ros2 topic list
ros2 service list

# Información detallada de nodos
ros2 node info /navigation_controller_node
ros2 node info /mission_executor_node
ros2 node info /waypoint_manager_node
ros2 node info /navigation_monitor_node
```

## Desarrollo y Personalización

### Agregar Nuevos Comportamientos de Recuperación:
1. Extender la clase `RecoveryAction` en `navigation_types.hpp`
2. Implementar lógica en `navigation_controller_node.cpp`
3. Agregar configuración en `navigation_params.yaml`

### Personalizar Métricas de Monitoreo:
1. Modificar `NavigationMetrics` en `navigation_types.hpp`
2. Actualizar cálculos en `navigation_monitor_node.cpp`
3. Ajustar umbrales en configuración

### Integrar Sensores Adicionales:
1. Agregar suscripciones en nodos relevantes
2. Incorporar datos en análisis de seguridad
3. Actualizar visualización en RViz

## Rendimiento y Optimización

### Configuraciones Recomendadas:

#### Para Robots Rápidos (>2 m/s):
```yaml
control_frequency: 30.0
max_linear_velocity: 2.5
goal_tolerance: 0.2
safety_distance_threshold: 0.8
```

#### Para Robots de Precisión:
```yaml
control_frequency: 50.0
goal_tolerance: 0.1
angle_tolerance: 0.1
default_waypoint_tolerance: 0.15
```

#### Para Entornos Complejos:
```yaml
monitor_frequency: 20.0
safety_distance_threshold: 1.0
enable_recovery_behaviors: true
costmap_analysis_enabled: true
```

## Licencia y Créditos

**Repositorio Privado - Semillero de Robótica**

Copyright (c) 2024 Semillero de Robótica. Todos los derechos reservados.

Desarrollado como parte del proyecto eCar para investigación en robótica móvil autónoma. Sistema optimizado para robots 4WD4WS con capacidades de navegación en interiores y exteriores.

### 🌐 **Enlaces del Proyecto**
- **Organización**: [TadeoRoboticsGroup](http://github.com/TadeoRoboticsGroup)
- **Repositorio**: [tadeo-eCar-ws](https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws)
- **Sitio Web**: [Semillero de Robótica](https://tadeoroboticsgroup.github.io/TadeoRoboticsGroup/)

## Contacto y Soporte

### 📞 **Soporte Técnico**
- **Email**: ing.marioalvarezvallejo@gmail.com
- **GitHub**: [TadeoRoboticsGroup](http://github.com/TadeoRoboticsGroup)
- **Issues**: [Reportar Problemas](https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws/issues)
- **Sitio Web**: [Semillero de Robótica](https://tadeoroboticsgroup.github.io/TadeoRoboticsGroup/)

Para problemas técnicos, mejoras o contribuciones al sistema de navegación, usar el sistema de issues del repositorio.