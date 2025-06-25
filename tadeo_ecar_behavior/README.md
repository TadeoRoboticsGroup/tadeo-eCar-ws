# Sistema de Comportamientos Tadeo eCar

## Descripción General

El paquete `tadeo_ecar_behavior` implementa un sistema avanzado de comportamientos y toma de decisiones para el robot autónomo Tadeo eCar 4WD4WS. Este sistema utiliza **BehaviorTree.CPP** para crear comportamientos complejos, jerárquicos y reutilizables que permiten al robot operar de manera autónoma en diferentes escenarios.

## Características Principales

### 🌳 Árboles de Comportamientos Avanzados
- **BehaviorTree.CPP** para lógica de decisión robusta
- **Árboles XML configurables** para diferentes misiones
- **Nodos personalizados** para funcionalidades específicas del Tadeo eCar
- **Composición jerárquica** de comportamientos complejos
- **Ejecución en tiempo real** con control de flujo

### 🤖 Máquina de Estados Inteligente
- **Estados del robot bien definidos** (IDLE, NAVIGATING, EXPLORING, etc.)
- **Transiciones automáticas** basadas en condiciones del sistema
- **Validación de transiciones** para prevenir estados inválidos
- **Gestión de emergencias** con recuperación automática
- **Monitoreo de batería** con comportamientos de carga automática

### 📊 Monitoreo y Diagnóstico
- **Métricas de rendimiento** en tiempo real
- **Detección de anomalías** en el comportamiento del sistema
- **Alertas inteligentes** con niveles de severidad
- **Logging automático** para análisis posterior
- **Visualización en RViz** del estado del sistema

### 🔧 Arquitectura Modular
- **Nodos independientes** para diferentes funcionalidades
- **Comunicación mediante tópicos y servicios** estándar de ROS2
- **Configuración YAML** para parámetros del sistema
- **Integración completa** con otros subsistemas del Tadeo eCar

## Arquitectura del Sistema

```
tadeo_ecar_behavior/
├── src/
│   ├── behavior_manager_node.cpp         # Gestor principal con BehaviorTree
│   ├── state_machine_node.cpp            # Máquina de estados del robot
│   ├── behavior_monitor_node.cpp         # Monitor de rendimiento
│   └── bt_action_nodes/                   # Nodos de acción para BehaviorTree
│       ├── navigation_actions.cpp        # Acciones de navegación
│       ├── safety_conditions.cpp         # Condiciones de seguridad
│       ├── utility_actions.cpp           # Acciones utilitarias
│       ├── patrol_actions.cpp            # Acciones de patrullaje
│       └── exploration_actions.cpp       # Acciones de exploración
├── include/tadeo_ecar_behavior/
│   └── behavior_types.hpp                # Tipos y estructuras de datos
├── behavior_trees/                       # Árboles de comportamientos XML
│   ├── main_behavior.xml                 # Árbol principal
│   ├── patrol_behavior.xml               # Comportamientos de patrullaje
│   └── exploration_behavior.xml          # Comportamientos de exploración
├── config/
│   └── behavior_params.yaml              # Configuración de parámetros
├── launch/
│   └── behavior_system.launch.py         # Lanzamiento del sistema completo
└── README.md
```

## Nodos Principales

### 1. Behavior Manager Node
**Archivo:** `behavior_manager_node.cpp`
**Función:** Gestor principal del sistema de comportamientos

#### Funcionalidades:
- Carga y ejecución de árboles de comportamientos XML
- Gestión del blackboard para compartir datos entre nodos
- Integración con BehaviorTree.CPP y nodos personalizados
- Monitoreo opcional con Groot para debugging visual
- Manejo de comandos de comportamiento externos

#### Tópicos Principales:
```bash
# Suscripciones
/behavior_command              # Comandos para controlar comportamientos
/odom                         # Odometría del robot
/battery_state                # Estado de la batería
/emergency_stop               # Señal de parada de emergencia
/navigation_status            # Estado del sistema de navegación
/system_health                # Salud general del sistema

# Publicaciones
/behavior_status              # Estado actual de los comportamientos
/robot_state                  # Estado del robot
/behavior_metrics             # Métricas de rendimiento
/behavior_visualization       # Marcadores para RViz
/behavior/manager_health      # Salud del gestor de comportamientos
```

#### Servicios:
```bash
/execute_behavior             # Servicio para ejecutar comportamientos específicos
```

#### Comandos de Control:
```bash
# Iniciar árbol de comportamientos
rostopic pub /behavior_command std_msgs/String "data: 'start_tree:main_behavior'"

# Detener ejecución
rostopic pub /behavior_command std_msgs/String "data: 'stop_tree'"

# Recargar árbol
rostopic pub /behavior_command std_msgs/String "data: 'reload_tree'"

# Parada de emergencia
rostopic pub /behavior_command std_msgs/String "data: 'emergency_stop'"

# Reset del sistema
rostopic pub /behavior_command std_msgs/String "data: 'reset'"
```

### 2. State Machine Node
**Archivo:** `state_machine_node.cpp**
**Función:** Máquina de estados jerárquica del robot

#### Estados del Robot:
- **IDLE:** Robot en reposo, esperando comandos
- **INITIALIZING:** Inicializando sistemas del robot
- **NAVIGATING:** Navegando hacia un objetivo
- **EXPLORING:** Explorando el entorno
- **PATROLLING:** Realizando patrullaje
- **CHARGING:** Cargando batería en estación
- **EMERGENCY:** Estado de emergencia activo
- **MAINTENANCE:** Modo de mantenimiento
- **MISSION_EXECUTING:** Ejecutando misión específica
- **WAITING:** Esperando condiciones para continuar
- **RECOVERING:** Recuperándose de fallos
- **DOCKING:** Acoplándose a estación de carga

#### Transiciones Automáticas:
```bash
# Emergencia tiene máxima prioridad
ANY_STATE -> EMERGENCY (si emergency_stop == true)

# Batería crítica fuerza carga
ANY_STATE -> CHARGING (si battery_level < 0.15)

# Recuperación automática post-emergencia
EMERGENCY -> RECOVERING (si emergency_stop == false)

# Vuelta a idle tras recovery exitoso
RECOVERING -> IDLE (tras 10 segundos sin problemas)
```

#### Comandos de Estado:
```bash
# Forzar transición de estado
rostopic pub /state_command std_msgs/String "data: 'TRANSITION:NAVIGATING'"

# Estados disponibles: IDLE, NAVIGATING, EXPLORING, PATROLLING, 
# CHARGING, EMERGENCY, MAINTENANCE, MISSION_EXECUTING, WAITING, RECOVERING, DOCKING
```

### 3. Behavior Monitor Node
**Archivo:** `behavior_monitor_node.cpp`
**Función:** Monitoreo y diagnóstico del sistema de comportamientos

#### Métricas Monitoreadas:
- **Rendimiento General:** Puntuación de 0.0 a 1.0
- **Frescura de Datos:** Verificación de actualizaciones recientes
- **Tasa de Éxito:** Porcentaje de comportamientos exitosos
- **Tiempo de Respuesta:** Latencia del sistema
- **Detección de Anomalías:** Patrones anómalos en el comportamiento

#### Tipos de Anomalías Detectadas:
- **Sin actualizaciones de estado** por más de 30 segundos
- **Alta frecuencia de alertas** del sistema
- **Rendimiento consistentemente bajo** por períodos prolongados
- **Fallos recurrentes** en comportamientos específicos

#### Tópicos de Monitoreo:
```bash
/behavior_performance         # Puntuación de rendimiento actual (0.0-1.0)
/behavior_diagnostics        # Información de diagnóstico detallada
/behavior_monitor_viz        # Visualización del estado del monitor
/behavior/monitor_health     # Salud del sistema de monitoreo
```

## Árboles de Comportamientos

### 1. main_behavior.xml
**Propósito:** Árbol principal para operación general del robot

#### Estructura:
```xml
MainBehavior
├── SafetyChecks                    # Verificaciones de seguridad críticas
│   ├── CheckEmergency             # ¿Hay emergencia activa?
│   └── EmergencyResponse          # Respuesta a emergencia
├── BatteryManagement              # Gestión inteligente de batería
│   ├── CheckBattery               # ¿Batería suficiente?
│   └── LowBatteryProtocol         # Protocolo de batería baja
└── MainBehaviorLogic              # Lógica principal de comportamientos
    ├── MissionExecution           # Ejecución de misiones activas
    ├── NavigationMode             # Navegación a objetivos
    ├── PatrolMode                 # Modo de patrullaje
    ├── ExplorationMode            # Modo de exploración
    └── IdleBehavior               # Comportamiento por defecto
```

#### Prioridades de Ejecución:
1. **Seguridad:** Verificaciones de emergencia
2. **Batería:** Gestión de energía crítica
3. **Misiones:** Misiones activas definidas por usuario
4. **Navegación:** Objetivos individuales de navegación
5. **Patrullaje:** Rutas de patrullaje automático
6. **Exploración:** Exploración autónoma del entorno
7. **Idle:** Estado de reposo por defecto

### 2. patrol_behavior.xml
**Propósito:** Comportamientos específicos para patrullaje

#### Características:
- **Carga de rutas** desde archivos de configuración
- **Navegación secuencial** por puntos de patrullaje
- **Acciones en cada punto** (escaneo, detección, logging)
- **Recovery específico** para fallos de patrullaje
- **Loop infinito** con verificaciones de seguridad

#### Acciones de Patrullaje:
- **Escaneo láser** del área circundante
- **Captura de imágenes** con cámaras
- **Detección de intrusos** o anomalías
- **Actualización de base de datos** de patrullaje
- **Generación de reportes** automáticos

### 3. exploration_behavior.xml
**Propósito:** Exploración autónoma inteligente

#### Estrategias de Exploración:
- **Exploración fronteriza:** Busca y explora fronteras del mapa conocido
- **Áreas desconocidas:** Navega hacia regiones no mapeadas
- **Exploración sistemática:** Patrones regulares de cobertura
- **Exploración aleatoria:** Fallback para situaciones complejas

#### Safety durante Exploración:
- **Verificación de batería** antes de áreas lejanas
- **Distancia máxima** desde punto de origen
- **Navegación segura** con evitación de obstáculos
- **Recovery específico** para situaciones de exploración

## Nodos de Acción Personalizados

### Nodos de Navegación
**Archivo:** `navigation_actions.cpp`

#### NavigateToGoalAction
- **Función:** Navegar a coordenadas específicas
- **Parámetros:** goal_x, goal_y, goal_theta, timeout
- **Integración:** Usa Nav2 action server
- **Resultado:** SUCCESS/FAILURE con mensaje descriptivo

#### ClearCostmapsAction
- **Función:** Limpiar costmaps global y local
- **Uso:** Recovery tras fallos de navegación
- **Servicios:** global_costmap/clear, local_costmap/clear

#### RotateRecoveryAction
- **Función:** Rotación de recovery (360° por defecto)
- **Parámetros:** rotation_speed, rotation_angle
- **Uso:** Desobstaculizar el robot

#### BackupRecoveryAction
- **Función:** Retroceso de recovery
- **Parámetros:** backup_speed, backup_distance
- **Uso:** Alejarse de obstáculos cercanos

### Nodos de Condiciones de Seguridad
**Archivo:** `safety_conditions.cpp`

#### CheckEmergencyCondition
- **Función:** Verificar estado de emergencia
- **Entrada:** Tópico /emergency_stop
- **Resultado:** SUCCESS si no hay emergencia, FAILURE si hay

#### CheckBatteryCondition
- **Función:** Verificar nivel de batería
- **Parámetros:** threshold (umbral mínimo)
- **Resultado:** SUCCESS si batería > umbral

#### CheckSensorsHealthyCondition
- **Función:** Verificar salud de sensores
- **Sensores:** Láser, cámaras, IMU
- **Resultado:** SUCCESS si todos los sensores están OK

#### CheckNavigationReadyCondition
- **Función:** Verificar que navegación está lista
- **Verificaciones:** Nav2 funcionando, odometría disponible
- **Resultado:** SUCCESS si navegación está operativa

### Nodos de Utilidades
**Archivo:** `utility_actions.cpp`

#### WaitAction
- **Función:** Esperar tiempo determinado
- **Parámetros:** duration (segundos)
- **Uso:** Pausas controladas en comportamientos

#### SendAlertAction
- **Función:** Enviar alertas del sistema
- **Parámetros:** message, level (INFO/WARN/ERROR/CRITICAL)
- **Publicación:** /system_alerts

#### StopRobotAction
- **Función:** Detener robot inmediatamente
- **Comando:** Publica velocidad cero múltiples veces
- **Uso:** Paradas de emergencia

#### SetRobotStateAction
- **Función:** Cambiar estado del robot
- **Parámetros:** state (IDLE, NAVIGATING, etc.)
- **Validación:** Verifica estados válidos

#### RequestHumanAssistanceAction
- **Función:** Solicitar asistencia humana
- **Parámetros:** reason, urgency (LOW/MEDIUM/HIGH/CRITICAL)
- **Publicación:** /human_assistance_request

## Configuración y Parámetros

### Archivo de Configuración Principal
**Ubicación:** `config/behavior_params.yaml`

#### Configuración del Behavior Manager:
```yaml
behavior_manager_node:
  ros__parameters:
    behavior_frequency: 10.0              # Hz - Frecuencia de tick del árbol
    default_tree_file: "main_behavior.xml" # Árbol por defecto
    behavior_trees_path: "/opt/ros/humble/share/tadeo_ecar_behavior/behavior_trees/"
    enable_behavior_logging: true         # Logging detallado
    enable_groot_monitoring: false        # Monitoreo con Groot
    groot_publisher_port: 1666            # Puerto ZMQ para Groot
    behavior_timeout_default: 30.0        # segundos - Timeout por defecto
    max_concurrent_behaviors: 5           # Máximo comportamientos concurrentes
    auto_recovery_enabled: true           # Recovery automático
```

#### Configuración de Comportamientos Específicos:
```yaml
# Navegación
navigation_behavior:
  goal_tolerance: 0.3                    # metros
  navigation_timeout: 120.0              # segundos
  max_retries: 3                         # Reintentos máximos
  recovery_enabled: true                 # Comportamientos de recovery

# Patrullaje  
patrol_behavior:
  default_patrol_route: "patrol_route_1" # Ruta por defecto
  patrol_point_tolerance: 0.5            # metros
  scan_duration: 5.0                     # segundos en cada punto
  max_patrol_retries: 2                  # Reintentos por punto

# Exploración
exploration_behavior:
  exploration_strategy: "frontier"        # frontier, systematic, random
  frontier_min_size: 5                   # Tamaño mínimo de frontera
  completion_threshold: 0.95             # 95% para completar exploración
  safe_distance: 1.0                     # metros de distancia segura

# Batería
battery_behavior:
  critical_battery_level: 0.15           # 15% - nivel crítico
  low_battery_level: 0.25                # 25% - nivel bajo
  full_battery_level: 0.95               # 95% - carga completa
```

## Uso del Sistema

### 1. Lanzamiento Completo del Sistema
```bash
# Lanzar sistema completo de comportamientos
ros2 launch tadeo_ecar_behavior behavior_system.launch.py

# Con parámetros personalizados
ros2 launch tadeo_ecar_behavior behavior_system.launch.py \
    use_sim_time:=true \
    enable_behavior_manager:=true \
    enable_state_machine:=true \
    enable_monitor:=true \
    behavior_tree_file:=main_behavior.xml \
    enable_groot:=false \
    log_level:=info
```

### 2. Control Manual de Comportamientos

#### Comandos Básicos:
```bash
# Iniciar árbol principal
rostopic pub /behavior_command std_msgs/String "data: 'start_tree:main_behavior'"

# Cambiar a árbol de patrullaje
rostopic pub /behavior_command std_msgs/String "data: 'start_tree:patrol_behavior'"

# Cambiar a árbol de exploración
rostopic pub /behavior_command std_msgs/String "data: 'start_tree:exploration_behavior'"

# Detener comportamientos
rostopic pub /behavior_command std_msgs/String "data: 'stop_tree'"

# Parada de emergencia
rostopic pub /behavior_command std_msgs/String "data: 'emergency_stop'"
```

#### Control de Estados:
```bash
# Cambiar a modo navegación
rostopic pub /state_command std_msgs/String "data: 'TRANSITION:NAVIGATING'"

# Activar modo patrullaje
rostopic pub /state_command std_msgs/String "data: 'TRANSITION:PATROLLING'"

# Forzar modo carga
rostopic pub /state_command std_msgs/String "data: 'TRANSITION:CHARGING'"

# Volver a idle
rostopic pub /state_command std_msgs/String "data: 'TRANSITION:IDLE'"
```

### 3. Monitoreo del Sistema

#### Verificar Estado Actual:
```bash
# Estado del robot
rostopic echo /robot_state

# Estado de comportamientos
rostopic echo /behavior_status

# Métricas de rendimiento
rostopic echo /behavior_metrics

# Alertas del sistema
rostopic echo /system_alerts

# Diagnósticos detallados
rostopic echo /behavior_diagnostics
```

#### Verificar Salud de Componentes:
```bash
# Salud del gestor de comportamientos
rostopic echo /behavior/manager_health

# Salud de la máquina de estados
rostopic echo /behavior/state_health

# Salud del monitor
rostopic echo /behavior/monitor_health
```

### 4. Debugging y Desarrollo

#### Habilitar Groot para Visualización:
```bash
# Lanzar con Groot habilitado
ros2 launch tadeo_ecar_behavior behavior_system.launch.py \
    enable_groot:=true \
    groot_port:=1666

# Conectar Groot (requiere instalación por separado)
# Groot se conecta automáticamente al puerto ZMQ 1666
```

#### Logging Avanzado:
```bash
# Habilitar logging detallado
ros2 launch tadeo_ecar_behavior behavior_system.launch.py \
    log_level:=debug \
    record_behaviors:=true

# Los logs se guardan en /tmp/tadeo_behavior_logs/
```

#### Herramientas de Desarrollo:
```bash
# Lanzar con herramientas de debugging
ros2 launch tadeo_ecar_behavior behavior_system.launch.py \
    use_rqt:=true \
    use_rqt_graph:=true \
    echo_topics:=true
```

## Integración con Otros Sistemas

### Dependencias del Tadeo eCar:
- **tadeo_ecar_msgs:** Mensajes de salud del sistema
- **tadeo_ecar_interfaces:** Servicios de ejecución de comportamientos
- **tadeo_ecar_navigation:** Estado de navegación y objetivos
- **tadeo_ecar_perception:** Salud de sensores
- **tadeo_ecar_safety:** Señales de emergencia

### Dependencias Externas:
- **BehaviorTree.CPP v3:** Motor de árboles de comportamientos
- **Nav2:** Sistema de navegación
- **yaml-cpp:** Procesamiento de archivos de configuración
- **ROS2 Lifecycle:** Gestión del ciclo de vida de nodos

## Creación de Comportamientos Personalizados

### 1. Añadir Nuevo Nodo de Acción

#### Paso 1: Definir la Clase
```cpp
// En bt_action_nodes/custom_actions.cpp
class MyCustomAction : public BT::SyncActionNode
{
public:
    MyCustomAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("my_custom_action_bt_node");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("parameter1", "Description of parameter1"),
            BT::OutputPort<std::string>("result", "Action result")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string param1;
        if (!getInput("parameter1", param1)) {
            setOutput("result", "Missing parameter1");
            return BT::NodeStatus::FAILURE;
        }
        
        // Implementar lógica personalizada aquí
        
        setOutput("result", "Custom action completed");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
};
```

#### Paso 2: Registrar en el Factory
```cpp
// En behavior_manager_node.cpp, método registerCustomNodes()
bt_factory_->registerNodeType<MyCustomAction>("MyCustomAction");
```

#### Paso 3: Usar en Árbol XML
```xml
<!-- En archivo XML del árbol -->
<MyCustomAction parameter1="mi_valor" result="{action_result}" />
```

### 2. Crear Nuevo Árbol de Comportamientos

#### Paso 1: Crear Archivo XML
```xml
<!-- En behavior_trees/my_behavior.xml -->
<?xml version="1.0"?>
<root main_tree_to_execute="MyCustomBehavior">
    <BehaviorTree ID="MyCustomBehavior">
        <Sequence name="MySequence">
            <CheckEmergency />
            <MyCustomAction parameter1="test" />
            <SendAlert message="Custom behavior executed" level="INFO" />
        </Sequence>
    </BehaviorTree>
</root>
```

#### Paso 2: Lanzar Árbol Personalizado
```bash
ros2 launch tadeo_ecar_behavior behavior_system.launch.py \
    behavior_tree_file:=my_behavior.xml
```

## Solución de Problemas

### Problemas Comunes y Soluciones:

#### 1. BehaviorTree No Se Ejecuta
```bash
# Verificar estado del behavior manager
rostopic echo /behavior_status

# Verificar que el árbol está cargado
rostopic echo /behavior/manager_health

# Iniciar manualmente
rostopic pub /behavior_command std_msgs/String "data: 'start_tree:main_behavior'"
```

#### 2. Estados de Robot Incorrectos
```bash
# Verificar máquina de estados
rostopic echo /robot_state
rostopic echo /state_transitions

# Forzar estado correcto
rostopic pub /state_command std_msgs/String "data: 'TRANSITION:IDLE'"

# Verificar condiciones de emergencia
rostopic echo /emergency_stop
```

#### 3. Rendimiento Degradado
```bash
# Monitorear métricas
rostopic echo /behavior_performance
rostopic echo /behavior_diagnostics

# Verificar anomalías
rostopic echo /system_alerts

# Revisar logs del monitor
cat /tmp/tadeo_behavior_monitor/*.log
```

#### 4. Nodos de Acción Fallan
```bash
# Verificar logging detallado
ros2 launch tadeo_ecar_behavior behavior_system.launch.py log_level:=debug

# Usar Groot para debugging visual (si está instalado)
ros2 launch tadeo_ecar_behavior behavior_system.launch.py enable_groot:=true

# Verificar dependencias de nodos
ros2 node list
ros2 service list
ros2 topic list
```

### Logs de Diagnóstico:
```bash
# Logs del sistema en tiempo real
ros2 node info /behavior_manager_node
ros2 node info /state_machine_node  
ros2 node info /behavior_monitor_node

# Verificar tópicos y servicios
ros2 topic info /behavior_status
ros2 service info /execute_behavior
```

## Desarrollo y Extensión

### Arquitectura para Nuevas Funcionalidades:
1. **Nuevos estados del robot:** Modificar `RobotState` enum en `behavior_types.hpp`
2. **Nuevas condiciones:** Crear clases derivadas de `BT::ConditionNode`
3. **Nuevas acciones:** Crear clases derivadas de `BT::SyncActionNode` o `BT::AsyncActionNode`
4. **Nuevos árboles:** Crear archivos XML en directorio `behavior_trees/`

### Mejores Prácticas:
- **Nodos atómicos:** Mantener acciones simples y específicas
- **Reutilización:** Diseñar nodos que puedan usarse en múltiples árboles
- **Error handling:** Siempre manejar casos de fallo graciosamente
- **Timeouts:** Usar timeouts apropiados para evitar bloqueos
- **Logging:** Incluir logging detallado para debugging

## Rendimiento y Optimización

### Configuraciones Recomendadas:

#### Para Uso en Producción:
```yaml
behavior_frequency: 10.0              # Frecuencia estándar
enable_behavior_logging: false        # Reducir overhead
enable_groot_monitoring: false        # Solo para desarrollo
auto_recovery_enabled: true           # Recuperación automática
```

#### Para Desarrollo y Testing:
```yaml
behavior_frequency: 5.0               # Frecuencia reducida para debugging
enable_behavior_logging: true         # Logging completo
enable_groot_monitoring: true         # Visualización en tiempo real
log_level: "DEBUG"                    # Información detallada
```

#### Para Sistemas de Alto Rendimiento:
```yaml
behavior_frequency: 20.0              # Mayor frecuencia
max_concurrent_behaviors: 10          # Más comportamientos paralelos
monitor_frequency: 5.0                # Monitoreo más frecuente
```

## Licencia y Créditos

**Repositorio Privado - Semillero de Robótica**

Copyright (c) 2024 Semillero de Robótica. Todos los derechos reservados.

Desarrollado como parte del proyecto eCar para investigación en comportamientos robóticos autónomos. El sistema utiliza BehaviorTree.CPP para crear comportamientos robustos y reutilizables optimizados para robots móviles 4WD4WS.

### 🌐 **Enlaces del Proyecto**
- **Organización**: [TadeoRoboticsGroup](http://github.com/TadeoRoboticsGroup)
- **Repositorio**: [tadeo-eCar-ws](https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws)
- **Sitio Web**: [Semillero de Robótica](https://tadeoroboticsgroup.github.io/TadeoRoboticsGroup/)

## Contacto y Soporte

### 📞 **Soporte Técnico**
- **Email**: ing.marioalvarezvallejo@gmail.com
- **GitHub**: [TadeoRoboticsGroup](http://github.com/TadeoRoboticsGroup)
- **Issues**: [Reportar Problemas](https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws/issues)

Para problemas técnicos, nuevas funcionalidades o contribuciones al sistema de comportamientos, usar el sistema de issues del repositorio.