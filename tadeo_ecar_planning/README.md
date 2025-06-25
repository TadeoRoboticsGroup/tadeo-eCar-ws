# Paquete de Planificación de Rutas Tadeo eCar

Este paquete proporciona capacidades avanzadas de planificación de rutas y generación de trayectorias para el robot autónomo Tadeo eCar. Implementa múltiples algoritmos de planificación incluyendo planificación global, planificación local, optimización de trayectorias y gestión integral de rutas.

## Resumen del Paquete

El paquete `tadeo_ecar_planning` proporciona:
- **Planificación Global**: Algoritmos de planificación de rutas de largo alcance usando OMPL
- **Planificación Local**: Evitación de obstáculos en tiempo real con DWA (Dynamic Window Approach)
- **Optimización de Trayectorias**: Refinamiento de rutas para suavidad y eficiencia
- **Gestión de Rutas**: Coordinación y monitoreo integral del sistema de navegación

## Estructura del Directorio

```
tadeo_ecar_planning/
├── include/tadeo_ecar_planning/
│   └── planning_types.hpp              # Estructuras de datos de planificación
├── src/
│   ├── global_path_planner_node.cpp   # Planificador global con OMPL
│   ├── local_path_planner_node.cpp    # Planificador local con DWA
│   ├── trajectory_optimizer_node.cpp  # Optimizador de trayectorias
│   └── path_manager_node.cpp          # Gestor coordinador de rutas
├── launch/
│   └── planning_system.launch.py      # Lanzamiento del sistema completo
├── config/
│   └── planning_params.yaml           # Parámetros de configuración
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Nodos

### 1. Nodo Planificador Global (`global_path_planner_node`)

Implementa planificación de rutas globales usando la librería OMPL con múltiples algoritmos de planificación.

**Temas Suscritos:**
- `map` (nav_msgs/OccupancyGrid): Mapa de ocupación del entorno
- `move_base_simple/goal` (geometry_msgs/PoseStamped): Meta de navegación
- `amcl_pose` (geometry_msgs/PoseWithCovarianceStamped): Pose actual del robot

**Temas Publicados:**
- `global_path` (nav_msgs/Path): Ruta global planificada
- `global_path_markers` (visualization_msgs/MarkerArray): Visualización de la ruta
- `planning/global_health` (tadeo_ecar_msgs/SystemHealth): Estado de salud del planificador

**Servicios:**
- `plan_global_path` (tadeo_ecar_interfaces/NavigateToGoal): Planificación de ruta bajo demanda

**Características Principales:**
- Soporte para múltiples algoritmos OMPL: RRT*, PRM, EST
- Verificación de colisiones considerando radio del robot y margen de seguridad
- Replanificación dinámica basada en desviación de ruta
- Suavizado de rutas opcional para mejorar la navegabilidad
- Manejo robusto de límites del mapa y obstáculos

**Parámetros:**
```yaml
global_path_planner_node:
  planning_frequency: 2.0                    # Frecuencia de planificación (Hz)
  planner_type: "RRTstar"                   # Algoritmo: "RRTstar", "PRM", "EST"
  max_planning_time: 5.0                    # Tiempo máximo de planificación (s)
  goal_tolerance: 0.2                       # Tolerancia de meta (m)
  robot_radius: 0.3                         # Radio del robot (m)
  safety_distance: 0.1                      # Distancia de seguridad (m)
  path_smoothing_enabled: true              # Activar suavizado de ruta
  dynamic_replanning: true                  # Replanificación dinámica
```

### 2. Nodo Planificador Local (`local_path_planner_node`)

Implementa planificación local y evitación de obstáculos usando Dynamic Window Approach (DWA) y Pure Pursuit.

**Temas Suscritos:**
- `global_path` (nav_msgs/Path): Ruta global de referencia
- `odom` (nav_msgs/Odometry): Odometría del robot
- `scan` (sensor_msgs/LaserScan): Datos del sensor LiDAR

**Temas Publicados:**
- `cmd_vel` (geometry_msgs/Twist): Comandos de velocidad
- `local_path` (nav_msgs/Path): Trayectoria local planificada
- `local_trajectory_markers` (visualization_msgs/MarkerArray): Visualización de trayectoria
- `planning/local_health` (tadeo_ecar_msgs/SystemHealth): Estado de salud del planificador

**Características Principales:**
- Algoritmo Dynamic Window Approach para evitación de obstáculos
- Algoritmo Pure Pursuit para seguimiento de rutas
- Muestreo de trayectorias con restricciones dinámicas
- Evaluación de costos multi-criterio (obstáculos, suavidad, meta, velocidad)
- Detección y manejo de situaciones de bloqueo
- Configuración adaptable entre DWA y Pure Pursuit

**Parámetros:**
```yaml
local_path_planner_node:
  planning_frequency: 20.0                  # Frecuencia de planificación (Hz)
  lookahead_distance: 2.0                   # Distancia de anticipación (m)
  max_linear_velocity: 1.5                  # Velocidad lineal máxima (m/s)
  max_angular_velocity: 1.0                 # Velocidad angular máxima (rad/s)
  prediction_horizon: 2.0                   # Horizonte de predicción (s)
  trajectory_samples: 20                    # Muestras de velocidad lineal
  angular_samples: 11                       # Muestras de velocidad angular
  dynamic_window_approach: true             # Usar DWA
```

### 3. Nodo Optimizador de Trayectorias (`trajectory_optimizer_node`)

Optimiza las rutas planificadas para mejorar suavidad, eficiencia y cumplimiento de restricciones del vehículo.

**Temas Suscritos:**
- `raw_path` (nav_msgs/Path): Ruta sin optimizar (conectada a `global_path`)

**Temas Publicados:**
- `optimized_path` (nav_msgs/Path): Ruta optimizada
- `velocity_profile` (geometry_msgs/TwistStamped): Perfil de velocidad optimizado
- `optimized_trajectory_markers` (visualization_msgs/MarkerArray): Visualización de optimización
- `planning/optimizer_health` (tadeo_ecar_msgs/SystemHealth): Estado de salud del optimizador

**Características Principales:**
- Suavizado de rutas usando descenso de gradiente
- Optimización de curvatura para cumplir restricciones del vehículo
- Generación de perfiles de velocidad optimizados
- Aplicación de límites de aceleración, desaceleración y jerk
- Parametrización temporal de trayectorias
- Visualización detallada de velocidad y curvatura

**Parámetros:**
```yaml
trajectory_optimizer_node:
  optimization_frequency: 5.0               # Frecuencia de optimización (Hz)
  max_velocity: 2.0                         # Velocidad máxima (m/s)
  max_acceleration: 1.0                     # Aceleración máxima (m/s²)
  max_curvature: 2.0                        # Curvatura máxima (1/m)
  enable_velocity_optimization: true        # Optimizar perfil de velocidad
  enable_path_smoothing: true               # Suavizar ruta
  optimization_iterations: 100              # Iteraciones máximas
```

### 4. Nodo Gestor de Rutas (`path_manager_node`)

Coordina todos los componentes del sistema de planificación y gestiona el estado de navegación.

**Temas Suscritos:**
- `global_path` (nav_msgs/Path): Ruta global planificada
- `optimized_path` (nav_msgs/Path): Ruta optimizada
- `local_path` (nav_msgs/Path): Trayectoria local
- `odom` (nav_msgs/Odometry): Odometría del robot
- `move_base_simple/goal` (geometry_msgs/PoseStamped): Meta de navegación

**Temas Publicados:**
- `final_path` (nav_msgs/Path): Ruta final consolidada
- `navigation_status` (std_msgs/String): Estado de navegación
- `path_progress` (geometry_msgs/PoseStamped): Progreso en la ruta
- `planning_visualization` (visualization_msgs/MarkerArray): Visualización del sistema
- `planning/manager_health` (tadeo_ecar_msgs/SystemHealth): Estado de salud del gestor

**Servicios:**
- `navigate_to_goal` (tadeo_ecar_interfaces/NavigateToGoal): Servicio de navegación

**Características Principales:**
- Máquina de estados para gestión de navegación
- Monitoreo de progreso y detección de bloqueos
- Replanificación automática en caso de desviación o fallo
- Priorización inteligente de rutas (optimizada > global > local)
- Detección de llegada a meta con tolerancias configurables
- Sistema integral de monitoreo de salud

**Estados de Navegación:**
- `IDLE`: Esperando meta
- `PLANNING_REQUESTED`: Planificación solicitada
- `GLOBAL_PLANNING_SUCCESS`: Planificación global exitosa
- `NAVIGATING`: Navegando hacia la meta
- `GOAL_REACHED`: Meta alcanzada
- `STUCK`: Robot bloqueado
- `REPLANNING`: Replanificando ruta
- `PLANNING_FAILED`: Planificación fallida

**Parámetros:**
```yaml
path_manager_node:
  management_frequency: 10.0                # Frecuencia de gestión (Hz)
  goal_tolerance: 0.3                       # Tolerancia de meta (m)
  max_planning_attempts: 3                  # Intentos máximos de planificación
  enable_automatic_replanning: true         # Replanificación automática
  stuck_detection_time: 10.0                # Tiempo para detectar bloqueo (s)
  path_deviation_threshold: 2.0             # Umbral de desviación (m)
```

## Ejemplos de Uso

### Lanzamiento Básico del Sistema

```bash
# Lanzar sistema completo de planificación
ros2 launch tadeo_ecar_planning planning_system.launch.py

# Lanzar con componentes específicos
ros2 launch tadeo_ecar_planning planning_system.launch.py \
    enable_global_planner:=true \
    enable_local_planner:=true \
    enable_trajectory_optimizer:=true \
    enable_path_manager:=true

# Lanzar con RViz para visualización
ros2 launch tadeo_ecar_planning planning_system.launch.py launch_rviz:=true

# Lanzar con algoritmo específico
ros2 launch tadeo_ecar_planning planning_system.launch.py planner_type:=RRTstar
```

### Lanzamiento de Nodos Individuales

```bash
# Lanzar solo planificador global
ros2 run tadeo_ecar_planning global_path_planner_node

# Lanzar solo planificador local
ros2 run tadeo_ecar_planning local_path_planner_node

# Lanzar optimizador de trayectorias
ros2 run tadeo_ecar_planning trajectory_optimizer_node

# Lanzar gestor de rutas
ros2 run tadeo_ecar_planning path_manager_node
```

### Operaciones de Navegación

```bash
# Establecer meta de navegación
ros2 topic pub /move_base_simple/goal geometry_msgs/PoseStamped "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: 'map'
pose:
  position: {x: 5.0, y: 3.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"

# Usar servicio de navegación
ros2 service call /navigate_to_goal tadeo_ecar_interfaces/srv/NavigateToGoal "
goal:
  pose:
    position: {x: 5.0, y: 3.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"

# Planificar ruta específica
ros2 service call /plan_global_path tadeo_ecar_interfaces/srv/NavigateToGoal "
goal:
  pose:
    position: {x: 5.0, y: 3.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

### Monitoreo del Sistema

```bash
# Monitorear ruta global
ros2 topic echo /global_path

# Monitorear comandos de velocidad
ros2 topic echo /cmd_vel

# Monitorear estado de navegación
ros2 topic echo /navigation_status

# Monitorear progreso de ruta
ros2 topic echo /path_progress

# Verificar salud del sistema
ros2 topic echo /planning/global_health
ros2 topic echo /planning/local_health
ros2 topic echo /planning/optimizer_health
ros2 topic echo /planning/manager_health
```

### Visualización

```bash
# Ver rutas en RViz
ros2 topic echo /global_path_markers
ros2 topic echo /local_trajectory_markers
ros2 topic echo /optimized_trajectory_markers
ros2 topic echo /planning_visualization

# Ejecutar RViz con configuración personalizada
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix tadeo_ecar_planning)/share/tadeo_ecar_planning/rviz/planning_visualization.rviz
```

## Detalles de Algoritmos

### Planificación Global
- **RRT* (Rapidly-exploring Random Tree)**: Algoritmo probabilísticamente completo con optimización asintótica
- **PRM (Probabilistic Roadmap)**: Pre-cálculo de hoja de ruta para consultas rápidas
- **EST (Expansive Space Trees)**: Exploración uniforme del espacio de estados
- **Verificación de Colisiones**: Considera geometría del robot y márgenes de seguridad
- **Suavizado de Rutas**: Post-procesamiento para mejorar navegabilidad

### Planificación Local
- **Dynamic Window Approach (DWA)**: Muestreo de velocidades factibles con restricciones dinámicas
- **Pure Pursuit**: Seguimiento de ruta con punto de anticipación
- **Evaluación de Trayectorias**: Función de costo multi-objetivo balanceando múltiples criterios
- **Evitación de Obstáculos**: Integración en tiempo real de datos de sensores

### Optimización de Trayectorias
- **Suavizado de Rutas**: Minimización de cambios bruscos de dirección
- **Optimización de Curvatura**: Cumplimiento de restricciones de steering del vehículo
- **Perfiles de Velocidad**: Generación de velocidades respetando límites dinámicos
- **Parametrización Temporal**: Asignación de tiempos para ejecución temporal

### Gestión de Rutas
- **Máquina de Estados**: Control robusto del flujo de navegación
- **Monitoreo de Progreso**: Seguimiento continuo del avance hacia la meta
- **Detección de Bloqueos**: Identificación automática de situaciones problemáticas
- **Replanificación Adaptativa**: Respuesta inteligente a cambios del entorno

## Características de Rendimiento

### Requisitos Computacionales
- **Planificador Global**: ~30% CPU a 2Hz (mapa 2000x2000)
- **Planificador Local**: ~25% CPU a 20Hz con DWA activado
- **Optimizador**: ~20% CPU a 5Hz con 100 iteraciones
- **Gestor de Rutas**: ~15% CPU a 10Hz de frecuencia de gestión

### Uso de Memoria
- **Estructuras de Planificación**: ~10MB para rutas típicas
- **Cache de OMPL**: ~20MB para espacio de estados 2D
- **Historiales de Trayectorias**: ~5MB para datos de seguimiento
- **Visualizaciones**: ~8MB para marcadores y temas de RViz

### Especificaciones de Precisión
- **Precisión de Planificación**: 5-10cm (dependiendo de resolución del mapa)
- **Precisión de Seguimiento**: 10-20cm en condiciones normales
- **Tolerancia de Meta**: Configurable (0.1-0.5m típico)
- **Frecuencia de Actualización**: 2-20Hz según componente

## Configuración

### Parámetros Centrales de Planificación

```yaml
# Configuración del Planificador Global
global_path_planner_node:
  planner_type: "RRTstar"                   # Más preciso pero lento
  max_planning_time: 5.0                    # Mayor tiempo = mejores rutas
  robot_radius: 0.3                         # Ajustar según geometría real
  safety_distance: 0.1                      # Mayor = más conservativo

# Configuración del Planificador Local  
local_path_planner_node:
  planning_frequency: 20.0                  # Mayor = más reactivo, más CPU
  prediction_horizon: 2.0                   # Mayor = más anticipación
  trajectory_samples: 20                    # Mayor = mejor exploración
  dynamic_window_approach: true             # DWA vs Pure Pursuit

# Configuración del Optimizador
trajectory_optimizer_node:
  enable_velocity_optimization: true        # Mejora eficiencia
  optimization_iterations: 100              # Mayor = mejor calidad
  max_curvature: 2.0                        # Límite físico del vehículo

# Configuración del Gestor
path_manager_node:
  enable_automatic_replanning: true         # Replanificación inteligente
  stuck_detection_time: 10.0                # Sensibilidad de bloqueo
  max_planning_attempts: 3                  # Persistencia en fallos
```

### Sintonización Específica por Entorno

**Entornos Interiores:**
- Reducir `max_planning_time` a 2-3s para respuesta rápida
- Aumentar `planning_frequency` a 25-30Hz para obstáculos dinámicos
- Usar `trajectory_samples` más altos (25-30) para espacios estrechos
- Activar `path_smoothing_enabled` para navegación suave

**Entornos Exteriores:**
- Aumentar `lookahead_distance` a 3-5m para velocidades altas
- Reducir `trajectory_samples` a 15-20 para eficiencia
- Configurar `max_velocity` según capacidades del vehículo
- Ajustar `safety_distance` según condiciones del terreno

**Navegación de Precisión:**
- Reducir `goal_tolerance` a 0.1-0.15m
- Aumentar `optimization_iterations` a 150-200
- Activar todas las optimizaciones de trayectoria
- Usar frecuencias altas en todos los componentes

## Solución de Problemas

### Problemas Comunes

1. **Fallo de Planificación Global**:
   ```bash
   # Verificar disponibilidad del mapa
   ros2 topic echo /map --field info
   
   # Verificar pose inicial
   ros2 topic echo /amcl_pose
   
   # Comprobar meta válida
   ros2 topic echo /move_base_simple/goal
   
   # Ajustar parámetros de planificación
   ros2 param set /global_path_planner_node max_planning_time 10.0
   ```

2. **Robot No Se Mueve (Planificador Local)**:
   ```bash
   # Verificar comandos de velocidad
   ros2 topic echo /cmd_vel
   
   # Comprobar detección de obstáculos
   ros2 topic echo /scan
   
   # Verificar ruta global disponible
   ros2 topic echo /global_path
   
   # Ajustar parámetros DWA
   ros2 param set /local_path_planner_node max_linear_velocity 0.5
   ```

3. **Navegación Errática**:
   ```bash
   # Revisar calidad de odometría
   ros2 topic echo /odom
   
   # Verificar estabilidad de localización
   ros2 topic echo /amcl_pose
   
   # Ajustar pesos de costo
   ros2 param set /local_path_planner_node obstacle_cost_weight 500.0
   ```

4. **Alto Uso de CPU**:
   ```bash
   # Reducir frecuencias de planificación
   ros2 param set /global_path_planner_node planning_frequency 1.0
   ros2 param set /local_path_planner_node planning_frequency 15.0
   
   # Reducir muestreo de trayectorias
   ros2 param set /local_path_planner_node trajectory_samples 15
   ros2 param set /local_path_planner_node angular_samples 9
   ```

5. **Bloqueos Frecuentes**:
   ```bash
   # Verificar detección de bloqueos
   ros2 topic echo /navigation_status
   
   # Ajustar sensibilidad
   ros2 param set /path_manager_node stuck_detection_time 15.0
   ros2 param set /path_manager_node stuck_distance_threshold 0.05
   
   # Habilitar replanificación agresiva
   ros2 param set /path_manager_node enable_automatic_replanning true
   ```

### Optimización de Rendimiento

**Para Rendimiento en Tiempo Real:**
- Reducir frecuencias de planificación a 1-5Hz (global) y 10-15Hz (local)
- Usar resoluciones de mapa más bajas (0.1m)
- Limitar iteraciones de optimización a 50-75
- Deshabilitar características costosas como suavizado complejo

**Para Máxima Precisión:**
- Aumentar frecuencias a 5Hz (global) y 25-30Hz (local)
- Usar resoluciones finas (0.02-0.05m)
- Aumentar iteraciones de optimización a 150-200
- Habilitar todas las optimizaciones disponibles

**Para Eficiencia Energética:**
- Optimizar perfiles de velocidad agresivamente
- Usar `max_acceleration` más bajos para suavidad
- Habilitar `enable_velocity_optimization`
- Balancear frecuencias según necesidades de la aplicación

## Integración con Otros Paquetes

### Dependencias
- `tadeo_ecar_msgs`: Mensajes de salud y estado del sistema
- `tadeo_ecar_interfaces`: Definiciones de servicios de navegación
- `tadeo_ecar_config`: Parámetros de configuración del sistema
- `OMPL`: Biblioteca de planificación de movimiento
- `Eigen3`: Operaciones matriciales para optimización

### Flujo de Datos
```
Mapa + Meta → Planificador Global → Optimizador → Gestor de Rutas
                ↓                      ↓             ↓
Sensores → Planificador Local → Comandos Velocidad   ↓
                ↓                      ↓             ↓
              Salud               Salud           Salud → Monitor Seguridad
```

### Integración con Stack de Navegación
- El planificador global proporciona rutas para el stack Nav2
- El gestor de rutas se integra con sistemas de misiones de alto nivel
- Los temas de salud se conectan con sistemas de seguridad
- Las visualizaciones se integran con herramientas de monitoreo

## Paquetes Relacionados

- `tadeo_ecar_perception`: Proporciona datos de sensores para planificación
- `tadeo_ecar_localization`: Proporciona localización precisa para planificación
- `tadeo_ecar_slam`: Proporciona mapas actualizados para planificación
- `tadeo_ecar_control`: Ejecuta los comandos generados por planificación
- `tadeo_ecar_safety`: Monitorea la salud y seguridad de planificación
- `tadeo_ecar_navigation`: Integra planificación con navegación de alto nivel

## Características Avanzadas

### Planificación Multi-Robot
- Soporte de espacios de nombres para sistemas multi-robot
- Coordinación básica evitando conflictos de rutas
- Planificación distributiva para flotas (mejora futura)

### Planificación Adaptativa
- Ajuste dinámico de parámetros según condiciones del entorno
- Detección automática de fallos de sensores y adaptación
- Optimización de rendimiento basada en recursos disponibles

### Gestión Avanzada de Trayectorias
- Versionado automático de rutas y respaldo
- Evaluación de calidad de rutas y validación
- Predicción de trayectorias para planificación proactiva

Este sistema integral de planificación proporciona navegación robusta, precisa y eficiente para operación autónoma en entornos diversos y desafiantes.