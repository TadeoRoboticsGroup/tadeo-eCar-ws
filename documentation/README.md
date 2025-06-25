# Documentación ROS2 - Robot Autónomo eCar 4WD4WS

## Índice de Contenidos

Esta documentación te llevará desde los conceptos básicos de ROS2 hasta el desarrollo de un robot autónomo completo usando como caso de estudio el sistema eCar 4WD4WS del Semillero de Robótica.

### Nivel Principiante

**[01. Introducción a ROS2](01-introduccion/README.md)**
- Qué es ROS2 y por qué usarlo
- Historia y evolución desde ROS1
- Instalación y configuración del entorno
- Primer ejemplo con el eCar

**[02. Conceptos Básicos](02-conceptos-basicos/README.md)**
- Arquitectura distribuida
- Workspace y paquetes
- Compilación con colcon
- Variables de entorno

**[03. Nodos](03-nodos/README.md)**
- Concepto de nodo en ROS2
- Creación de nodos en C++ y Python
- Lifecycle nodes
- Ejemplos con sensores del eCar

**[04. Tópicos](04-topicos/README.md)**
- Comunicación publisher/subscriber
- Tipos de mensajes
- QoS (Quality of Service)
- Implementación con datos del eCar

### Nivel Intermedio

**[05. Servicios](05-servicios/README.md)**
- Comunicación síncrona cliente/servidor
- Definición de servicios personalizados
- Manejo de errores y timeouts
- Servicios del sistema eCar

**[06. Acciones](06-acciones/README.md)**
- Comunicación asíncrona para tareas largas
- Goals, feedback y results
- Cancelación de acciones
- Navegación del eCar con acciones

**[07. Parámetros](07-parametros/README.md)**
- Sistema de parámetros dinámicos
- Archivos de configuración YAML
- Parámetros por nodo
- Configuración del eCar

**[08. Launch Files](08-launch/README.md)**
- Sistema de lanzamiento
- Argumentos y condiciones
- Composición y namespace
- Launch completo del eCar

### Nivel Intermedio-Avanzado

**[09. TF2 y Transformaciones](09-tf2/README.md)**
- Sistema de coordenadas
- Transformaciones estáticas y dinámicas
- tf2_tools y debugging
- Frames del eCar 4WD4WS

**[10. Navegación Autónoma](10-navigation/README.md)**
- Stack Nav2
- Costmaps y planificadores
- Behavior trees de navegación
- Implementación en el eCar

**[11. SLAM](11-slam/README.md)**
- Simultaneous Localization and Mapping
- SLAM Toolbox vs Cartographer
- Generación y gestión de mapas
- SLAM con el eCar

**[12. Behavior Trees](12-behavior-trees/README.md)**
- Árboles de comportamiento
- BehaviorTree.CPP
- Nodos personalizados
- Sistema de comportamientos del eCar

### Nivel Avanzado

**[13. Control](13-control/README.md)**
- ros2_control framework
- Controladores hardware
- Control 4WD4WS
- Implementación del eCar

**[14. Percepción](14-perception/README.md)**
- Procesamiento de sensores
- Computer vision con OpenCV
- Point cloud processing
- Fusión sensorial del eCar

**[15. Safety](15-safety/README.md)**
- Sistemas de seguridad
- Monitoring y watchdog
- Emergency stop
- Safety multicapa del eCar

**[16. Integración de Sistemas](16-integracion/README.md)**
- Arquitectura de sistemas complejos
- Lifecycle management
- Error handling
- Sistema completo del eCar

### Nivel Experto

**[17. Debugging y Diagnósticos](17-debug/README.md)**
- Herramientas de debugging
- rqt y visualización
- Log analysis
- Profiling y optimización

**[18. Testing](18-testing/README.md)**
- Unit testing en ROS2
- Integration testing
- Simulación para testing
- CI/CD pipelines

**[19. Deployment](19-deployment/README.md)**
- Deployment en producción
- Docker y contenedores
- System services
- Monitoring en producción

**[20. Temas Avanzados](20-avanzado/README.md)**
- DDS y comunicación
- Security en ROS2
- Multi-robot systems
- Real-time ROS2

## Organización de la Documentación

Cada capítulo incluye:

- **README.md**: Teoría y conceptos
- **examples/**: Ejemplos de código
- **exercises/**: Ejercicios prácticos
- **reference/**: Material de referencia

## Requisitos Previos

- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- Conocimientos básicos de programación (C++ o Python)
- Terminal/línea de comandos

## Cómo Usar Esta Documentación

1. **Secuencial**: Sigue los capítulos en orden para un aprendizaje completo
2. **Por temas**: Accede directamente a capítulos específicos según tus necesidades
3. **Práctica**: Cada capítulo incluye ejemplos ejecutables del proyecto eCar
4. **Referencia**: Usa como guía de consulta durante el desarrollo

## Contribuciones

Esta documentación es parte del proyecto eCar del Semillero de Robótica.

Para reportar errores o sugerir mejoras:
- **Issues**: https://github.com/TadeoRoboticsGroup/tadeo-eCar-ws/issues
- **Email**: ing.marioalvarezvallejo@gmail.com

## Licencia

Copyright (c) 2024 Semillero de Robótica. Todos los derechos reservados.

---

**Nota**: Esta documentación está diseñada para ser práctica y aplicable. Todos los ejemplos están basados en el sistema real del robot autónomo eCar 4WD4WS.