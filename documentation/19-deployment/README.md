# Capítulo 19: Deployment y Producción para eCar 4WD4WS

## Tabla de Contenidos

1. [Introducción al Deployment](#introducción-al-deployment)
2. [Estrategias de Deployment](#estrategias-de-deployment)
3. [Containerización con Docker](#containerización-con-docker)
4. [Orquestación con Docker Compose](#orquestación-con-docker-compose)
5. [Deployment en Hardware](#deployment-en-hardware)
6. [Configuración de Producción](#configuración-de-producción)
7. [Monitoreo y Logging](#monitoreo-y-logging)
8. [Updates y Mantenimiento](#updates-y-mantenimiento)
9. [Backup y Recovery](#backup-y-recovery)
10. [Scaling y Load Balancing](#scaling-y-load-balancing)

## Introducción al Deployment

### ¿Qué es Deployment en Robótica?

El deployment en robótica es el proceso de llevar el sistema desde el entorno de desarrollo hasta producción, asegurando operación confiable y mantenible.

```
Deployment Pipeline para eCar 4WD4WS:

Development → Testing → Staging → Production

Componentes del Deployment:
✓ Containerización de aplicaciones
✓ Configuración de entorno de producción
✓ Automatización de deployment
✓ Monitoreo y alertas
✓ Backup y recovery
✓ Updates automáticos
✓ Rollback capabilities
```

### Desafíos del Deployment para eCar

**1. Hardware Dependencies**
- Drivers específicos de sensores
- Calibración de hardware
- Dependencias de sistema operativo

**2. Real-time Requirements**
- Latencias críticas
- Determinismo temporal
- Prioridades de procesos

**3. Safety-Critical System**
- Zero-downtime deployments
- Rollback inmediato
- Validación exhaustiva

**4. Edge Computing**
- Recursos limitados
- Conectividad intermitente
- Autonomía operacional

## Estrategias de Deployment

### Deployment Architecture

```yaml
# deployment_architecture.yml
apiVersion: v1
kind: ConfigMap
metadata:
  name: ecar-deployment-strategy
data:
  strategy: |
    eCar 4WD4WS Deployment Strategy:
    
    1. Blue-Green Deployment:
       - Mantener dos entornos idénticos
       - Switch instantáneo entre versiones
       - Rollback inmediato si necesario
    
    2. Canary Deployment:
       - Deployment gradual a subset de robots
       - Monitoreo de métricas clave
       - Expansion progresiva si exitoso
    
    3. Rolling Update:
       - Update nodo por nodo
       - Verificación de salud en cada paso
       - Mantenimiento de disponibilidad
    
    4. Shadow Deployment:
       - Nueva versión recibe tráfico duplicado
       - Comparación de comportamiento
       - Switch cuando validado
```

### Deployment Manager

```cpp
// src/deployment_manager.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <filesystem>
#include <yaml-cpp/yaml.h>

class DeploymentManager : public rclcpp::Node
{
public:
    DeploymentManager() : Node("deployment_manager")
    {
        // Parámetros de deployment
        this->declare_parameter("deployment_mode", "blue_green");
        this->declare_parameter("health_check_timeout", 30.0);
        this->declare_parameter("rollback_threshold", 5);
        this->declare_parameter("backup_retention_days", 7);
        
        // Publishers
        deployment_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/deployment/status", 10);
        
        deployment_events_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/deployment/events", 10);
        
        // Subscribers
        health_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10,
            std::bind(&DeploymentManager::healthCallback, this, std::placeholders::_1));
        
        // Services para deployment
        deployment_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/deployment/deploy",
            std::bind(&DeploymentManager::deployService, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        rollback_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/deployment/rollback",
            std::bind(&DeploymentManager::rollbackService, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Timer para monitoreo
        monitor_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&DeploymentManager::monitorDeployment, this));
        
        // Estado inicial
        initializeDeploymentState();
        
        RCLCPP_INFO(this->get_logger(), "Deployment Manager initialized");
    }

private:
    enum class DeploymentState {
        IDLE,
        DEPLOYING,
        HEALTH_CHECK,
        ACTIVE,
        ROLLING_BACK,
        FAILED
    };
    
    struct DeploymentInfo {
        std::string version;
        std::string commit_hash;
        std::chrono::system_clock::time_point timestamp;
        DeploymentState state;
        int health_check_failures;
        std::string deployment_path;
    };
    
    void initializeDeploymentState()
    {
        current_deployment_.version = getCurrentVersion();
        current_deployment_.state = DeploymentState::ACTIVE;
        current_deployment_.health_check_failures = 0;
        current_deployment_.timestamp = std::chrono::system_clock::now();
        
        // Cargar información de deployment previo
        loadDeploymentHistory();
    }
    
    std::string getCurrentVersion()
    {
        try {
            // Leer versión desde archivo de versión
            std::ifstream version_file("/opt/ecar/version.txt");
            std::string version;
            std::getline(version_file, version);
            return version.empty() ? "unknown" : version;
        } catch (...) {
            return "unknown";
        }
    }
    
    void loadDeploymentHistory()
    {
        std::string history_file = "/opt/ecar/deployment_history.yaml";
        
        if (std::filesystem::exists(history_file)) {
            try {
                YAML::Node history = YAML::LoadFile(history_file);
                
                for (const auto& deployment : history["deployments"]) {
                    DeploymentInfo info;
                    info.version = deployment["version"].as<std::string>();
                    info.commit_hash = deployment["commit_hash"].as<std::string>();
                    // Parse timestamp, etc.
                    
                    deployment_history_.push_back(info);
                }
                
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), 
                           "Failed to load deployment history: %s", e.what());
            }
        }
    }
    
    void deployService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (current_deployment_.state != DeploymentState::IDLE && 
            current_deployment_.state != DeploymentState::ACTIVE) {
            response->success = false;
            response->message = "Deployment already in progress";
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Starting deployment process");
        
        try {
            // Iniciar proceso de deployment
            startDeployment();
            
            response->success = true;
            response->message = "Deployment started successfully";
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Deployment failed: " + std::string(e.what());
            
            RCLCPP_ERROR(this->get_logger(), "Deployment failed: %s", e.what());
        }
    }
    
    void rollbackService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_WARN(this->get_logger(), "Manual rollback requested");
        
        try {
            performRollback();
            
            response->success = true;
            response->message = "Rollback completed successfully";
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Rollback failed: " + std::string(e.what());
            
            RCLCPP_ERROR(this->get_logger(), "Rollback failed: %s", e.what());
        }
    }
    
    void startDeployment()
    {
        current_deployment_.state = DeploymentState::DEPLOYING;
        current_deployment_.health_check_failures = 0;
        
        publishDeploymentEvent("DEPLOYMENT_STARTED", "Starting new deployment");
        
        std::string deployment_mode = this->get_parameter("deployment_mode").as_string();
        
        if (deployment_mode == "blue_green") {
            performBlueGreenDeployment();
        } else if (deployment_mode == "rolling") {
            performRollingDeployment();
        } else if (deployment_mode == "canary") {
            performCanaryDeployment();
        } else {
            throw std::runtime_error("Unknown deployment mode: " + deployment_mode);
        }
    }
    
    void performBlueGreenDeployment()
    {
        RCLCPP_INFO(this->get_logger(), "Performing Blue-Green deployment");
        
        // 1. Backup current deployment
        createBackup();
        
        // 2. Prepare new environment (Green)
        std::string green_path = prepareGreenEnvironment();
        
        // 3. Deploy new version to Green
        deployToEnvironment(green_path);
        
        // 4. Health check Green environment
        if (healthCheckEnvironment(green_path)) {
            // 5. Switch traffic to Green
            switchToGreenEnvironment(green_path);
            
            current_deployment_.state = DeploymentState::HEALTH_CHECK;
            current_deployment_.deployment_path = green_path;
            
            publishDeploymentEvent("BLUE_GREEN_SWITCHED", "Switched to Green environment");
        } else {
            // Cleanup failed Green environment
            cleanupEnvironment(green_path);
            throw std::runtime_error("Green environment health check failed");
        }
    }
    
    void performRollingDeployment()
    {
        RCLCPP_INFO(this->get_logger(), "Performing Rolling deployment");
        
        std::vector<std::string> nodes = getROS2Nodes();
        
        for (const auto& node : nodes) {
            if (isNodeCritical(node)) {
                RCLCPP_INFO(this->get_logger(), "Updating node: %s", node.c_str());
                
                // Stop node
                stopNode(node);
                
                // Update node
                updateNode(node);
                
                // Start node
                startNode(node);
                
                // Health check
                if (!healthCheckNode(node)) {
                    RCLCPP_ERROR(this->get_logger(), "Node %s failed health check", node.c_str());
                    throw std::runtime_error("Rolling deployment failed at node: " + node);
                }
                
                // Wait before next node
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
        }
        
        current_deployment_.state = DeploymentState::HEALTH_CHECK;
        publishDeploymentEvent("ROLLING_COMPLETED", "Rolling deployment completed");
    }
    
    void performCanaryDeployment()
    {
        RCLCPP_INFO(this->get_logger(), "Performing Canary deployment");
        
        // 1. Deploy to canary subset (10% of functionality)
        deployCanarySubset();
        
        // 2. Monitor canary metrics
        if (monitorCanaryMetrics()) {
            // 3. Gradually increase canary traffic
            for (int percentage : {25, 50, 75, 100}) {
                increaseCanaryTraffic(percentage);
                
                if (!monitorCanaryMetrics()) {
                    RCLCPP_ERROR(this->get_logger(), "Canary failed at %d%%", percentage);
                    rollbackCanary();
                    throw std::runtime_error("Canary deployment failed");
                }
                
                std::this_thread::sleep_for(std::chrono::seconds(30));
            }
            
            current_deployment_.state = DeploymentState::ACTIVE;
            publishDeploymentEvent("CANARY_COMPLETED", "Canary deployment successful");
        } else {
            rollbackCanary();
            throw std::runtime_error("Canary initial health check failed");
        }
    }
    
    void healthCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
    {
        if (current_deployment_.state != DeploymentState::HEALTH_CHECK) {
            return;
        }
        
        // Analizar diagnósticos para determinar salud del deployment
        bool healthy = true;
        
        for (const auto& status : msg->status) {
            if (status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
                healthy = false;
                RCLCPP_WARN(this->get_logger(), 
                           "Health check failed: %s - %s", 
                           status.name.c_str(), status.message.c_str());
            }
        }
        
        if (healthy) {
            current_deployment_.health_check_failures = 0;
        } else {
            current_deployment_.health_check_failures++;
            
            int threshold = this->get_parameter("rollback_threshold").as_int();
            if (current_deployment_.health_check_failures >= threshold) {
                RCLCPP_ERROR(this->get_logger(), 
                           "Health check failures exceeded threshold, rolling back");
                performRollback();
            }
        }
    }
    
    void monitorDeployment()
    {
        // Monitoreo continuo del deployment
        publishDeploymentStatus();
        
        // Cleanup de deployments antiguos
        cleanupOldDeployments();
        
        // Verificar espacio en disco
        checkDiskSpace();
    }
    
    void performRollback()
    {
        RCLCPP_WARN(this->get_logger(), "Performing rollback");
        
        current_deployment_.state = DeploymentState::ROLLING_BACK;
        
        try {
            // Encontrar último deployment exitoso
            auto previous_deployment = findPreviousSuccessfulDeployment();
            
            if (previous_deployment) {
                // Rollback a versión anterior
                rollbackToVersion(previous_deployment->version);
                
                current_deployment_ = *previous_deployment;
                current_deployment_.state = DeploymentState::ACTIVE;
                
                publishDeploymentEvent("ROLLBACK_COMPLETED", 
                                     "Rolled back to version: " + previous_deployment->version);
            } else {
                throw std::runtime_error("No previous successful deployment found");
            }
            
        } catch (const std::exception& e) {
            current_deployment_.state = DeploymentState::FAILED;
            publishDeploymentEvent("ROLLBACK_FAILED", e.what());
            throw;
        }
    }
    
    void createBackup()
    {
        std::string backup_dir = "/opt/ecar/backups/" + 
                               std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
        
        std::filesystem::create_directories(backup_dir);
        
        // Backup configuration
        std::filesystem::copy("/opt/ecar/config", backup_dir + "/config",
                            std::filesystem::copy_options::recursive);
        
        // Backup binaries
        std::filesystem::copy("/opt/ecar/bin", backup_dir + "/bin",
                            std::filesystem::copy_options::recursive);
        
        // Save deployment info
        saveDeploymentInfo(backup_dir);
        
        RCLCPP_INFO(this->get_logger(), "Backup created: %s", backup_dir.c_str());
    }
    
    std::string prepareGreenEnvironment()
    {
        std::string green_path = "/opt/ecar/green";
        
        // Limpiar entorno Green anterior
        if (std::filesystem::exists(green_path)) {
            std::filesystem::remove_all(green_path);
        }
        
        std::filesystem::create_directories(green_path);
        return green_path;
    }
    
    void deployToEnvironment(const std::string& env_path)
    {
        // Copiar nueva versión al entorno
        std::string source_path = "/tmp/ecar_deployment";
        
        if (!std::filesystem::exists(source_path)) {
            throw std::runtime_error("Deployment source not found: " + source_path);
        }
        
        std::filesystem::copy(source_path, env_path,
                            std::filesystem::copy_options::recursive);
        
        // Configurar permisos
        std::filesystem::permissions(env_path, 
                                   std::filesystem::perms::owner_all |
                                   std::filesystem::perms::group_read |
                                   std::filesystem::perms::others_read);
    }
    
    bool healthCheckEnvironment(const std::string& env_path)
    {
        // Verificar que todos los archivos necesarios están presentes
        std::vector<std::string> required_files = {
            env_path + "/bin/motor_controller",
            env_path + "/bin/servo_controller",
            env_path + "/config/main_config.yaml"
        };
        
        for (const auto& file : required_files) {
            if (!std::filesystem::exists(file)) {
                RCLCPP_ERROR(this->get_logger(), "Required file missing: %s", file.c_str());
                return false;
            }
        }
        
        // Test básico de funcionalidad
        return testEnvironmentFunctionality(env_path);
    }
    
    bool testEnvironmentFunctionality(const std::string& env_path)
    {
        // Ejecutar tests básicos en el nuevo entorno
        std::string test_cmd = env_path + "/bin/system_test --quick";
        
        int result = std::system(test_cmd.c_str());
        return result == 0;
    }
    
    void switchToGreenEnvironment(const std::string& green_path)
    {
        // Crear symlink atomic switch
        std::string blue_path = "/opt/ecar/blue";
        std::string current_path = "/opt/ecar/current";
        
        // Backup current as blue
        if (std::filesystem::exists(current_path)) {
            std::filesystem::remove_all(blue_path);
            std::filesystem::rename(current_path, blue_path);
        }
        
        // Switch to green
        std::filesystem::create_symlink(green_path, current_path);
        
        // Restart services with new version
        restartSystemServices();
    }
    
    void restartSystemServices()
    {
        std::vector<std::string> services = {
            "ecar-motor-controller",
            "ecar-servo-controller",
            "ecar-perception",
            "ecar-navigation"
        };
        
        for (const auto& service : services) {
            std::string cmd = "systemctl restart " + service;
            int result = std::system(cmd.c_str());
            
            if (result != 0) {
                RCLCPP_WARN(this->get_logger(), "Failed to restart service: %s", service.c_str());
            }
        }
    }
    
    void publishDeploymentEvent(const std::string& event_type, const std::string& message)
    {
        std_msgs::msg::String event_msg;
        event_msg.data = "[" + event_type + "] " + message;
        deployment_events_pub_->publish(event_msg);
        
        RCLCPP_INFO(this->get_logger(), "Deployment event: %s", event_msg.data.c_str());
    }
    
    void publishDeploymentStatus()
    {
        std_msgs::msg::String status_msg;
        
        std::ostringstream status;
        status << "{"
               << "\"version\":\"" << current_deployment_.version << "\","
               << "\"state\":\"" << deploymentStateToString(current_deployment_.state) << "\","
               << "\"health_failures\":" << current_deployment_.health_check_failures
               << "}";
        
        status_msg.data = status.str();
        deployment_status_pub_->publish(status_msg);
    }
    
    std::string deploymentStateToString(DeploymentState state)
    {
        switch (state) {
            case DeploymentState::IDLE: return "IDLE";
            case DeploymentState::DEPLOYING: return "DEPLOYING";
            case DeploymentState::HEALTH_CHECK: return "HEALTH_CHECK";
            case DeploymentState::ACTIVE: return "ACTIVE";
            case DeploymentState::ROLLING_BACK: return "ROLLING_BACK";
            case DeploymentState::FAILED: return "FAILED";
            default: return "UNKNOWN";
        }
    }
    
    // Miembros
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr deployment_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr deployment_events_pub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_sub_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr deployment_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rollback_service_;
    
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    
    DeploymentInfo current_deployment_;
    std::vector<DeploymentInfo> deployment_history_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeploymentManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Containerización con Docker

### Dockerfile para eCar

```dockerfile
# Dockerfile.ecar
FROM ros:humble-ros-base-jammy

# Instalar dependencias del sistema
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Configurar workspace
WORKDIR /opt/ros/workspace

# Copiar código fuente
COPY src/ src/

# Instalar dependencias de ROS
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Configurar entorno
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/workspace/install/setup.bash" >> ~/.bashrc

# Configurar usuario no-root para seguridad
RUN useradd -m -s /bin/bash ecar && \
    chown -R ecar:ecar /opt/ros/workspace

USER ecar

# Configurar entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Comando por defecto
CMD ["ros2", "launch", "tadeo_ecar_system", "ecar_system.launch.py"]

# Metadata
LABEL version="1.0.0"
LABEL description="eCar 4WD4WS Robot System"
LABEL maintainer="semillero@universidad.edu"

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD ros2 node list | grep -q "motor_controller" || exit 1

# Expose ports for communication
EXPOSE 11311 11312
```

### Multi-stage Dockerfile para Producción

```dockerfile
# Dockerfile.production
# Stage 1: Build environment
FROM ros:humble-ros-base-jammy as builder

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build

# Copy source and build
COPY src/ src/
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/ecar \
        --install

# Stage 2: Runtime environment
FROM ros:humble-ros-base-jammy as runtime

# Instalar solo dependencias de runtime
RUN apt-get update && apt-get install -y \
    python3-pip \
    libopencv-dev \
    libpcl-dev \
    systemd \
    supervisor \
    && rm -rf /var/lib/apt/lists/*

# Copiar archivos compilados desde builder
COPY --from=builder /opt/ecar /opt/ecar

# Configurar variables de entorno
ENV ROS_DOMAIN_ID=42
ENV ROS_LOCALHOST_ONLY=0
ENV ECAR_CONFIG_PATH=/opt/ecar/config

# Copiar configuración
COPY config/ /opt/ecar/config/
COPY docker/supervisord.conf /etc/supervisor/conf.d/ecar.conf

# Configurar logging
RUN mkdir -p /var/log/ecar && \
    chmod 755 /var/log/ecar

# Usuario de runtime
RUN useradd -m -s /bin/bash ecar && \
    usermod -aG dialout ecar && \
    chown -R ecar:ecar /opt/ecar /var/log/ecar

# Scripts de inicio
COPY docker/start.sh /start.sh
RUN chmod +x /start.sh

USER ecar

EXPOSE 11311 11312 8080

HEALTHCHECK --interval=30s --timeout=10s --start-period=120s --retries=3 \
    CMD /opt/ecar/bin/health_check || exit 1

CMD ["/start.sh"]
```

### Docker Entrypoint Script

```bash
#!/bin/bash
# docker/entrypoint.sh

set -e

# Colores para logging
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Función para verificar dependencias
check_dependencies() {
    log_info "Checking dependencies..."
    
    # Verificar ROS2
    if ! command -v ros2 &> /dev/null; then
        log_error "ROS2 not found"
        exit 1
    fi
    
    # Verificar workspace
    if [ ! -f "/opt/ros/workspace/install/setup.bash" ]; then
        log_error "ROS workspace not found"
        exit 1
    fi
    
    log_info "Dependencies check passed"
}

# Función para configurar entorno
setup_environment() {
    log_info "Setting up environment..."
    
    # Source ROS2
    source /opt/ros/humble/setup.bash
    
    # Source workspace
    source /opt/ros/workspace/install/setup.bash
    
    # Configurar ROS_DOMAIN_ID si no está configurado
    if [ -z "$ROS_DOMAIN_ID" ]; then
        export ROS_DOMAIN_ID=42
        log_info "Set ROS_DOMAIN_ID to $ROS_DOMAIN_ID"
    fi
    
    # Configurar logging
    export RCUTILS_LOGGING_BUFFERED_STREAM=1
    export RCUTILS_LOGGING_USE_STDOUT=1
    
    log_info "Environment setup completed"
}

# Función para verificar hardware
check_hardware() {
    log_info "Checking hardware connectivity..."
    
    # Verificar dispositivos serie
    if [ -d "/dev" ]; then
        if ls /dev/ttyUSB* 1> /dev/null 2>&1; then
            log_info "USB serial devices found: $(ls /dev/ttyUSB*)"
        else
            log_warn "No USB serial devices found"
        fi
        
        if ls /dev/ttyACM* 1> /dev/null 2>&1; then
            log_info "ACM serial devices found: $(ls /dev/ttyACM*)"
        else
            log_warn "No ACM serial devices found"
        fi
    fi
    
    # Verificar cámaras
    if ls /dev/video* 1> /dev/null 2>&1; then
        log_info "Video devices found: $(ls /dev/video*)"
    else
        log_warn "No video devices found"
    fi
}

# Función para health check
health_check() {
    log_info "Performing health check..."
    
    # Verificar que ROS2 daemon está corriendo
    if ! ros2 daemon status &> /dev/null; then
        log_info "Starting ROS2 daemon..."
        ros2 daemon start
        sleep 2
    fi
    
    # Verificar conectividad básica
    if ! ros2 node list &> /dev/null; then
        log_warn "ROS2 node list failed, but continuing..."
    fi
    
    log_info "Health check completed"
}

# Función para cleanup en señal
cleanup() {
    log_info "Cleaning up..."
    
    # Enviar SIGTERM a todos los procesos hijos
    if [ ! -z "$CHILD_PID" ]; then
        kill -TERM "$CHILD_PID" 2>/dev/null || true
        wait "$CHILD_PID" 2>/dev/null || true
    fi
    
    # Parar ROS2 daemon
    ros2 daemon stop 2>/dev/null || true
    
    log_info "Cleanup completed"
    exit 0
}

# Configurar signal handlers
trap cleanup SIGTERM SIGINT

# Main execution
main() {
    log_info "Starting eCar 4WD4WS System..."
    log_info "Container arguments: $@"
    
    # Verificaciones iniciales
    check_dependencies
    setup_environment
    check_hardware
    health_check
    
    # Si no se proporcionan argumentos, usar comportamiento por defecto
    if [ $# -eq 0 ]; then
        log_info "No arguments provided, starting default system"
        set -- ros2 launch tadeo_ecar_system ecar_system.launch.py
    fi
    
    # Si el primer argumento es un comando ROS2, configurar entorno completo
    if [ "$1" = "ros2" ]; then
        log_info "Starting ROS2 command: $@"
        
        # Ejecutar comando y capturar PID
        "$@" &
        CHILD_PID=$!
        
        # Esperar a que termine el proceso
        wait $CHILD_PID
        
    else
        log_info "Executing command: $@"
        exec "$@"
    fi
}

# Ejecutar función principal
main "$@"
```

## Orquestación con Docker Compose

### Docker Compose para Desarrollo

```yaml
# docker-compose.dev.yml
version: '3.8'

services:
  ecar-core:
    build:
      context: .
      dockerfile: Dockerfile.ecar
      target: runtime
    container_name: ecar-core
    hostname: ecar-core
    privileged: true  # Necesario para acceso a hardware
    network_mode: host  # Necesario para ROS2 discovery
    environment:
      - ROS_DOMAIN_ID=42
      - ROS_LOCALHOST_ONLY=0
      - ECAR_MODE=development
      - RCUTILS_LOGGING_SEVERITY=DEBUG
    volumes:
      - /dev:/dev  # Acceso a dispositivos hardware
      - ./config:/opt/ecar/config:ro
      - ./logs:/var/log/ecar
      - ecar-data:/opt/ecar/data
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0  # Motor controller
      - /dev/ttyUSB1:/dev/ttyUSB1  # Servo controller
      - /dev/video0:/dev/video0    # Camera
    command: >
      ros2 launch tadeo_ecar_system ecar_system.launch.py
      use_simulation:=false
      debug_mode:=true
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "ros2", "node", "list"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 60s

  ecar-dashboard:
    build:
      context: ./dashboard
      dockerfile: Dockerfile
    container_name: ecar-dashboard
    depends_on:
      - ecar-core
    environment:
      - NODE_ENV=development
      - ECAR_API_URL=http://localhost:8080
    ports:
      - "3000:3000"
    volumes:
      - ./dashboard/src:/app/src
    command: npm run dev
    restart: unless-stopped

  ecar-database:
    image: timescaledb/timescaledb:latest-pg14
    container_name: ecar-database
    environment:
      - POSTGRES_DB=ecar_logs
      - POSTGRES_USER=ecar
      - POSTGRES_PASSWORD=ecar_password_dev
    ports:
      - "5432:5432"
    volumes:
      - ecar-db-data:/var/lib/postgresql/data
      - ./database/init.sql:/docker-entrypoint-initdb.d/init.sql:ro
    restart: unless-stopped

  ecar-monitoring:
    image: prom/prometheus:latest
    container_name: ecar-monitoring
    ports:
      - "9090:9090"
    volumes:
      - ./monitoring/prometheus.yml:/etc/prometheus/prometheus.yml:ro
      - ecar-prometheus-data:/prometheus
    command:
      - '--config.file=/etc/prometheus/prometheus.yml'
      - '--storage.tsdb.path=/prometheus'
      - '--web.console.libraries=/etc/prometheus/console_libraries'
      - '--web.console.templates=/etc/prometheus/consoles'
    restart: unless-stopped

  grafana:
    image: grafana/grafana:latest
    container_name: ecar-grafana
    depends_on:
      - ecar-monitoring
      - ecar-database
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=admin
    ports:
      - "3001:3000"
    volumes:
      - ecar-grafana-data:/var/lib/grafana
      - ./monitoring/grafana-datasources.yml:/etc/grafana/provisioning/datasources/datasources.yml:ro
      - ./monitoring/grafana-dashboards.yml:/etc/grafana/provisioning/dashboards/dashboards.yml:ro
      - ./monitoring/dashboards:/var/lib/grafana/dashboards:ro
    restart: unless-stopped

volumes:
  ecar-data:
    driver: local
  ecar-db-data:
    driver: local
  ecar-prometheus-data:
    driver: local
  ecar-grafana-data:
    driver: local

networks:
  default:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16
```

### Docker Compose para Producción

```yaml
# docker-compose.prod.yml
version: '3.8'

services:
  ecar-system:
    build:
      context: .
      dockerfile: Dockerfile.production
    container_name: ecar-production
    hostname: ecar-robot-${ROBOT_ID:-001}
    privileged: true
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
      - ROS_LOCALHOST_ONLY=0
      - ECAR_MODE=production
      - ROBOT_ID=${ROBOT_ID:-001}
      - RCUTILS_LOGGING_SEVERITY=INFO
      - ECAR_CONFIG_PATH=/opt/ecar/config
    volumes:
      - /dev:/dev
      - ecar-config:/opt/ecar/config
      - ecar-logs:/var/log/ecar
      - ecar-data:/opt/ecar/data
      - /etc/localtime:/etc/localtime:ro
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
      - /dev/ttyUSB1:/dev/ttyUSB1
      - /dev/ttyUSB2:/dev/ttyUSB2
      - /dev/video0:/dev/video0
    restart: always
    stop_grace_period: 30s
    healthcheck:
      test: ["CMD", "/opt/ecar/bin/health_check"]
      interval: 30s
      timeout: 15s
      retries: 3
      start_period: 120s
    logging:
      driver: "json-file"
      options:
        max-size: "100m"
        max-file: "5"
        labels: "service=ecar-system"
    deploy:
      resources:
        limits:
          memory: 4G
          cpus: '2.0'
        reservations:
          memory: 2G
          cpus: '1.0'

  ecar-watchdog:
    image: alpine:latest
    container_name: ecar-watchdog
    depends_on:
      - ecar-system
    environment:
      - CHECK_INTERVAL=60
      - RESTART_THRESHOLD=3
    volumes:
      - /var/run/docker.sock:/var/run/docker.sock:ro
      - ./scripts/watchdog.sh:/watchdog.sh:ro
    command: ["/watchdog.sh", "ecar-production"]
    restart: always

  ecar-logger:
    build:
      context: ./logging
      dockerfile: Dockerfile
    container_name: ecar-logger
    depends_on:
      - ecar-system
    environment:
      - LOG_LEVEL=INFO
      - RETENTION_DAYS=30
    volumes:
      - ecar-logs:/var/log/ecar:ro
      - ecar-archived-logs:/var/log/ecar/archive
    restart: unless-stopped

volumes:
  ecar-config:
    driver: local
    driver_opts:
      type: none
      o: bind
      device: /opt/ecar/config
  ecar-logs:
    driver: local
    driver_opts:
      type: none
      o: bind
      device: /var/log/ecar
  ecar-data:
    driver: local
  ecar-archived-logs:
    driver: local

networks:
  default:
    driver: bridge
```

### Deployment Script

```bash
#!/bin/bash
# scripts/deploy.sh

set -e

# Configuración
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ENVIRONMENT=${1:-production}
ROBOT_ID=${2:-001}

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Verificar prerequisitos
check_prerequisites() {
    log_info "Checking prerequisites..."
    
    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed"
        exit 1
    fi
    
    if ! command -v docker-compose &> /dev/null; then
        log_error "Docker Compose is not installed"
        exit 1
    fi
    
    # Verificar que estamos en el directorio correcto
    if [ ! -f "$PROJECT_DIR/docker-compose.prod.yml" ]; then
        log_error "docker-compose.prod.yml not found"
        exit 1
    fi
    
    log_info "Prerequisites check passed"
}

# Configurar entorno
setup_environment() {
    log_info "Setting up environment for $ENVIRONMENT..."
    
    # Crear archivo .env si no existe
    if [ ! -f "$PROJECT_DIR/.env" ]; then
        cat > "$PROJECT_DIR/.env" << EOF
ROBOT_ID=$ROBOT_ID
ROS_DOMAIN_ID=42
ECAR_VERSION=1.0.0
COMPOSE_PROJECT_NAME=ecar-robot-$ROBOT_ID
EOF
        log_info "Created .env file"
    fi
    
    # Configurar directorios necesarios
    sudo mkdir -p /opt/ecar/{config,data,logs}
    sudo mkdir -p /var/log/ecar
    
    # Configurar permisos
    sudo chown -R $USER:$USER /opt/ecar
    sudo chown -R $USER:$USER /var/log/ecar
    
    log_info "Environment setup completed"
}

# Backup de configuración actual
backup_current_deployment() {
    if docker ps | grep -q ecar-production; then
        log_info "Creating backup of current deployment..."
        
        BACKUP_DIR="/opt/ecar/backups/$(date +%Y%m%d_%H%M%S)"
        mkdir -p "$BACKUP_DIR"
        
        # Backup de configuración
        cp -r /opt/ecar/config "$BACKUP_DIR/"
        
        # Backup de datos importantes
        if [ -d "/opt/ecar/data" ]; then
            cp -r /opt/ecar/data "$BACKUP_DIR/"
        fi
        
        # Información del contenedor actual
        docker inspect ecar-production > "$BACKUP_DIR/container_info.json"
        
        log_info "Backup created at $BACKUP_DIR"
    else
        log_info "No running deployment found, skipping backup"
    fi
}

# Build de imágenes
build_images() {
    log_info "Building Docker images..."
    
    cd "$PROJECT_DIR"
    
    # Build imagen principal
    docker build -f Dockerfile.production -t ecar:$ENVIRONMENT .
    
    # Build imagen de logging si existe
    if [ -d "logging" ]; then
        docker build logging/ -t ecar-logger:$ENVIRONMENT
    fi
    
    log_info "Images built successfully"
}

# Deployment
deploy() {
    log_info "Deploying eCar system..."
    
    cd "$PROJECT_DIR"
    
    # Parar servicios actuales
    if docker ps | grep -q ecar; then
        log_info "Stopping current services..."
        docker-compose -f docker-compose.prod.yml down
    fi
    
    # Limpiar contenedores órfanos
    docker system prune -f
    
    # Deploar nueva versión
    log_info "Starting new deployment..."
    ROBOT_ID=$ROBOT_ID docker-compose -f docker-compose.prod.yml up -d
    
    # Verificar deployment
    sleep 10
    verify_deployment
}

# Verificar deployment
verify_deployment() {
    log_info "Verifying deployment..."
    
    # Verificar que contenedores están corriendo
    if ! docker ps | grep -q ecar-production; then
        log_error "Main container is not running"
        show_logs
        exit 1
    fi
    
    # Health check
    local retries=10
    while [ $retries -gt 0 ]; do
        if docker exec ecar-production /opt/ecar/bin/health_check; then
            log_info "Health check passed"
            break
        else
            log_warn "Health check failed, retrying... ($retries attempts left)"
            retries=$((retries - 1))
            sleep 10
        fi
    done
    
    if [ $retries -eq 0 ]; then
        log_error "Health check failed after all retries"
        show_logs
        exit 1
    fi
    
    # Verificar ROS2 nodes
    if docker exec ecar-production ros2 node list | grep -q motor_controller; then
        log_info "ROS2 nodes are running"
    else
        log_error "ROS2 nodes not found"
        show_logs
        exit 1
    fi
    
    log_info "Deployment verification completed successfully"
}

# Mostrar logs en caso de error
show_logs() {
    log_error "Deployment failed. Showing recent logs:"
    docker-compose -f docker-compose.prod.yml logs --tail=50
}

# Rollback
rollback() {
    log_warn "Rolling back deployment..."
    
    # Encontrar último backup
    LATEST_BACKUP=$(ls -t /opt/ecar/backups/ | head -n1)
    
    if [ -z "$LATEST_BACKUP" ]; then
        log_error "No backup found for rollback"
        exit 1
    fi
    
    log_info "Rolling back to backup: $LATEST_BACKUP"
    
    # Parar servicios actuales
    docker-compose -f docker-compose.prod.yml down
    
    # Restaurar configuración
    cp -r "/opt/ecar/backups/$LATEST_BACKUP/config" /opt/ecar/
    
    # Restaurar datos si existen
    if [ -d "/opt/ecar/backups/$LATEST_BACKUP/data" ]; then
        cp -r "/opt/ecar/backups/$LATEST_BACKUP/data" /opt/ecar/
    fi
    
    # Reiniciar con configuración anterior
    docker-compose -f docker-compose.prod.yml up -d
    
    log_info "Rollback completed"
}

# Cleanup de recursos no utilizados
cleanup() {
    log_info "Cleaning up unused resources..."
    
    # Limpiar imágenes no utilizadas
    docker image prune -f
    
    # Limpiar volúmenes no utilizados
    docker volume prune -f
    
    # Limpiar backups antiguos (más de 7 días)
    find /opt/ecar/backups -type d -mtime +7 -exec rm -rf {} \; 2>/dev/null || true
    
    log_info "Cleanup completed"
}

# Función principal
main() {
    echo "=== eCar 4WD4WS Deployment Script ==="
    echo "Environment: $ENVIRONMENT"
    echo "Robot ID: $ROBOT_ID"
    echo ""
    
    case "${1:-deploy}" in
        "deploy")
            check_prerequisites
            setup_environment
            backup_current_deployment
            build_images
            deploy
            cleanup
            log_info "Deployment completed successfully!"
            ;;
        "rollback")
            rollback
            ;;
        "verify")
            verify_deployment
            ;;
        "logs")
            docker-compose -f docker-compose.prod.yml logs -f
            ;;
        "stop")
            docker-compose -f docker-compose.prod.yml down
            ;;
        "status")
            docker-compose -f docker-compose.prod.yml ps
            ;;
        *)
            echo "Usage: $0 {deploy|rollback|verify|logs|stop|status} [robot_id]"
            exit 1
            ;;
    esac
}

# Ejecutar función principal
main "$@"
```

Este capítulo cubre todos los aspectos fundamentales del deployment para el eCar 4WD4WS, desde containerización hasta orquestación y automatización de deployment en producción.