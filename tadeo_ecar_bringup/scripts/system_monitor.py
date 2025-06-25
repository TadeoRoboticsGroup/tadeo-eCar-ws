#!/usr/bin/env python3

"""
Script de monitoreo del sistema Tadeo eCar
Proporciona información en tiempo real del estado del sistema
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import BatteryState
from tadeo_ecar_msgs.msg import SystemHealth, RobotStatus
import psutil
import time
import json
from datetime import datetime

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        # Suscriptores
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        self.health_sub = self.create_subscription(
            SystemHealth, '/system_health', self.health_callback, 10)
        self.robot_state_sub = self.create_subscription(
            String, '/robot_state', self.robot_state_callback, 10)
        
        # Variables de estado
        self.battery_level = 0.0
        self.robot_state = "UNKNOWN"
        self.system_health = {}
        self.start_time = time.time()
        
        # Timer para monitoreo
        self.monitor_timer = self.create_timer(1.0, self.monitor_system)
        
        self.get_logger().info("Sistema de monitoreo iniciado")
    
    def battery_callback(self, msg):
        """Callback para estado de batería"""
        if msg.capacity > 0:
            self.battery_level = (msg.charge / msg.capacity) * 100.0
        else:
            self.battery_level = msg.percentage
    
    def health_callback(self, msg):
        """Callback para salud del sistema"""
        self.system_health[msg.component_name] = {
            'status': msg.status,
            'error_code': msg.error_code,
            'error_message': msg.error_message,
            'timestamp': datetime.now().isoformat()
        }
    
    def robot_state_callback(self, msg):
        """Callback para estado del robot"""
        self.robot_state = msg.data
    
    def monitor_system(self):
        """Función principal de monitoreo"""
        # Obtener métricas del sistema
        cpu_percent = psutil.cpu_percent(interval=None)
        memory = psutil.virtual_memory()
        disk = psutil.disk_usage('/')
        
        # Calcular uptime
        uptime = time.time() - self.start_time
        uptime_str = f"{int(uptime // 3600):02d}:{int((uptime % 3600) // 60):02d}:{int(uptime % 60):02d}"
        
        # Mostrar información
        self.print_system_status(cpu_percent, memory, disk, uptime_str)
    
    def print_system_status(self, cpu_percent, memory, disk, uptime):
        """Imprimir estado del sistema en terminal"""
        # Limpiar pantalla (solo en sistemas Unix)
        import os
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("=" * 80)
        print("           MONITOR DEL SISTEMA ECAR 4WD4WS")
        print("             SEMILLERO DE ROBÓTICA")
        print("=" * 80)
        print(f"Fecha/Hora: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Uptime: {uptime}")
        print()
        
        # Estado del Robot
        print("🤖 ESTADO DEL ROBOT")
        print("-" * 40)
        print(f"Estado Actual: {self.robot_state}")
        print(f"Batería: {self.battery_level:.1f}%")
        battery_bar = self.create_progress_bar(self.battery_level, 100)
        print(f"Nivel: {battery_bar}")
        print()
        
        # Recursos del Sistema
        print("💻 RECURSOS DEL SISTEMA")
        print("-" * 40)
        print(f"CPU: {cpu_percent:.1f}%")
        cpu_bar = self.create_progress_bar(cpu_percent, 100)
        print(f"Uso:  {cpu_bar}")
        
        memory_percent = (memory.used / memory.total) * 100
        print(f"RAM: {memory_percent:.1f}% ({memory.used / (1024**3):.1f}GB / {memory.total / (1024**3):.1f}GB)")
        memory_bar = self.create_progress_bar(memory_percent, 100)
        print(f"Uso:  {memory_bar}")
        
        disk_percent = (disk.used / disk.total) * 100
        print(f"Disco: {disk_percent:.1f}% ({disk.used / (1024**3):.1f}GB / {disk.total / (1024**3):.1f}GB)")
        disk_bar = self.create_progress_bar(disk_percent, 100)
        print(f"Uso:   {disk_bar}")
        print()
        
        # Salud de Componentes
        print("🔧 SALUD DE COMPONENTES")
        print("-" * 40)
        if self.system_health:
            for component, health in self.system_health.items():
                status_icon = self.get_status_icon(health['status'])
                print(f"{status_icon} {component}: {health['status']}")
                if health['error_message']:
                    print(f"   Error: {health['error_message']}")
        else:
            print("No hay información de salud disponible")
        print()
        
        # Procesos ROS2
        print("🔄 PROCESOS ROS2")
        print("-" * 40)
        ros_processes = self.get_ros_processes()
        for proc in ros_processes[:10]:  # Mostrar solo los primeros 10
            print(f"PID {proc['pid']}: {proc['name']} (CPU: {proc['cpu']:.1f}%)")
        print()
        
        print("Presiona Ctrl+C para salir...")
        print("=" * 80)
    
    def create_progress_bar(self, value, max_value, length=30):
        """Crear barra de progreso ASCII"""
        percentage = min(value / max_value, 1.0)
        filled_length = int(length * percentage)
        bar = '█' * filled_length + '-' * (length - filled_length)
        
        # Colores basados en el porcentaje
        if percentage > 0.8:
            color = '\033[91m'  # Rojo
        elif percentage > 0.6:
            color = '\033[93m'  # Amarillo
        else:
            color = '\033[92m'  # Verde
        
        reset_color = '\033[0m'
        return f"{color}[{bar}]{reset_color} {percentage*100:.1f}%"
    
    def get_status_icon(self, status):
        """Obtener icono para estado"""
        icons = {
            'OK': '✅',
            'WARNING': '⚠️',
            'ERROR': '❌',
            'EMERGENCY': '🚨',
            'UNKNOWN': '❓'
        }
        return icons.get(status, '❓')
    
    def get_ros_processes(self):
        """Obtener procesos relacionados con ROS2"""
        ros_processes = []
        for proc in psutil.process_iter(['pid', 'name', 'cpu_percent']):
            try:
                if any(keyword in proc.info['name'].lower() for keyword in 
                       ['ros', 'gazebo', 'rviz', 'nav2', 'slam', 'tadeo']):
                    ros_processes.append({
                        'pid': proc.info['pid'],
                        'name': proc.info['name'],
                        'cpu': proc.info['cpu_percent'] or 0.0
                    })
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        
        return sorted(ros_processes, key=lambda x: x['cpu'], reverse=True)
    
    def save_log(self):
        """Guardar log del estado actual"""
        log_data = {
            'timestamp': datetime.now().isoformat(),
            'robot_state': self.robot_state,
            'battery_level': self.battery_level,
            'system_health': self.system_health,
            'system_resources': {
                'cpu_percent': psutil.cpu_percent(),
                'memory_percent': (psutil.virtual_memory().used / psutil.virtual_memory().total) * 100,
                'disk_percent': (psutil.disk_usage('/').used / psutil.disk_usage('/').total) * 100
            }
        }
        
        # Guardar en archivo
        log_file = f"/tmp/tadeo_system_monitor_{datetime.now().strftime('%Y%m%d')}.json"
        try:
            with open(log_file, 'a') as f:
                f.write(json.dumps(log_data) + '\n')
        except Exception as e:
            self.get_logger().error(f"Error guardando log: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = SystemMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nDeteniendo monitor del sistema...")
    except Exception as e:
        print(f"Error en el monitor: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()