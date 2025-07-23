# Imagen base de ROS2 Humble
FROM ros:humble

# Evitar interacciones durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# Instalar dependencias básicas primero
RUN apt-get update && apt-get install -y \
    curl wget git build-essential cmake nano \
    lsb-release gnupg2 software-properties-common \
    python3-pip python3-dev \
    usbutils libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# Agregar el repositorio oficial de Gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Eliminar cualquier configuración previa de ROS-Industrial
RUN rm -f /etc/apt/sources.list.d/ros-industrial.list || true

# Instalar paquetes ROS2 core y herramientas básicas
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    python3-colcon-common-extensions \
    python3-vcstool \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Instalar MoveIt y controladores
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-servo \
    ros-humble-joint-trajectory-controller \
    ros-humble-effort-controllers \
    ros-humble-position-controllers \
    && rm -rf /var/lib/apt/lists/*

# Instalar controladores específicos para 4WD4WS
RUN apt-get update && apt-get install -y \
    ros-humble-ackermann-msgs \
    ros-humble-four-wheel-steering-msgs \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Instalar percepción 3D y SLAM
RUN apt-get update && apt-get install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-perception-pcl \
    ros-humble-rtabmap-ros \
    ros-humble-octomap \
    ros-humble-octomap-ros \
    ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Instalar soporte para cámaras RealSense (opcional)
RUN apt-get update && apt-get install -y \
    ros-humble-librealsense2 \
    ros-humble-realsense2-camera \
    && rm -rf /var/lib/apt/lists/* || \
    echo "RealSense packages not available, skipping..."

# Instalar navegación
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-smac-planner \
    ros-humble-nav2-behavior-tree \
    ros-humble-behaviortree-cpp-v3 \
    && rm -rf /var/lib/apt/lists/*

# Instalar interfaces con robots específicos
RUN apt-get update && apt-get install -y \
    ros-humble-ur-description \
    ros-humble-ur-msgs \
    && rm -rf /var/lib/apt/lists/*

# Paquetes opcionales que pueden no estar disponibles
RUN apt-get update && apt-get install -y \
    ros-humble-franka-description \
    && rm -rf /var/lib/apt/lists/* || \
    echo "Franka packages not available, skipping..."

RUN apt-get update && apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    && rm -rf /var/lib/apt/lists/* || \
    echo "MAVROS packages not available, skipping..."

# FKIE multimaster (puede no estar disponible)
RUN apt-get update && apt-get install -y \
    ros-humble-fkie-multimaster \
    && rm -rf /var/lib/apt/lists/* || \
    echo "FKIE multimaster not available, skipping..."

# Instalar comunicación serial y hardware (MEJORADO)
RUN apt-get update && apt-get install -y \
    ros-humble-serial-driver \
    ros-humble-v4l2-camera \
    ros-humble-usb-cam \
    ros-humble-image-tools \
    libusb-1.0-0-dev \
    libftdi1-dev \
    && rm -rf /var/lib/apt/lists/*

# Micro XRCE-DDS Agent (nombre corregido)
RUN apt-get update && apt-get install -y \
    ros-humble-micro-xrce-dds-agent \
    && rm -rf /var/lib/apt/lists/* || \
    echo "Micro XRCE-DDS Agent not available, skipping..."

# ROSSerial (puede no estar disponible en ROS2 Humble)
RUN apt-get update && apt-get install -y \
    ros-humble-rosserial \
    && rm -rf /var/lib/apt/lists/* || \
    echo "ROSSerial packages not available for ROS2 Humble, skipping..."

# Instalar visión y procesamiento de imágenes
RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*

# Instalar Gazebo Ignition Fortress (v6) y dependencias para renderizado
RUN apt-get update && apt-get install -y \
    ignition-fortress \
    ros-humble-ros-ign-bridge \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-image \
    ros-humble-ros-ign-interfaces \
    && rm -rf /var/lib/apt/lists/*

# INSTALAR MESA Y DRIVERS DE RENDERIZADO COMPLETOS (CORREGIDO)
RUN apt-get update && apt-get install -y \
    mesa-utils \
    mesa-common-dev \
    libegl1-mesa \
    libegl1-mesa-dev \
    libgl1-mesa-glx \
    libgl1-mesa-dev \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libgles2-mesa \
    libgles2-mesa-dev \
    mesa-va-drivers \
    mesa-vdpau-drivers \
    mesa-vulkan-drivers \
    libglapi-mesa \
    libgbm1 \
    libegl-mesa0 \
    libxrandr2 \
    libxss1 \
    libxcursor1 \
    libxcomposite1 \
    libasound2 \
    libxi6 \
    libxtst6 \
    && rm -rf /var/lib/apt/lists/*

# Instalar puente ROS1-ROS2 y dependencias adicionales
RUN apt-get update && apt-get install -y \
    ros-humble-rosbridge-server \
    ros-humble-rosbridge-msgs \
    ros-humble-rosbridge-library \
    python-is-python3 \
    libqt5x11extras5 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    qtbase5-dev \
    libqt5gui5 \
    libegl1 \
    libglib2.0-0 \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Intentar instalar el puente ROS1-ROS2 si está disponible
RUN apt-get update && \
    (apt-get install -y ros-humble-ros1-bridge || \
    echo "ros1-bridge not available, using rosbridge as alternative") && \
    rm -rf /var/lib/apt/lists/*

# Crear usuario no-root para mayor seguridad (MEJORADO)
RUN useradd -m -s /bin/bash -G dialout,video,audio,plugdev,users rosuser && \
    echo "rosuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Instalar dependencias Python básicas
RUN pip3 install --no-cache-dir \
    transforms3d \
    numpy \
    matplotlib \
    PyQt5 \
    pyyaml \
    rospkg \
    empy

# Instalar dependencias Python para visión y ML
RUN pip3 install --no-cache-dir \
    opencv-python \
    scikit-learn \
    scikit-image \
    pillow

# Instalar frameworks de deep learning (comentado por tamaño)
# Descomenta si necesitas ML/DL
# RUN pip3 install --no-cache-dir \
#     tensorflow \
#     keras \
#     torch \
#     torchvision

# Instalar paquetes para RL y comunicación
RUN pip3 install --no-cache-dir \
    pyserial \
    gym \
    stable-baselines3 \
    pydot \
    graphviz \
    xarray \
    optuna

# Paquetes específicos de ROS2 (pueden no estar disponibles)
RUN pip3 install --no-cache-dir \
    plansys2-tools \
    nav2-simple-commander || \
    echo "Some ROS2 Python packages not available, skipping..."

# Configurar acceso a puertos USB y cámaras (MEJORADO)
RUN mkdir -p /etc/udev/rules.d && \
    echo 'KERNEL=="ttyUSB[0-9]*",MODE="0666"' >> /etc/udev/rules.d/99-serial.rules && \
    echo 'KERNEL=="ttyACM[0-9]*",MODE="0666"' >> /etc/udev/rules.d/99-serial.rules && \
    echo 'KERNEL=="video[0-9]*",MODE="0666"' >> /etc/udev/rules.d/99-camera.rules && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="*", MODE="0666"' >> /etc/udev/rules.d/99-usb.rules

# Configurar variables de entorno para Gazebo Ignition Fortress y renderizado (CORREGIDO)
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe
ENV MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
ENV MESA_GL_VERSION_OVERRIDE=3.3
ENV MESA_GLSL_VERSION_OVERRIDE=330
ENV NO_AT_BRIDGE=1
ENV QT_ACCESSIBILITY=1
ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins
ENV IGN_GUI_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/ign-gui-6/plugins

# Crear y configurar el espacio de trabajo ROS2
WORKDIR /home/rosuser
RUN mkdir -p /home/rosuser/ros2_ws/src && \
    chown -R rosuser:rosuser /home/rosuser/ros2_ws

# Cambiar a usuario rosuser
USER rosuser
WORKDIR /home/rosuser

# Descargar paquetes adicionales (comentado pero disponible)
# WORKDIR /home/rosuser/ros2_ws/src
# RUN git clone -b humble https://github.com/ros-planning/moveit2_tutorials.git
# RUN git clone -b ros2 https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git
# RUN git clone https://github.com/ros-planning/moveit_task_constructor.git
# RUN git clone https://github.com/PlanSys2/ros2_planning_system.git
# RUN git clone -b humble https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples.git

# Volver a root temporalmente para instalar code-server
USER root
RUN curl -fsSL https://code-server.dev/install.sh | sh

# Configurar el entorno ROS2 y Gazebo para el usuario (ACTUALIZADO PARA TADEO-ECAR-WS)
USER rosuser
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "# Source workspace principal (tadeo-eCar-ws)" >> ~/.bashrc && \
    echo "if [ -f ~/tadeo-eCar-ws/install/setup.bash ]; then source ~/tadeo-eCar-ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo "# Source workspace adicional (ros2_ws)" >> ~/.bashrc && \
    echo "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc && \
    echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
    echo "export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins" >> ~/.bashrc && \
    echo "export IGN_GUI_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/ign-gui-6/plugins" >> ~/.bashrc && \
    echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc && \
    echo "export GALLIUM_DRIVER=llvmpipe" >> ~/.bashrc && \
    echo "export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe" >> ~/.bashrc && \
    echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> ~/.bashrc && \
    echo "# Ir al workspace principal por defecto" >> ~/.bashrc && \
    echo "cd ~/tadeo-eCar-ws" >> ~/.bashrc

# Scripts como root
USER root

# Script para iniciar Gazebo Ignition Fortress (ACTUALIZADO CON VARIABLES CORREGIDAS)
RUN echo '#!/bin/bash' > /start_gazebo.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /start_gazebo.sh && \
    echo 'if [ -f "/home/rosuser/tadeo-eCar-ws/install/setup.bash" ]; then' >> /start_gazebo.sh && \
    echo '  source /home/rosuser/tadeo-eCar-ws/install/setup.bash' >> /start_gazebo.sh && \
    echo 'fi' >> /start_gazebo.sh && \
    echo 'if [ -f "/home/rosuser/ros2_ws/install/setup.bash" ]; then' >> /start_gazebo.sh && \
    echo '  source /home/rosuser/ros2_ws/install/setup.bash' >> /start_gazebo.sh && \
    echo 'fi' >> /start_gazebo.sh && \
    echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> /start_gazebo.sh && \
    echo 'export GALLIUM_DRIVER=llvmpipe' >> /start_gazebo.sh && \
    echo 'export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe' >> /start_gazebo.sh && \
    echo 'export MESA_GL_VERSION_OVERRIDE=3.3' >> /start_gazebo.sh && \
    echo 'ign gazebo -v 4' >> /start_gazebo.sh && \
    chmod +x /start_gazebo.sh

# Script para Gazebo headless (SIN RENDERIZADO)
RUN echo '#!/bin/bash' > /start_gazebo_server_only.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /start_gazebo_server_only.sh && \
    echo 'if [ -f "/home/rosuser/tadeo-eCar-ws/install/setup.bash" ]; then' >> /start_gazebo_server_only.sh && \
    echo '  source /home/rosuser/tadeo-eCar-ws/install/setup.bash' >> /start_gazebo_server_only.sh && \
    echo 'fi' >> /start_gazebo_server_only.sh && \
    echo 'if [ -f "/home/rosuser/ros2_ws/install/setup.bash" ]; then' >> /start_gazebo_server_only.sh && \
    echo '  source /home/rosuser/ros2_ws/install/setup.bash' >> /start_gazebo_server_only.sh && \
    echo 'fi' >> /start_gazebo_server_only.sh && \
    echo 'ign gazebo -r -s' >> /start_gazebo_server_only.sh && \
    chmod +x /start_gazebo_server_only.sh

# Script para el puente ROS-Gazebo Ignition (ACTUALIZADO)
RUN echo '#!/bin/bash' > /start_ros_ign_bridge.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /start_ros_ign_bridge.sh && \
    echo 'if [ -f "/home/rosuser/tadeo-eCar-ws/install/setup.bash" ]; then' >> /start_ros_ign_bridge.sh && \
    echo '  source /home/rosuser/tadeo-eCar-ws/install/setup.bash' >> /start_ros_ign_bridge.sh && \
    echo 'fi' >> /start_ros_ign_bridge.sh && \
    echo 'if [ -f "/home/rosuser/ros2_ws/install/setup.bash" ]; then' >> /start_ros_ign_bridge.sh && \
    echo '  source /home/rosuser/ros2_ws/install/setup.bash' >> /start_ros_ign_bridge.sh && \
    echo 'fi' >> /start_ros_ign_bridge.sh && \
    echo 'ros2 run ros_ign_bridge parameter_bridge' >> /start_ros_ign_bridge.sh && \
    chmod +x /start_ros_ign_bridge.sh

# Resto de scripts igual que antes...
RUN echo '#!/bin/bash' > /start_bridge.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /start_bridge.sh && \
    echo 'if [ -f "/home/rosuser/tadeo-eCar-ws/install/setup.bash" ]; then' >> /start_bridge.sh && \
    echo '  source /home/rosuser/tadeo-eCar-ws/install/setup.bash' >> /start_bridge.sh && \
    echo 'fi' >> /start_bridge.sh && \
    echo 'if [ -f "/home/rosuser/ros2_ws/install/setup.bash" ]; then' >> /start_bridge.sh && \
    echo '  source /home/rosuser/ros2_ws/install/setup.bash' >> /start_bridge.sh && \
    echo 'fi' >> /start_bridge.sh && \
    echo 'ros2 run ros1_bridge dynamic_bridge || echo "ros1_bridge no disponible."' >> /start_bridge.sh && \
    chmod +x /start_bridge.sh

RUN echo '#!/bin/bash' > /monitor_serial.sh && \
    echo 'echo "Puertos seriales disponibles:"' >> /monitor_serial.sh && \
    echo 'ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No hay dispositivos conectados"' >> /monitor_serial.sh && \
    chmod +x /monitor_serial.sh

# Script de punto de entrada (ACTUALIZADO PARA TADEO-ECAR-WS)
RUN echo '#!/bin/bash' > /ros_entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /ros_entrypoint.sh && \
    echo '# Source workspace principal (tadeo-eCar-ws)' >> /ros_entrypoint.sh && \
    echo 'if [ -f "/home/rosuser/tadeo-eCar-ws/install/setup.bash" ]; then' >> /ros_entrypoint.sh && \
    echo '  source /home/rosuser/tadeo-eCar-ws/install/setup.bash' >> /ros_entrypoint.sh && \
    echo 'fi' >> /ros_entrypoint.sh && \
    echo '# Source workspace adicional (ros2_ws)' >> /ros_entrypoint.sh && \
    echo 'if [ -f "/home/rosuser/ros2_ws/install/setup.bash" ]; then' >> /ros_entrypoint.sh && \
    echo '  source /home/rosuser/ros2_ws/install/setup.bash' >> /ros_entrypoint.sh && \
    echo 'fi' >> /ros_entrypoint.sh && \
    echo 'cd /home/rosuser/tadeo-eCar-ws' >> /ros_entrypoint.sh && \
    echo 'exec "$@"' >> /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Volver al usuario rosuser para el contenedor
USER rosuser
WORKDIR /home/rosuser/tadeo-eCar-ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]