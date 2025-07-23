#!/bin/bash
# start.sh - Script para iniciar el entorno ROS2

echo "🚀 Iniciando entorno ROS2 Humble con Gazebo Ignition..."

# Verificar que existe el archivo .env
if [ ! -f .env ]; then
    echo "❌ Error: No se encuentra el archivo .env"
    echo "Por favor copia .env.example a .env y configura las rutas"
    exit 1
fi

# Cargar variables de entorno
source .env

# Verificar que existen las rutas configuradas
if [ ! -d "$TADEO_ECAR_WS_PATH" ]; then
    echo "❌ Error: No se encuentra el workspace en $TADEO_ECAR_WS_PATH"
    echo "Por favor verifica la ruta TADEO_ECAR_WS_PATH en el archivo .env"
    exit 1
fi

# Permitir conexiones X11
echo "🔓 Configurando permisos X11..."
xhost +local:docker

# Verificar si el contenedor ya existe
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "📦 El contenedor $CONTAINER_NAME ya existe."
    if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
        echo "✅ El contenedor ya está ejecutándose."
    else
        echo "▶️  Iniciando contenedor existente..."
        docker-compose start
    fi
else
    echo "🏗️  Construyendo y iniciando contenedor..."
    docker-compose up -d --build
fi

echo ""
echo "✅ Entorno ROS2 iniciado correctamente!"
echo ""
echo "📋 Comandos útiles:"
echo "   Entrar al contenedor:  ./enter.sh"
echo "   Iniciar Gazebo:        ./gazebo.sh"
echo "   Ver logs:              docker-compose logs -f"
echo "   Parar:                 ./stop.sh"
echo ""
echo "🎯 Tu workspace está en: /home/rosuser/tadeo-eCar-ws"