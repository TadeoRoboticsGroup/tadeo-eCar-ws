#!/bin/bash
# stop.sh - Script para parar el entorno

source .env

echo "🛑 Parando entorno ROS2..."
docker-compose down

echo "🔒 Removiendo permisos X11..."
xhost -local:docker

echo "✅ Entorno ROS2 detenido correctamente."