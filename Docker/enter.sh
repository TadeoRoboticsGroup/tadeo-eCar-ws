#!/bin/bash
# enter.sh - Script para entrar al contenedor

source .env

if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "🐳 Entrando al contenedor $CONTAINER_NAME..."
    docker-compose exec ros2_humble bash
else
    echo "❌ El contenedor $CONTAINER_NAME no está ejecutándose."
    echo "Ejecuta ./start.sh primero"
fi