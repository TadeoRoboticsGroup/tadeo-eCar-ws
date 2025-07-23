#!/bin/bash
# build.sh - Script para reconstruir el contenedor

source .env

echo "🔨 Reconstruyendo imagen Docker..."
docker-compose down
docker-compose build --no-cache
docker-compose up -d

echo "✅ Imagen reconstruida correctamente!" 