# Docker Deployment Guide

This guide explains how to run the CARLA V2X Research Platform frontend using Docker.

## Quick Start

### Using Docker Compose (Recommended)

```bash
# Build and start the container
docker-compose up -d

# View logs
docker-compose logs -f

# Stop the container
docker-compose down
```

### Using Docker directly

```bash
# Build the image
docker build -t carla-v2x-frontend .

# Run the container
docker run -d \
  --name carla-v2x-frontend \
  -p 8000:8000 \
  -v $(pwd)/logs:/app/logs \
  -v $(pwd)/data:/app/data \
  carla-v2x-frontend

# View logs
docker logs -f carla-v2x-frontend

# Stop and remove
docker stop carla-v2x-frontend && docker rm carla-v2x-frontend
```

## Accessing the Frontend

Once running, access the web interface at:

| Page | URL | Description |
|------|-----|-------------|
| **Home** | http://localhost:8000/ | Links to the pages below |
| **Admin panel** | http://localhost:8000/admin | Connect to CARLA, traffic, V2X (admin token required) |
| **Observer** | http://localhost:8000/observer | Read-only live map and V2X view |
| **Vehicle** | http://localhost:8000/vehicle | Join as a vehicle (`/client` redirects here) |

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Docker Container                              │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │              FastAPI Web Server (Port 8000)                 │ │
│  │  ┌─────────────┐  ┌──────────────┐  ┌─────────────────────┐ │ │
│  │  │ Admin panel │  │ Client join  │  │   REST / WebSocket  │ │ │
│  │  └─────────────┘  └──────────────┘  └─────────────────────┘ │ │
│  └─────────────────────────────────────────────────────────────┘ │
└───────────────────────────┬─────────────────────────────────────┘
                            │ Network (TCP)
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                 CARLA Server (External Machine)                  │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │              CarlaUE4.exe (Port 2000)                       │ │
│  │  - Windows/Linux host with GPU                              │ │
│  │  - Running CARLA 0.9.16                                     │ │
│  └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `CARLA_HOST` | (none) | CARLA server IP; set it in `.env` or the shell, there is no built-in default |
| `CARLA_PORT` | `2000` | CARLA server RPC port |

### Docker Compose Configuration

Edit `docker-compose.yml` to customize:

```yaml
services:
  frontend:
    environment:
      - CARLA_HOST=your.carla.server.ip
      - CARLA_PORT=2000
    ports:
      - "8080:8000"  # Change host port if needed
```
