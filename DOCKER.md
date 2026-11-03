# Docker Deployment Guide

This guide explains how to run the CARLA V2V Research Platform frontend using Docker.

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
docker build -t carla-v2v-frontend .

# Run the container
docker run -d \
  --name carla-v2v-frontend \
  -p 8000:8000 \
  -v $(pwd)/logs:/app/logs \
  -v $(pwd)/data:/app/data \
  carla-v2v-frontend

# View logs
docker logs -f carla-v2v-frontend

# Stop and remove
docker stop carla-v2v-frontend && docker rm carla-v2v-frontend
```

## Accessing the Frontend

Once running, access the web interface at:

| Page | URL | Description |
|------|-----|-------------|
| **Home** | http://localhost:8000/ | Links to the pages below |
| **Admin panel** | http://localhost:8000/admin | Connect to CARLA, traffic, V2X (admin token required) |
| **Client join** | http://localhost:8000/client | Join as a vehicle |

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
| `CARLA_HOST` | `192.168.1.101` | Default CARLA server IP |
| `CARLA_PORT` | `2000` | Default CARLA server port |

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
