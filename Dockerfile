
FROM python:3.10-slim

# Proxy build args (passed from docker-compose or --build-arg)
ARG HTTP_PROXY
ARG HTTPS_PROXY
ARG NO_PROXY=localhost,127.0.0.1
ENV http_proxy=$HTTP_PROXY \
    https_proxy=$HTTPS_PROXY \
    HTTP_PROXY=$HTTP_PROXY \
    HTTPS_PROXY=$HTTPS_PROXY \
    no_proxy=$NO_PROXY

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app

# Set working directory
WORKDIR /app

# Configure apt proxy if build was passed one (env vars alone aren't enough for apt)
RUN if [ -n "$HTTP_PROXY" ]; then \
        printf 'Acquire::http::Proxy "%s";\nAcquire::https::Proxy "%s";\n' \
               "$HTTP_PROXY" "${HTTPS_PROXY:-$HTTP_PROXY}" \
        > /etc/apt/apt.conf.d/01proxy; \
            fi

# Install system dependencies for CARLA client and numpy
RUN http_proxy="" https_proxy="" apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    libjpeg-dev \
    libpng-dev \
    && rm -rf /var/lib/apt/lists/* /etc/apt/apt.conf.d/01proxy 2>/dev/null; true


COPY requirements.docker.txt requirements.txt

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY src/ ./src/
COPY start_server.py .

# Recorded runs name the code version: docker compose build --build-arg GIT_SHA=$(git rev-parse --short HEAD)
ARG GIT_SHA=nogit
ENV GIT_SHA=$GIT_SHA

# Create logs directory
RUN mkdir -p logs data runs

# Expose the web server port
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:8000/')" || exit 1

# Run the server
CMD ["python", "start_server.py"]
