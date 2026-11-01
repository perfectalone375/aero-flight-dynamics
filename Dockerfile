# Multi-stage Dockerfile for aero-flight-dynamics
FROM python:3.12-slim AS base

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# Copy packaging configuration and install dependencies
COPY pyproject.toml README.md LICENSE /app/
COPY src/ /app/src/
COPY tests/ /app/tests/

RUN pip install --upgrade pip && \
    pip install ".[dev]"

# Target for testing in isolated container environment
FROM base AS test
CMD ["pytest", "-v", "--cov=aero_flight_dynamics", "tests/"]

# Target for running simulation CLI
FROM base AS runtime
ENTRYPOINT ["aeroflight"]
CMD ["--help"]
