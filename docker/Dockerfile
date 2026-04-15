FROM python:3.10-slim

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# Runtime libraries needed by Open3D.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    libglib2.0-0 \
    libgl1 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install package with LAS/LAZ conversion extras.
COPY pyproject.toml README.md requirements.txt ./
COPY lanelet2_generator ./lanelet2_generator
RUN pip install --upgrade pip setuptools wheel && \
    pip install ".[las]"

# Default to showing converter help.
CMD ["python", "-m", "lanelet2_generator.las_mgrs_cli", "--help"]
