FROM python:3.10-slim

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    MCAP_CLI_VERSION=0.0.62

# Runtime libraries needed by Open3D.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    libglib2.0-0 \
    libgl1 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# MCAP CLI for indexed reads on large recordings (no ROS required).
RUN curl -fsSL \
    "https://github.com/foxglove/mcap/releases/download/releases/mcap-cli/v${MCAP_CLI_VERSION}/mcap-linux-amd64" \
    -o /usr/local/bin/mcap && \
    chmod +x /usr/local/bin/mcap

WORKDIR /app

# Install package with LAS/LAZ conversion extras.
COPY pyproject.toml README.md requirements.txt ./
COPY lanelet2_generator ./lanelet2_generator
RUN pip install --upgrade pip setuptools wheel && \
    pip install ".[las,mcap]"

# Default to showing converter help.
CMD ["python", "-m", "lanelet2_generator.las_mgrs_cli", "--help"]
