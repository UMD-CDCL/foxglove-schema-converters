FROM node:20-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    bash \
    ca-certificates \
    python3 \
    python3-pip \
    ripgrep \
    jq \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces/foxglove-schema-converters
