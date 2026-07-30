# Build environment for the CDCL Foxglove converter extension: Node for the
# extension build, Python for the converter generator. Used by ./build.sh.
FROM node:20-bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    python3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
