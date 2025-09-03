FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    make \
    cmake \
    build-essential \
    git \
    wget \
    vim \
    pkg-config \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-venv \
    && apt-get clean

CMD ["/bin/bash"]