# Dockerfile for GICI-LIB compilation on Ubuntu 22.04
FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    pkg-config \
    gdb \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Install Eigen3
RUN apt-get update && apt-get install -y libeigen3-dev

# Install OpenCV dependencies and OpenCV
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    libopencv-contrib-dev \
    python3-opencv

# Install yaml-cpp
RUN apt-get update && apt-get install -y libyaml-cpp-dev

# Install gflags and glog from packages (more reliable)
RUN apt-get update && apt-get install -y \
    libgflags-dev \
    libgoogle-glog-dev

# Install Ceres Solver from packages
RUN apt-get update && apt-get install -y \
    libceres-dev \
    libatlas-base-dev \
    libsuitesparse-dev

# Create a working directory
WORKDIR /workspace

# Set environment variables
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Default command
CMD ["/bin/bash"]
