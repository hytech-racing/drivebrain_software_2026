# DriveBrain Software 2026

This guide explains how to build the DriveBrain Software 2026 using the provided Docker cross-compilation environment.

## Prerequisites

* Docker installed on your system
* Local clone of the DriveBrain repository
* DriveBrain Conan packages available at `[insert_your_directory]/drivebrain_packages/.conan2`
* Be sure to have replaced [insert_your_directory] with something that exists on your system

## Building the Software

### 1. Enter the Docker Container

Run the following command to start the Docker container with the necessary volumes mounted:

```bash
docker run -it \
  -v [insert_your_directory]/drivebrain_packages/.conan2:/root/.conan2 \
  -v "$(pwd)":/app \
  -w /app \
  rpi-cross-compile /bin/bash
```

* `-v [insert_your_directory]/drivebrain_packages/.conan2:/root/.conan2` mounts your local Conan cache inside the container.
* `-v "$(pwd)":/app` mounts the current repository folder inside the container.
* `-w /app` sets the working directory inside the container.

### 2. Run the Build Script

Inside the container, execute:

```bash
./build_script.sh
```

This script will build the DriveBrain software using the cross-compilation toolchain for the target platform.

### Notes
* Make sure the build script has execute permissions: `chmod +x build_script.sh`.
* The build artifacts will be generated inside the mounted repository folder, so they are accessible on your host system.
