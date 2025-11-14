# DriveBrain Software 2026

This guide explains how to build the DriveBrain Software 2026 using the provided Docker cross-compilation environment.

## Prerequisites

* Docker installed on your system
* Local clone of the DriveBrain repository
* DriveBrain Conan packages available at `[insert_your_directory]/drivebrain_packages/.conan2`
* Be sure to have replaced [insert_your_directory] with something that exists on your system

## Building the Software

## 0. Build the dockerfile (you only need to do this once)
Run the following command to build the dockerfile: 
```
docker build -t rpi-cross-compile .
```
It may take a few minutes. 

### 1. Enter the Docker Container

Run the following command to start the Docker container with the necessary volumes mounted:

```bash
docker run -it \
  -v [insert_your_directory]/drivebrain_packages/.conan2:/root/.conan2 \
  -v "$(pwd)":/app \
  -w /app \
  rpi-cross-compile /bin/bash
```

On some systems you may want to add the following flag `--memory=16g --cpus=4`.

* `-v [insert_your_directory]/drivebrain_packages/.conan2:/root/.conan2` mounts your local Conan cache inside the container.
* `-v "$(pwd)":/app` mounts the current repository folder inside the container.
* `-w /app` sets the working directory inside the container.

### 2. Run the Build Script

Inside the container, execute:

```bash
./build_script.sh
```

This script will build the DriveBrain software using the cross-compilation toolchain for the target platform.
The first time you run it, it will take some time because it needs to compile all the dependencies. However, future builds past the first one take <1 minute because dependencies are cached. 

### Notes
* Make sure the build script has execute permissions: `chmod +x build_script.sh`.
* The build artifacts will be generated inside the mounted repository folder, so they are accessible on your host system.
