# ROS1 & ROS2 Docker Containers for ZED Camera on Jetson Orin

This repository provides Docker containers for running ROS1 (Noetic) and ROS2 (Humble) with Stereolabs ZED camera support on NVIDIA Jetson Orin platforms.

It's specifically configured for use with the ZED X Mini camera.

## Prerequisites

* NVIDIA Jetson Orin device
* NVIDIA JetPack SDK (Tested with JetPack 6.0 DP / L4T r36.4.0 for ROS2)
* [Docker](https://docs.docker.com/engine/install/) installed on the Jetson
* [Docker Compose](https://docs.docker.com/compose/install/) installed on the Jetson
* [NVIDIA Container Runtime](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) configured for Docker
* Stereolabs ZED SDK v5.0.0 (This is installed within the ROS2 container during build)
* ZED X Mini camera connected

## Setup Host Machine (X11 Forwarding)

Before running the containers, you need to allow the containers to access your host's display server. Open a terminal on the Jetson host and run:

```bash
export DISPLAY=:0
# Allow local root user to access the X server
# Note: The exact path to Xauthority might differ based on your display manager (GDM, LightDM, etc.)
# try `ls -l ~/.Xauthority` first or `ls -l /run/user/$(id -u)/gdm/Xauthority`to obtain XAUTH_PATH.
# for kyon is sudo env XAUTHORITY=/run/user/128/gdm/Xauthority xhost +si:localuser:root
sudo env XAUTHORITY=$XAUTH_PATH xhost +si:localuser:root
```

## Build and Run Containers

Navigate to the specific ROS version directory (`ros1_noetic` or `ros2_humble`) containing the `docker-compose.yml` file you want to use.

### ROS2 Humble Specifics
 
Open a terminal *on your host machine* and run:

```bash
# Navigate to the wrapper source directory
cd /path/to/your/repo/ros2_humble

# Clone the repository into zed_components
git clone https://github.com/stereolabs/zed-ros2-wrapper.git ./zed_components

# Now proceed with the build step below
```

### Build and Run Steps

1.  **Build the Docker image:**
    *(Run this from within the `ros1_noetic` or `ros2_humble` directory)*
    ```bash
    docker compose build
    ```
2.  **Run the Docker container in detached mode:**
    ```bash
    docker compose up -d
    ```

## Accessing the Containers

To open a terminal inside the running containers:

* **ROS1 (Noetic):**
    ```bash
    docker exec -it zed-ros-noetic bash
    ```
* **ROS2 (Humble):**
    ```bash
    docker exec -it zed_ros2_container bash
    ```

## Running the ZED Camera Wrapper

Inside the container's terminal:

**Launch the ZED Wrapper:**

 * **ROS1 (Noetic):**
   ```bash
   # Ensure the correct workspace is sourced (done by entrypoint.sh)
   # source /root/catkin_ws/install/setup.bash # Usually done by entrypoint
   roslaunch zed_wrapper zedxm.launch
    ```

 * **ROS2 (Humble):**
   ```bash
   # Ensure the correct workspace is sourced (done by entrypoint.sh)
   # source /opt/ros/humble/install/setup.bash # Done by entrypoint
   # source /root/ros2_ws/install/local_setup.bash # Done by entrypoint
   ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm
   ```

You should now see the ZED node starting up and publishing topics. You can use tools like `rostopic list` (ROS1) or `ros2 topic list` (ROS2) to see the available camera topics.
## Cross-Compilation

Cross-compilation allows you to build the `linux/arm64` Docker images for your Jetson Orin on a host machine with a different architecture (e.g., an x86_64 PC). This can be faster and more convenient than building directly on the Jetson device. This setup uses QEMU for emulation and Docker Buildx to manage multi-architecture builds.

**Note on Kernel Versions:** While these Docker images are tailored for the L4T kernel provided with JetPack (e.g., L4T r36.4.0 for JetPack 6.0 uses a Linux 5.15-based kernel), it has been noted that for broader Ubuntu compatibility or specific custom setups, Linux kernel version 6.8.49 might be a relevant consideration. Always ensure your build environment aligns with your target Jetson's L4T kernel.

### Setting up the Host for Cross-Compilation

These steps are typically run once on your build machine to enable it to build `linux/arm64` images:

1.  **Update package lists:**
    ```bash
    sudo apt-get update
    ```
    Ensures your package manager has the latest list of available software.

2.  **Install QEMU user static binaries:**
    ```bash
    sudo apt-get install qemu-user-static
    ```
    QEMU is an emulator and virtualizer. `qemu-user-static` allows your non-ARM host to execute ARM binaries, which is essential for building ARM Docker images.

3.  **Register QEMU interpreters with the kernel:**
    ```bash
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
    ```
    This command uses a Docker image from `multiarch` to register the QEMU interpreters with your host system's `binfmt_misc` handler. This allows the kernel to automatically use QEMU when it encounters an executable for a different architecture (like ARM64).
    * `--rm`: Removes the container once the command finishes.
    * `--privileged`: Gives the container extended privileges, necessary to modify the host's `binfmt_misc` settings.
    * `multiarch/qemu-user-static`: The Docker image containing the QEMU static binaries and registration script.
    * `--reset -p yes`: Resets any existing QEMU binfmt_misc registrations and registers them persistently (`-p yes`).

### Building with Docker Buildx

Docker Buildx is a CLI plugin that extends Docker with new features, including the ability to build for multiple architectures.

1.  **Create and select a Buildx builder instance (if not already done):**
    A builder instance is where your builds will run.
    ```bash
    docker buildx create --bootstrap --name mybuilder --driver docker-container --use
    ```
    * `docker buildx create`: Command to create a new builder instance.
    * `--bootstrap`: Starts the builder instance immediately.
    * `--name mybuilder`: Assigns a name (`mybuilder`) to this builder instance. You can choose any name.
    * `--driver docker-container`: Specifies that this builder will use a Docker container as its build environment. This is necessary for multi-platform builds.
    * `--use`: Switches the current Docker context to use this newly created builder instance by default for `docker buildx build` commands.

2.  **Perform the cross-compilation build:**
    Navigate to the directory containing the Dockerfile you want to build (e.g., `ros1_noetic` or `ros2_humble`).
    ```bash
    docker buildx build \
      --builder mybuilder \
      --platform linux/arm64 \
      --tag local/zed-ros-noetic-orin:latest \
      --load \
      .
    ```
    * `docker buildx build`: The command to start a build using Buildx.
    * `--builder mybuilder`: Specifies which Buildx builder instance to use (the one we named `mybuilder`).
    * `--platform linux/arm64`: This crucial flag tells Buildx to build an image for the `linux/arm64` architecture, which is the target for your Jetson Orin.
    * `--tag local/zed-ros-noetic-orin:latest`: Tags the built image with a name and tag (e.g., `local/zed-ros-noetic-orin` with the `latest` tag). Adjust this for your specific image (e.g., for ROS2 Humble).
    * `--load`: This flag instructs Buildx to load the built image directly into your local Docker image store. Without it, the image might only exist within the builder's cache, especially for multi-platform builds. This makes the image immediately available for `docker images` and for pushing to a registry.
    * `.`: Specifies the build context (the current directory), where your Dockerfile is located.

### Troubleshooting Buildx Issues

If you encounter persistent problems with your Buildx builder instance (`mybuilder` in these examples), you might need to remove it and recreate it. This can help resolve issues related to a corrupted builder state or cache.

1.  **Remove the existing builder instance:**
    ```bash
    docker buildx rm mybuilder
    ```
    * `docker buildx rm`: Command to remove a builder instance.
    * `mybuilder`: The name of the builder instance you want to remove.

2.  **Recreate the builder instance:**
    After removing it, you can recreate it using the `docker buildx create` command shown previously:
    ```bash
    docker buildx create --bootstrap --name mybuilder --driver docker-container --use
    ```
    Once recreated, you can attempt your `docker buildx build` command again.

---
## Notes

* **Container Names:** The default container names are `zed-ros-noetic` for ROS1 and `zed_ros2_container` for ROS2. These are defined in the respective `docker-compose.yml` files.
* **Volumes:** The compose files mount necessary `/dev` nodes and directories for device access and X11 forwarding. Additional volumes for ZED SDK settings or resources might be needed depending on your setup (refer to the Stereolabs Docker documentation).
* **JetPack/L4T Version:** The ROS2 container is configured for L4T r36.4.0 (JetPack 6.0 DP). If you are using a different version, you might need to adjust the `L4T_VERSION` build argument in `ros2_humble/docker-compose.yml` and potentially the base image in the Dockerfile.
