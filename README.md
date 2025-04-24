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

## Notes

* **Container Names:** The default container names are `zed-ros-noetic` for ROS1 and `zed_ros2_container` for ROS2. These are defined in the respective `docker-compose.yml` files.
* **Volumes:** The compose files mount necessary `/dev` nodes and directories for device access and X11 forwarding. Additional volumes for ZED SDK settings or resources might be needed depending on your setup (refer to the Stereolabs Docker documentation).
* **JetPack/L4T Version:** The ROS2 container is configured for L4T r36.4.0 (JetPack 6.0 DP). If you are using a different version, you might need to adjust the `L4T_VERSION` build argument in `ros2_humble/docker-compose.yml` and potentially the base image in the Dockerfile.
