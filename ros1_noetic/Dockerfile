# 1. Define ARGs
ARG L4T_VERSION=l4t-r36.4.0
ARG CUDA_MAJOR=12 # Check specific CUDA version for JP6 GA if needed, usually 12.2
ARG CUDA_MINOR=2
ARG ZED_SDK_MAJOR=5
ARG ZED_SDK_MINOR=0
ARG ZED_SDK_PATCH=0
ARG L4T_MAJOR=36  # Used in ZED URL
ARG L4T_MINOR=4   # Used in ZED URL

# Use the corresponding base L4T image
ARG IMAGE_NAME=nvcr.io/nvidia/l4t-jetpack:r36.4.0
ARG ROS1_DISTRO=noetic # Keep for reference

FROM ${IMAGE_NAME}

# Set ARGs as ENV
ARG L4T_VERSION
ARG CUDA_MAJOR
ARG CUDA_MINOR
ARG ZED_SDK_MAJOR
ARG ZED_SDK_MINOR
ARG ZED_SDK_PATCH
ARG L4T_MAJOR
ARG L4T_MINOR
ARG ROS1_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

# Configure environment
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility
ENV LOGNAME=root
ENV ROS_DISTRO=${ROS1_DISTRO}
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV TZ=Europe/Paris

# 2. Install Base Dependencies (Combine existing and ROS build requirements)
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils dialog && \
    rm -rf /var/lib/apt/lists/*

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install apt packages (python3-catkin-pkg and python3-rospkg removed)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    lsb-release wget curl less udev sudo zstd \
    build-essential cmake git \
    python3 python3-dev python3-pip python3-setuptools \
    python3-empy python3-yaml python3-netifaces \
    libboost-all-dev \
    libusb-1.0-0-dev libsuitesparse-dev software-properties-common \
    libv4l-dev libglew-dev libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev \
    libglu1-mesa-dev libjpeg-dev libpng-dev libavcodec-dev libavformat-dev \
    libavutil-dev libswscale-dev libopenblas-dev liblapack-dev libpq-dev \
    libgeographic-dev libconsole-bridge-dev libpoco-dev libtinyxml2-dev \
    liblz4-dev libbz2-dev libtheora-dev sbcl libjpeg-turbo8-dev libgpgme-dev \
    python3-pil libxcb-cursor0 \
    libyaml-cpp-dev \
    libturbojpeg0-dev \
    liburdfdom-headers-dev \
    liburdfdom-dev \
    libgl1-mesa-glx libegl1 mesa-utils \
    libxinerama1 libxrandr2 libxfixes3 libxcursor1 libxcomposite1 libxi6 libxtst6 \
    libxcb-glx0 libxcb-keysyms1 libxcb-render-util0 libxcb-icccm4 libxcb-image0 \
    libxcb-render0 libxcb-shape0 libxcb-shm0 libxcb-sync1 libxcb-xfixes0 libxcb-xinerama0 libxcb-xkb1 \
    libxkbcommon-x11-0 \
    && rm -rf /var/lib/apt/lists/*


# Upgrade pip and install necessary ROS python tools and other dependencies via pip
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install -U \
    catkin-pkg \
    rospkg \
    catkin-tools \
    rosdep \
    wstool \
    rosinstall_generator \
    numpy \
    opencv-python-headless \
    pyyaml \
    protobuf \
    cython \
    coverage \
    nose \
    mock \
    paramiko \
    psutil \
    defusedxml \
    pycryptodomex \
    python-gnupg \
    Pillow


# 3. Install ZED SDK (Use confirmed URL and version)
ENV ZED_SDK_URL="https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/ZED_SDK_Tegra_L4T${L4T_MAJOR}.${L4T_MINOR}_v${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}.zstd.run"

# Check URL validity (optional but good practice, borrowed from ROS2 file)
RUN if [ "$(curl -I "${ZED_SDK_URL}" -o /dev/null -s -w '%{http_code}\n' | head -n 1)" = "200" ]; then echo "ZED SDK URL is valid."; else echo "ZED SDK URL is invalid or unreachable."; exit 1; fi

RUN apt-get update -y || true && \
    apt-get install --no-install-recommends -y && \
    wget -q -O ZED_SDK_Linux_JP.run ${ZED_SDK_URL} && \
    chmod +x ZED_SDK_Linux_JP.run && \
    # Run silently, skip tools, skip dependencies (we installed them)
    ./ZED_SDK_Linux_JP.run silent skip_depends && \
    rm -rf /usr/local/zed/resources/* && \
    rm ZED_SDK_Linux_JP.run && \
    # Ensure libusb link exists if needed (was in original file)
    ln -sf /lib/aarch64-linux-gnu/libusb-1.0.so.0 /usr/lib/aarch64-linux-gnu/libusb-1.0.so && \
    rm -rf /var/lib/apt/lists/*

# 4. Prepare ROS Workspace and Clone Sources
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Clone ROS Noetic Base sources (using specific tags from the guide)
RUN git clone https://github.com/ros/actionlib.git -b 1.14.0 && \
    git clone https://github.com/ros/bond_core.git -b 1.8.6 && \
    git clone https://github.com/ros/catkin.git -b 0.8.10 && \
    git clone https://github.com/ros/class_loader.git -b 0.5.0 && \
    git clone https://github.com/ros/cmake_modules.git -b 0.5.0 && \
    git clone https://github.com/ros/common_msgs.git -b 1.13.1 && \
    git clone https://github.com/ros/dynamic_reconfigure.git -b 1.7.3 && \
    git clone https://github.com/ros/gencpp.git -b 0.7.0 && \
    git clone https://github.com/jsk-ros-pkg/geneus.git -b 3.0.0 && \
    git clone https://github.com/ros/genlisp.git -b 0.4.18 && \
    git clone https://github.com/ros/genmsg.git -b 0.6.0 && \
    git clone https://github.com/RethinkRobotics-opensource/gennodejs.git -b 2.0.1 && \
    git clone https://github.com/ros/genpy.git -b 0.6.16 && \
    git clone https://github.com/ros/message_generation.git -b 0.4.1 && \
    git clone https://github.com/ros/message_runtime.git -b 0.4.13 && \
    git clone https://github.com/ros/nodelet_core.git -b 1.10.2 && \
    git clone https://github.com/ros/pluginlib.git -b 1.13.0 && \
    git clone https://github.com/ros/ros.git -b 1.15.8 && \
    git clone https://github.com/ros/ros_comm.git -b 1.16.0 && \
    git clone https://github.com/ros/ros_comm_msgs.git -b 1.11.3 && \
    git clone https://github.com/ros/ros_environment.git -b 1.3.2 && \
    git clone https://github.com/ros/rosbag_migration_rule.git -b 1.0.1 && \
    git clone https://github.com/ros/rosconsole.git -b 1.14.3 && \
    git clone https://github.com/ros/rosconsole_bridge.git -b 0.5.4 && \
    git clone https://github.com/ros/roscpp_core.git -b 0.7.2 && \
    git clone https://github.com/ros/roslisp.git -b 1.9.25 && \
    git clone https://github.com/ros/rospack.git -b 2.6.2 && \
    git clone https://github.com/ros/std_msgs.git -b 0.5.13 && \
    git clone https://github.com/ros/geometry2.git -b ${ROS_DISTRO}-devel && \
    git clone https://github.com/ros/diagnostics.git -b ${ROS_DISTRO}-devel && \
    git clone https://github.com/ros/xacro.git -b ${ROS_DISTRO}-devel && \
    git clone https://github.com/ros/urdf.git -b ${ROS_DISTRO}-devel && \
    git clone https://github.com/ros/kdl_parser.git -b ${ROS_DISTRO}-devel && \
    git clone https://github.com/ros/robot_state_publisher.git -b ${ROS_DISTRO}-devel && \
    git clone https://github.com/ros/roslint.git -b 0.12.0

    #vedere     git clone https://github.com/ros/roslint.git -b master



# Clone vision_opencv and image_transport_plugins (Noetic branches)
RUN git clone --branch ${ROS_DISTRO}-devel https://github.com/ros-perception/image_common.git && \
    git clone --branch ${ROS_DISTRO}-devel https://github.com/ros-perception/image_transport_plugins.git && \
    git clone --branch ${ROS_DISTRO} https://github.com/ros-perception/vision_opencv.git

# Clone ZED ROS1 Wrapper (recursively to get zed-ros-interfaces)
RUN git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git

# 5. Initialize rosdep
# rosdep command itself installed via pip
RUN rosdep init || echo "Rosdep already initialized." && \
    rosdep update

# 6. Install OS dependencies using rosdep
WORKDIR /root/catkin_ws

RUN rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:jammy || true


RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libbullet-dev \
    libeigen3-dev \
    liborocos-kdl-dev


# --- Stage 1: Build ROS Base + vision_opencv (excluding plugins and zed wrapper) ---
RUN mkdir -p /tmp/skipped_pkgs && \
    mv src/image_transport_plugins /tmp/skipped_pkgs/ && \
    mv src/zed-ros-wrapper /tmp/skipped_pkgs/

# Use the catkin_make executable from the cloned catkin source
# Add explicit CMAKE_INSTALL_PREFIX
RUN ./src/catkin/bin/catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/root/catkin_ws/install \
      -DCMAKE_BUILD_TYPE=Release \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      -DCATKIN_ENABLE_TESTING=OFF

# --- !! Verification Step BETWEEN Stages !! ---
# Check if image_transport AND tf2_ros installed their files correctly in Stage 1
RUN echo "VERIFICATION Stage 1: Listing image_transport cmake files..." && \
    (ls -l /root/catkin_ws/install/share/image_transport/cmake/*.cmake || exit 1) && \
    echo "VERIFICATION Stage 1: Listing tf2_ros cmake files..." && \
    (ls -l /root/catkin_ws/install/share/tf2_ros/cmake/*.cmake || exit 1) && \
    echo "VERIFICATION SUCCESS: image_transport and tf2_ros CMake files found AFTER Stage 1." \
    || (echo "VERIFICATION FAILED: CMake files for image_transport or tf2_ros NOT FOUND AFTER Stage 1!" && exit 1)


# --- Stage 2: Build the remaining packages ---
# Source the environment from Stage 1, move skipped packages back, clean build/devel dirs
# Add echo to check CMAKE_PREFIX_PATH after sourcing
RUN /bin/bash -c ". /root/catkin_ws/install/setup.bash && \
    echo Sourced Stage 1 setup. CMAKE_PREFIX_PATH is now: $CMAKE_PREFIX_PATH && \
    mv /tmp/skipped_pkgs/* src/ && \
    rm -rf /root/catkin_ws/build /root/catkin_ws/devel"

# Build and install again, now including the previously skipped packages
# Add explicit CMAKE_INSTALL_PREFIX
RUN /bin/bash -c ". /root/catkin_ws/install/setup.bash && \
    echo Check within Stage 2 subshell. CMAKE_PREFIX_PATH is: $CMAKE_PREFIX_PATH && \
    ./src/catkin/bin/catkin_make install \
    -DCMAKE_INSTALL_PREFIX=/root/catkin_ws/install \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DCATKIN_ENABLE_TESTING=OFF"

# Add sourcing to .bashrc
RUN echo 'source /root/catkin_ws/install/setup.bash' >> /root/.bashrc

# --- Final Cleanup ---
RUN rm -rf /root/catkin_ws/build /root/catkin_ws/devel /tmp/skipped_pkgs

# 7. Setup Entrypoint
COPY ros_entrypoint.sh /sbin/ros_entrypoint.sh
RUN chmod 755 /sbin/ros_entrypoint.sh
# Ensure ros_entrypoint.sh sources /root/catkin_ws/install/setup.bash

ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]