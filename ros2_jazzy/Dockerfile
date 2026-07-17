# ROS 2 Jazzy + ZED SDK for Jetson Orin (JetPack 6 host, Ubuntu 24.04 userspace).
#
# Jazzy only exists on Ubuntu 24.04, while JetPack 6 (L4T r36.4) ships Ubuntu 22.04.
# Orin cannot move to JetPack 7 (that is Thor-only), so the only way to Jazzy is an
# Ubuntu 24.04 container on the unchanged JetPack 6 host. The base image below is
# exactly that: Noble userspace + CUDA 12.8 built against the L4T r36.4 driver stack.
# The ZED SDK Jetson installer is built for L4T r36.4 (Jammy) -- Stereolabs does not
# support it on a Noble userspace. See README.md before debugging build failures.
#
# Tag layout changed upstream: humble was `humble-desktop-l4t-r36.4.0`,
# jazzy is `jazzy-desktop-r36.4.0-cu128-24.04` (no `l4t-` prefix, CUDA + Ubuntu suffix).
ARG L4T_VERSION=r36.4.0
ARG CUDA_TAG=cu128
ARG UBUNTU_RELEASE=24.04
ARG IMAGE_NAME=dustynv/ros:jazzy-desktop-${L4T_VERSION}-${CUDA_TAG}-${UBUNTU_RELEASE}

FROM ${IMAGE_NAME}

# zed-ros2-wrapper master requires ZED SDK >= 5.2.
ARG ZED_SDK_MAJOR=5
ARG ZED_SDK_MINOR=4
ARG ZED_SDK_PATCH=0
ARG L4T_MAJOR=36
ARG L4T_MINOR=4
ARG L4T_PATCH=0

# ROS2 distribution
ARG ROS2_DIST=jazzy

ARG ZED_WRAPPER_VERSION=master
ARG ZED_MSGS_VERSION=5.3.0

# ZED ROS2 Wrapper dependencies, pinned to the versions released into Jazzy
# (source of truth: https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml)
ARG XACRO_VERSION=2.1.1
ARG DIAGNOSTICS_VERSION=4.2.7
ARG AMENT_LINT_VERSION=0.17.5
ARG ROBOT_LOCALIZATION_VERSION=3.8.3
ARG NMEA_MSGS_VERSION=2.1.0
ARG ANGLES_VERSION=1.16.1
ARG GEOGRAPHIC_INFO_VERSION=1.0.6
ARG POINTCLOUD_TRANSPORT_VERSION=4.0.9
ARG POINTCLOUD_TRANSPORT_PLUGINS_VERSION=4.0.4
ARG COB_VERSION=2.8.12
ARG RMW_CYCLONEDDS_VERSION=2.2.3
# vision_opencv and ffmpeg_image_transport_msgs have no `jazzy` branch (they go
# humble -> iron -> rolling), so these are pinned by release tag instead of branch.
ARG VISION_OPENCV_VERSION=4.1.0
ARG IMAGE_TRANSPORT_PLUGINS_VERSION=4.0.7
ARG FFMPEG_IMAGE_TRANSPORT_MSGS_VERSION=1.3.0

ENV DEBIAN_FRONTEND=noninteractive

# Official Stereolabs URL: resolves to the current .run for this SDK/L4T pair and
# survives CDN changes, unlike a hardcoded digitaloceanspaces path. Needs -L (redirect).
ENV ZED_SDK_URL="https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons"

RUN set -eux; \
    # Remove any stale ROS repo/key from the base image
    rm -f /etc/apt/sources.list.d/ros*.list /etc/apt/sources.list.d/ros2*.list /etc/apt/sources.list.d/ros-latest.list || true; \
    rm -f /etc/apt/trusted.gpg.d/ros* /etc/apt/trusted.gpg.d/ros2* || true; \
    # Install tooling to add the new key
    apt-get update; \
    apt-get install -y --no-install-recommends ca-certificates curl gnupg; \
    # Install the current ROS archive key and repo (resolves to noble on this base image)
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list; \
    apt-get update

# Check that this SDK exists
RUN if [ "$(curl -L -I "${ZED_SDK_URL}" -o /dev/null -s -w '%{http_code}\n' | head -n 1)" = "200" ]; then \
        echo "The URL points to something."; \
    else \
        echo "The URL does not point to a .run file or the file does not exist."; \
        exit 1; \
    fi


ENV TZ=Europe/Paris

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
  apt-get update && \
  apt-get install --yes lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libpq-dev zstd usbutils && \
  rm -rf /var/lib/apt/lists/*

# --- Install Build Dependencies (Excluding conflicting OpenCV dev packages) ---
# Install all theora and video codec dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  python3-numpy \
  python3-dev \
  libboost-python-dev \
  # Theora video codec dependencies
  libtheora-dev \
  libogg-dev \
  libvorbis-dev \
  # Additional multimedia libraries that theora might need
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  # Standard image libraries
  libzstd-dev \
  libpng-dev \
  libjpeg-dev \
  libturbojpeg0-dev \
  && rm -rf /var/lib/apt/lists/*


  # libgeographiclib-dev is the Noble name; on Jammy the same package is libgeographic-dev.
  RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}.${L4T_PATCH}" > /etc/nv_tegra_release && \
  apt-get update -y && \
  apt-get install -y --no-install-recommends \
    zstd wget less cmake curl gnupg2 build-essential \
    python3 python3-pip python3-dev python3-setuptools \
    libusb-1.0-0-dev libgeographiclib-dev libdraco-dev zlib1g-dev \
    python3-protobuf && \
  wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run "${ZED_SDK_URL}" && \
  chmod +x ZED_SDK_Linux_JP.run && \
  ./ZED_SDK_Linux_JP.run silent skip_tools && \
  rm -rf /usr/local/zed/resources/* ZED_SDK_Linux_JP.run && \
  rm -rf /var/lib/apt/lists/*

# Install the ZED ROS2 Wrapper
ENV ROS_DISTRO=${ROS2_DIST}

# Clone ZED ROS2 Wrapper and install other dependencies from source
WORKDIR /root/ros2_ws/src
RUN git clone --branch ${ZED_WRAPPER_VERSION} https://github.com/stereolabs/zed-ros2-wrapper.git && \
    wget https://github.com/ros/xacro/archive/refs/tags/${XACRO_VERSION}.tar.gz -O - | tar -xvz && mv xacro-${XACRO_VERSION} xacro && \
    wget https://github.com/ros/diagnostics/archive/refs/tags/${DIAGNOSTICS_VERSION}.tar.gz -O - | tar -xvz && mv diagnostics-${DIAGNOSTICS_VERSION} diagnostics && \
    wget https://github.com/ament/ament_lint/archive/refs/tags/${AMENT_LINT_VERSION}.tar.gz -O - | tar -xvz && mv ament_lint-${AMENT_LINT_VERSION} ament-lint && \
    wget https://github.com/cra-ros-pkg/robot_localization/archive/refs/tags/${ROBOT_LOCALIZATION_VERSION}.tar.gz -O - | tar -xvz && mv robot_localization-${ROBOT_LOCALIZATION_VERSION} robot-localization && \
    wget https://github.com/stereolabs/zed-ros2-interfaces/archive/refs/tags/${ZED_MSGS_VERSION}.tar.gz -O - | tar -xvz && mv zed-ros2-interfaces-${ZED_MSGS_VERSION} zed-ros2-interfaces && \
    wget https://github.com/ros-drivers/nmea_msgs/archive/refs/tags/${NMEA_MSGS_VERSION}.tar.gz -O - | tar -xvz && mv nmea_msgs-${NMEA_MSGS_VERSION} nmea_msgs && \
    wget https://github.com/ros/angles/archive/refs/tags/${ANGLES_VERSION}.tar.gz -O - | tar -xvz && mv angles-${ANGLES_VERSION} angles && \
    wget https://github.com/ros-perception/point_cloud_transport/archive/refs/tags/${POINTCLOUD_TRANSPORT_VERSION}.tar.gz -O - | tar -xvz && mv point_cloud_transport-${POINTCLOUD_TRANSPORT_VERSION} point_cloud_transport && \
    wget https://github.com/ros-perception/point_cloud_transport_plugins/archive/refs/tags/${POINTCLOUD_TRANSPORT_PLUGINS_VERSION}.tar.gz -O - | tar -xvz && mv point_cloud_transport_plugins-${POINTCLOUD_TRANSPORT_PLUGINS_VERSION} point_cloud_transport_plugins && \
    wget https://github.com/ros2/rmw_cyclonedds/archive/refs/tags/${RMW_CYCLONEDDS_VERSION}.tar.gz -O - | tar -xvz && mv rmw_cyclonedds-${RMW_CYCLONEDDS_VERSION} rmw_cyclonedds && \
    wget https://github.com/ros-geographic-info/geographic_info/archive/refs/tags/${GEOGRAPHIC_INFO_VERSION}.tar.gz -O - | tar -xvz && mv geographic_info-${GEOGRAPHIC_INFO_VERSION} geographic-info && \
    cp -r geographic-info/geographic_msgs/ . && \
    rm -rf geographic-info && \
    wget https://github.com/4am-robotics/cob_common/archive/refs/tags/${COB_VERSION}.tar.gz -O - | tar -xvz && mv cob_common-${COB_VERSION} cob_common && \
    cp -r cob_common/cob_srvs/ . && \
    rm -rf cob_common && \
    # vision_opencv and the ffmpeg transport stack, pinned by tag (see ARGs above)
    wget https://github.com/ros-perception/vision_opencv/archive/refs/tags/${VISION_OPENCV_VERSION}.tar.gz -O - | tar -xvz && mv vision_opencv-${VISION_OPENCV_VERSION} vision_opencv && \
    git clone https://github.com/ros-misc-utilities/ffmpeg_image_transport.git && \
    git clone https://github.com/ros-misc-utilities/ffmpeg_encoder_decoder.git && \
    wget https://github.com/ros-misc-utilities/ffmpeg_image_transport_msgs/archive/refs/tags/${FFMPEG_IMAGE_TRANSPORT_MSGS_VERSION}.tar.gz -O - | tar -xvz && mv ffmpeg_image_transport_msgs-${FFMPEG_IMAGE_TRANSPORT_MSGS_VERSION} ffmpeg_image_transport_msgs
# Get only the theora/zstd plugins from image_transport_plugins (avoid OpenCV-dev deps).
# zstd_image_transport is a hard exec_depend of zed_wrapper on Jazzy and up.
RUN wget https://github.com/ros-perception/image_transport_plugins/archive/refs/tags/${IMAGE_TRANSPORT_PLUGINS_VERSION}.tar.gz -O - | tar -xvz && \
    mv image_transport_plugins-${IMAGE_TRANSPORT_PLUGINS_VERSION} image_transport_plugins && \
    rm -rf image_transport_plugins/compressed_image_transport \
           image_transport_plugins/compressed_depth_image_transport \
           image_transport_plugins/image_transport_plugins  # metapackage

# Install cython. The base image puts a venv on PATH at /opt/venv, so pip installs
# land there and Noble's externally-managed-environment guard does not trigger.
RUN python3 -m pip install --upgrade cython
RUN python3 -m pip install --no-cache-dir --index-url https://pypi.org/simple black

# Build the dependencies and the ZED ROS2 Wrapper
WORKDIR /root/ros2_ws
# CMake should find the preinstalled OpenCV under /usr/local

# List of rosdep keys that map to Ubuntu libopencv-* packages
ARG OPENCV_SKIP="opencv opencv2 OpenCV \
  libopencv-dev libopencv-core-dev libopencv-imgproc-dev \
  libopencv-highgui-dev libopencv-imgcodecs-dev \
  libopencv-video-dev libopencv-calib3d-dev libopencv-features2d-dev"

  # Add the meta-package and OpenCV-heavy plugins to the skip list
ARG EXTRA_SKIP="image_transport_plugins compressed_image_transport compressed_depth_image_transport"

    # FFmpeg dev (for ffmpeg_image_transport)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ffmpeg \
      libavcodec-dev libavformat-dev libavutil-dev \
      libswscale-dev libavdevice-dev libavfilter-dev \
      pkg-config && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-backward-ros \
    libdw-dev binutils-dev \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
 && apt-get install -y --no-install-recommends libxcb-cursor0 \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -y \
      --rosdistro ${ROS_DISTRO} \
      --skip-keys "${OPENCV_SKIP} ${EXTRA_SKIP}" \
 && rm -rf /var/lib/apt/lists/*

 # Stage 1: Build all message packages
WORKDIR /root/ros2_ws

# Optional: skip some heavy/optional plugins here
ARG SKIP_PKGS="compressed_image_transport compressed_depth_image_transport image_transport_plugins ament_black ament_cmake_black"

# A) Build everything needed up to the image & point cloud transports you care about
#    (this ensures rosidl *typesupport* is generated before dependents compile)
RUN bash -lc '\
  source /opt/ros/$ROS_DISTRO/setup.bash; \
  colcon build \
    --symlink-install \
    --parallel-workers $(nproc) \
    --event-handlers console_direct+ \
    --packages-skip ${SKIP_PKGS} \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs \
      -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined" \
      -DCMAKE_VERBOSE_MAKEFILE=ON \
'
RUN bash -lc '\
  source /opt/ros/$ROS_DISTRO/setup.bash; \
  source /root/ros2_ws/install/setup.bash; \
  colcon build \
    --symlink-install \
    --parallel-workers $(nproc) \
    --event-handlers console_direct+ \
    --packages-skip ${SKIP_PKGS} \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs \
      -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined" \
'

WORKDIR /root/ros2_ws

# Add sourcing to .bashrc
RUN echo 'source /opt/ros/$ROS_DISTRO/install/setup.bash' >> /root/.bashrc && \
  echo 'source /root/ros2_ws/install/local_setup.bash' >> /root/.bashrc


# Copy the entrypoint script
COPY ros_entrypoint.sh /sbin/ros_entrypoint.sh
RUN chmod 755 /sbin/ros_entrypoint.sh

ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]
