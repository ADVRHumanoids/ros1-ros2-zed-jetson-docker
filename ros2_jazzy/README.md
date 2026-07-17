# ROS 2 Jazzy + ZED on Jetson Orin — status, constraints, and what changed

This directory is the Jazzy counterpart of `ros2_humble/`. **Read this first**: the
migration is not a drop-in distro swap, and it is not on a supported path.

## Read this before you build

ROS 2 Jazzy exists **only on Ubuntu 24.04**. On Jetson the Ubuntu version is tied to
the JetPack version, and that is where the problem starts:

| | Ubuntu | ROS 2 | Orin support |
|---|---|---|---|
| JetPack 6 / L4T r36.x | 22.04 Jammy | Humble | yes — what you run today |
| JetPack 7 / L4T r38.x | 24.04 Noble | Jazzy | **no — Thor only** |

So **an Orin cannot be flashed to an Ubuntu 24.04 host**. Stereolabs state this
directly: *"the Ubuntu version is tied to the Jetpack version"* and *"Nvidia does not
support Jetpack 7 (which comes with Ubuntu24) for the Orin module"*, recommending
*"the easiest way of having support is to use the images for L4T36.4"* (i.e. 22.04 /
Humble). Earlier: *"The ZED Box cannot work with Ubuntu 24.04. NVIDIA does not provide
drivers for this version of Ubuntu."*

The only remaining route to Jazzy on an Orin is the one this directory takes: keep the
**JetPack 6 host unchanged** and run an **Ubuntu 24.04 container** on it. The base image
`dustynv/ros:jazzy-desktop-r36.4.0-cu128-24.04` is exactly that — Noble userspace and
CUDA 12.8 built against the L4T r36.4 driver stack.

The catch is the ZED SDK. Stereolabs ship the Jetson SDK as **L4T-tied builds only**, and
the L4T r36.4 build is compiled for Jammy. There is no `l4t-r36.4 + ubuntu24.04` image in
`stereolabs/zed` — their L4T images are 22.04, and their 24.04 images are desktop/Thor.
So this container installs a **Jammy-built ZED SDK into a Noble userspace**, and the
host's Jammy-built Tegra driver libraries get mounted in by the NVIDIA container runtime.
NVIDIA's own `jetson-containers` pairs these (its `zed` package lists
`distros=['resolue', 'humble', 'jazzy']` while pinning the l4t36.4 SDK), so it is
*intended* to work — but it is **not supported by Stereolabs and not validated here**.

**Expect trouble at runtime rather than at build time**, most likely around the ZED X
Mini's GMSL2 path (`zed_x_daemon`, nvargus, `libv4l2`), which is the part most exposed to
the Jammy/Noble split. If the camera does not open, that is the first place to look —
not the ROS build.

**If you just need a working ZED X Mini on Orin, stay on `ros2_humble/`.**
Humble is supported until May 2027.

## Build and run, step by step on the Jetson

The flow matches `ros2_humble/`, but because this container is on an unsupported path
each step below says **how to tell it worked**. Do not skip step 0 — it is what tells you
whether the base image matches your board at all.

The image and container are named separately from the Humble ones
(`zed_ros2_jazzy_jetson_orin` / `zed_ros2_jazzy_container`), so both can coexist. Nothing
here touches the Humble setup.

### 0. Pre-flight checks on the Jetson host

Run these **on the Jetson**, before building:

```bash
uname -m                              # expect: aarch64
cat /etc/nv_tegra_release             # expect: # R36 (release), REVISION: 4.x
docker info | grep -i "^ Runtimes"    # expect 'nvidia' to be listed
sudo systemctl status zed_x_daemon    # expect: active (running)  -- ZED X Mini / GMSL2 only
```

- **`/etc/nv_tegra_release` must say R36, REVISION 4.x.** The base image is pinned to
  `r36.4.0`. R36.4.x is fine (JetPack 6.1/6.2). If you see R35 or R38, stop: the tag in
  `docker-compose.yml` is wrong for this board, and R38 (JetPack 7) is not an Orin.
- **`zed_x_daemon` must be running on the host**, not in the container. The ZED X Mini is
  a GMSL2 camera and the daemon owns it; the container only mounts its unit file.
- If `nvidia` is not listed as a runtime, the NVIDIA Container Toolkit is not configured
  and nothing below will see the GPU.

### 1. Allow the container to reach the X server

Once per boot, on the host (this is the same command your `ros1_noetic` scripts use):

```bash
export DISPLAY=:0
sudo XAUTHORITY=/run/user/$(id -u gdm)/gdm/Xauthority xhost +si:localuser:root
```

If that path is wrong for your display manager, see "Setup Host Machine" in the
repository root `README.md`.

### 2. Build

**Build on the Jetson itself** (the base image is `linux/arm64`), or cross-compile from an
x86 machine following "Cross-Compilation" in the root `README.md`. From this directory:

```bash
docker compose build 2>&1 | tee build.log
```

Expect this to take a **long time** (hours on-device): it pulls a multi-GB base image and
then colcon-builds the whole dependency stack plus the ZED wrapper from source.

The first thing worth watching for is early: the Dockerfile probes the ZED SDK URL and
**fails the build on purpose** if it does not resolve. If it gets past that, the download
is fine.

Success looks like `docker compose build` exiting 0. Then confirm the image exists:

```bash
docker images | grep zed_ros2_jazzy_jetson_orin
```

### 3. Start the container

```bash
docker compose up -d
docker exec -it zed_ros2_jazzy_container bash
```

The entrypoint prints a banner and runs `ros2 pkg list | grep zed`. **That banner is your
first real check**: if it lists `zed_wrapper`, `zed_components` and `zed_msgs`, the ROS
side of the migration built correctly. If the banner is missing them, the problem is the
build, not the camera.

### 4. Launch the camera

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm
```

This is the step most likely to fail on this container — see below. To confirm the camera
is actually streaming (not just that the node started), from a second shell:

```bash
docker exec -it zed_ros2_jazzy_container bash
ros2 topic list | grep zed                  # discover the real topic names
ros2 topic hz /zed/zed_node/rgb/image_rect_color   # substitute a topic from the list
```

A non-zero, steady rate from `ros2 topic hz` is the only proof the camera opened. A node
that starts and publishes nothing means the SDK did not get the camera.

## If it breaks

**Node starts but the camera never opens** — this is the failure this container is most
exposed to, and it is *not* a ROS problem. It is the Jammy-built SDK meeting the Noble
userspace on the GMSL2 path. Check, in this order:

1. `sudo systemctl status zed_x_daemon` **on the host** — must be running.
2. `ls -l /dev/video*` on the host and inside the container — they should match
   (`/dev` is bind-mounted).
3. Compare against `ros2_humble/`: **build and run the Humble container on the same
   board**. If Humble sees the camera and Jazzy does not, you have confirmed the
   Jammy/Noble split is the cause, and there is no fix on this path — the ZED SDK for
   Jetson is only built against L4T/Jammy.

Note that the SDK is installed with `silent skip_tools`, so **`ZED_Explorer` and
`ZED_Diagnostic` are not available inside the container**. Diagnose from the host, or via
ROS topics as above.

**Build fails resolving apt packages** — check whether the base image tag still exists;
only two Jazzy tags were ever published for r36.4.0, both dated 2025-03-03, and this
directory depends on them.

## What changed vs `ros2_humble/`, and why

Everything below is a change that is **required** and would not be caught by only
swapping `humble` → `jazzy`.

### 1. Base image — the tag naming scheme changed

```
humble: dustynv/ros:humble-desktop-l4t-r36.4.0
jazzy:  dustynv/ros:jazzy-desktop-r36.4.0-cu128-24.04
```

The `l4t-` prefix is gone and the CUDA and Ubuntu versions are now part of the tag, so
`L4T_VERSION` alone no longer builds a valid tag. `docker-compose.yml` therefore passes
`L4T_VERSION` / `CUDA_TAG` / `UBUNTU_RELEASE` separately. These are the only two Jazzy
tags that exist for r36.4.0 (`jazzy-desktop-…` and `jazzy-ros-base-…`, both 2025-03-03).

Consequences of the base image itself: **Python 3.12** (was 3.10), **CUDA 12.8** (was
12.6, works on a JetPack 6 host via `/usr/local/cuda/compat`), **NumPy ≥ 2.0** forced by
the image, and a **venv on `PATH` at `/opt/venv`** — which is what keeps `pip install`
from tripping Noble's `externally-managed-environment` guard.

`/opt/ros/jazzy/install/setup.bash` remains the correct path: the base image still builds
ROS from source with `colcon build --merge-install` into `$ROS_ROOT`, so `ros_entrypoint.sh`
needed no path change.

### 2. `libgeographic-dev` does not exist on Noble — hard build failure

Renamed upstream; rosdep encodes exactly this:

```yaml
geographiclib:
  ubuntu:
    '*': [libgeographiclib-dev]   # noble and later
    jammy: [libgeographic-dev]    # what ros2_humble/ uses
```

### 3. Two `--branch ${ROS_DISTRO}` clones would fail — there is no `jazzy` branch

This is the trap that makes a naive `humble` → `jazzy` substitution fail:

- **`vision_opencv`** — branches go `humble` → `iron` → `rolling`. No `jazzy`.
- **`ffmpeg_image_transport_msgs`** — has `humble`, `iron`, `master`, `release`, `rolling`. No `jazzy`.

Both are now pinned to the tag released into Jazzy (`4.1.0` and `1.3.0`). `image_transport_plugins`
*does* have a `jazzy` branch but is pinned by tag too, for reproducibility.

### 4. Dependency versions — several are major-version jumps

Pinned to what rosdistro actually released into Jazzy
([`jazzy/distribution.yaml`](https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml));
every tag below was verified to resolve.

| Package | Humble (was) | Jazzy (now) |
|---|---|---|
| `point_cloud_transport` | 1.0.18 | **4.0.9** |
| `point_cloud_transport_plugins` | 1.0.11 | **4.0.4** |
| `rmw_cyclonedds` | 1.3.4 | **2.2.3** |
| `vision_opencv` | branch `humble` | **4.1.0** |
| `image_transport_plugins` | branch `humble` | **4.0.7** |
| `ament_lint` | 0.12.11 | **0.17.5** |
| `robot_localization` | 3.5.3 | **3.8.3** |
| `diagnostics` | 4.0.0 | **4.2.7** |
| `xacro` | 2.0.8 | **2.1.1** |
| `angles` | 1.15.0 | **1.16.1** |
| `nmea_msgs` | 2.0.0 | **2.1.0** |
| `cob_common` | 2.7.10 | **2.8.12** |
| `ffmpeg_image_transport_msgs` | branch `humble` | **1.3.0** |
| `geographic_info` | 1.0.6 | 1.0.6 (unchanged) |

`zed-ros2-interfaces` goes 5.0.0 → **5.3.0** (its latest; the wrapper is at 5.4.0 —
upstream does not keep the two in lockstep).

`zed_wrapper` gains a Jazzy-only hard dependency,
`<exec_depend condition="$ROS_DISTRO >= jazzy">zstd_image_transport</exec_depend>`,
which is why `image_transport_plugins` must still be built (only the OpenCV-heavy
`compressed*` plugins are stripped, as before).

### 5. ZED SDK 5.0.5 → 5.4.0, and a better URL

`zed-ros2-wrapper` master **requires ZED SDK ≥ 5.2**. Worth flagging: `ros2_humble/`
pins SDK 5.0.5 while cloning the wrapper from an unpinned `master`, so that combination
is *already* inconsistent with upstream's stated requirement and will drift further.

The URL now uses the official redirect endpoint instead of a hardcoded CDN path:

```
was: https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/5.0/ZED_SDK_Tegra_L4T36.4_v5.0.5.zstd.run
now: https://download.stereolabs.com/zedsdk/5.4.0/l4t36.4/jetsons
```

This is the form Stereolabs use in their own Dockerfiles; it resolves to the current
`.run` and survives CDN changes. It redirects, so the existence check now uses `curl -L -I`.

`ZED_WRAPPER_VERSION` was added as a build arg (default `master`, i.e. today's behaviour).
Pinning it to `v5.4.0` is recommended for reproducible builds.

## Sources

- [Stereolabs: JetPack 7 / Ubuntu 24.04 not supported on Orin](https://community.stereolabs.com/t/zed-box-mini-and-ubuntu-24-04-support/11030)
- [Stereolabs: Ubuntu 24.04 support depends on NVIDIA's L4T BSP](https://community.stereolabs.com/t/ubuntu-24-04-support/5965)
- [zed-ros2-wrapper — supported distros and SDK requirement](https://github.com/stereolabs/zed-ros2-wrapper)
- [jetson-containers — `zed` package pairing the l4t36.4 SDK with jazzy](https://github.com/dusty-nv/jetson-containers/tree/master/packages/hw/zed)
- [rosdistro — `jazzy/distribution.yaml`](https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml)
- [ROS signing key migration guide](https://discourse.openrobotics.org/t/ros-signing-key-migration-guide/43937)
