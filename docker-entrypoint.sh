#!/bin/bash
set -e

# ── USB bandwidth fix ────────────────────────────────────────────────────────
# Increase usbfs memory limit so the RealSense D435i USB 3.0 transfers do not
# hit ENOMEM under sustained RGB+Depth+IMU streaming.
# Requires the container to run with --privileged (or CAP_SYS_ADMIN).
if [ -w /sys/module/usbcore/parameters/usbfs_memory_mb ]; then
    echo 256 > /sys/module/usbcore/parameters/usbfs_memory_mb
    echo "[entrypoint] usbfs_memory_mb set to 256"
else
    echo "[entrypoint] WARNING: cannot set usbfs_memory_mb (need --privileged)"
fi

# ── Source ROS2 & workspace ──────────────────────────────────────────────────
source /opt/ros/foxy/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# ── DDS middleware ───────────────────────────────────────────────────────────
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# DDS domain — must match on ALL machines that need to discover each other.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Network interface for CycloneDDS.
# Override at runtime: docker run -e DDS_INTERFACE=wlan0 ...
# Defaults to "auto" (CycloneDDS picks the first non-loopback interface).
DDS_INTERFACE=${DDS_INTERFACE:-auto}

# Substitute $DDS_INTERFACE into the XML template at runtime so no rebuild is needed.
CYCLONEDDS_XML_TEMPLATE=/config/cyclonedds_go2.xml
CYCLONEDDS_XML_RUNTIME=/tmp/cyclonedds_runtime.xml
sed "s/\${DDS_INTERFACE}/${DDS_INTERFACE}/g" "$CYCLONEDDS_XML_TEMPLATE" > "$CYCLONEDDS_XML_RUNTIME"
export CYCLONEDDS_URI=file://${CYCLONEDDS_XML_RUNTIME}

echo "[entrypoint] DDS interface: $DDS_INTERFACE"

# ── Display for RViz2 (X11 forwarding) ───────────────────────────────────────
# If DISPLAY is not set (pure headless), RViz2 will be disabled via rviz:=false.
# To use X11 forwarding: docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ...
if [ -z "${DISPLAY}" ]; then
    echo "[entrypoint] No DISPLAY set — appending rviz:=false to launch args"
    # Inject rviz:=false only if not already specified by the caller
    if [[ "$*" != *"rviz:="* ]]; then
        set -- "$@" "rviz:=false"
    fi
fi

echo "[entrypoint] RMW: $RMW_IMPLEMENTATION"
echo "[entrypoint] DDS domain: $ROS_DOMAIN_ID"
echo "[entrypoint] DISPLAY: ${DISPLAY:-(none, headless)}"
echo "[entrypoint] Launching: $*"

exec "$@"
