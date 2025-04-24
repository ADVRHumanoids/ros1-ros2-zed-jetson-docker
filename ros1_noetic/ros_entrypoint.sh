#!/bin/bash
set -e

# Setup ROS environment from the install space of the custom build
echo "Sourcing /root/catkin_ws/install/setup.bash..."
source "/root/catkin_ws/install/setup.bash"
echo "Sourcing complete."

# Execute the command passed to this script
echo "Executing command: $@"
exec "$@"