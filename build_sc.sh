#!/bin/bash

# Define the workspace directory
WORKSPACE_DIR="/home/veesion"

# Check if the workspace directory exists
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Workspace directory not found: $WORKSPACE_DIR"
    exit 1
fi

# Navigate to the workspace directory
echo "Changing to workspace directory: $WORKSPACE_DIR"
cd "$WORKSPACE_DIR"
mkdir src
sudo rosdep update
sudo rosdep install --from-paths src --ignore-src -y
sudo chown -R $(whoami) /home/veesion/

# Clean up old build files if exists
echo "Cleaning up old build files (build/, install/, and log/)..."
rm -rf build install log

# Clone and build OpenVINS project 
echo "Building OpenVINS project using colcon..."
cd src
git clone https://github.com/rpng/open_vins/
cd ..
export MAKEFLAGS="-j 1"
colcon build --executor sequential
colcon build --event-handlers console_cohesion+ --packages-select ov_core ov_init ov_msckf ov_eval

# Clone imu_tools
cd src
git clone -b humble https://github.com/CCNYRoboticsLab/imu_tools.git
cd ..

# Build the workspace
echo "Building the workspace using colcon..."
colcon build --symlink-install

# Source the workspace
echo "Sourcing the workspace install/setup.bash..."
source install/setup.bash

# Add sourcing to .bashrc if not already present
if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
    echo "Adding workspace sourcing to ~/.bashrc..."
    echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
else
    echo "Workspace sourcing already present in ~/.bashrc."
fi

# Additional setup (if needed)
echo "Workspace build completed and environment sourced."
echo "You can now run your ROS 2 nodes or launch files."

