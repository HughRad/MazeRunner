#!/bin/bash

echo "Setting up ROS workspace overlay for maze_runner..."

# First, source the base ROS installation
source /opt/ros/$(rosversion -d)/setup.bash
echo "✓ Sourced ROS $(rosversion -d)"

# Check and source the MoveIt workspace
MOVEIT_PATH=""
if [ -d ~/ws_moveit ]; then
  MOVEIT_PATH=~/ws_moveit
  source $MOVEIT_PATH/devel/setup.bash
  echo "✓ Sourced ws_moveit as underlay"
elif [ -d ~/moveit_ws ]; then
  MOVEIT_PATH=~/moveit_ws
  source $MOVEIT_PATH/devel/setup.bash
  echo "✓ Sourced moveit_ws as underlay"
else
  echo "✗ ERROR: Neither ~/ws_moveit nor ~/moveit_ws found! MoveIt dependencies will be missing."
  exit 1
fi

# Verify MoveIt is available
if ! rospack find moveit_core >/dev/null 2>&1; then
  echo "✗ ERROR: moveit_core package not found after sourcing MoveIt workspace."
  echo "  Check that your MoveIt workspace was built successfully and contains MoveIt packages."
  exit 1
fi

# Debug information - show environment
echo "------- Environment Diagnostics -------"
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
echo "CPATH: $CPATH"

# Find MoveIt headers to verify they're in searchable paths
HEADER_PATH=$(find $MOVEIT_PATH -name "move_group_interface.h" | head -1)
if [ -n "$HEADER_PATH" ]; then
  echo "✓ Found move_group_interface.h at: $HEADER_PATH"
  
  # Extract directory path
  HEADER_DIR=$(dirname "$HEADER_PATH")
  echo "  Header directory: $HEADER_DIR"
  
  # Add to CPATH explicitly to help compiler find it
  export CPATH="$HEADER_DIR:$CPATH"
  echo "  Added to CPATH"
else
  echo "✗ WARNING: Could not find move_group_interface.h in $MOVEIT_PATH."
  echo "  This may cause compilation errors."
fi
echo "---------------------------------------"

# Now work with catkin_ws as an overlay
if [ -d ~/catkin_ws ]; then
  cd ~/catkin_ws
  
  # Check for the maze_runner package
  if [ ! -d src/maze_runner ]; then
    echo "✗ ERROR: maze_runner package not found in ~/catkin_ws/src/"
    exit 1
  fi
  
  echo "Building maze_runner package..."
  
  # First, clean the build for this package
  echo "Cleaning previous build artifacts..."
  rm -rf build/maze_runner devel/lib/maze_runner devel/share/maze_runner
  
  # Build the package with full output
  catkin_make --pkg maze_runner VERBOSE=1
  
  if [ $? -eq 0 ]; then
    # Source the overlay workspace
    source ~/catkin_ws/devel/setup.bash
    echo "✓ Build successful! Sourced catkin_ws as overlay."
    echo "✓ Workspace overlay complete: ROS → $(basename $MOVEIT_PATH) → catkin_ws"
    
    echo ""
    echo "Ready to use maze_runner package with MoveIt functionality!"
    echo "You can now run: roslaunch maze_runner maze_runner.launch"
  else
    echo "✗ Build failed. See errors above."
    
    # Check if the problematic header can be found in ROS include paths
    echo "------- Error Diagnostics -------"
    echo "Checking ROS include paths for problematic header..."
    HEADER_PATHS=$(find /opt/ros/$(rosversion -d)/include -name "move_group_interface.h" 2>/dev/null)
    if [ -n "$HEADER_PATHS" ]; then
      echo "Found in ROS include paths:"
      echo "$HEADER_PATHS"
    else
      echo "Not found in ROS include paths."
    fi
    
    # Add explicit include directories to CMakeLists.txt
    echo ""
    echo "You may need to manually modify your CMakeLists.txt to add:"
    echo "include_directories($HEADER_DIR)"
    echo "---------------------------------"
  fi
else
  echo "✗ ERROR: ~/catkin_ws not found!"
  exit 1
fi
