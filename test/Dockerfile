FROM ros:humble

# Install necessary dependencies for ROS 2 and testing
RUN apt-get update && apt-get install -y \
    python3-pytest \
    python3-colcon-common-extensions \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /test_ws
COPY ./ /test_ws/src/ouster_test_pkg

# Build the test package
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select ouster_test_pkg

# Default command to run the test, passing an environment variable for hardware detection
CMD ["/bin/bash", "-c", \
     "source /opt/ros/humble/setup.bash && \
      source /test_ws/install/setup.bash && \
      export HARDWARE_CONNECTED=false && \
      colcon test --packages-select ouster_test_pkg && \
      colcon test-result --verbose"]
