# Pilot02-Physical Ouster Lidar ROS 2 Driver

This repository contains the ROS 2 driver for **Pilot02-Physical Ouster Lidar** based on https://github.com/ouster-lidar/ouster-ros/tree/ros2 package. It includes all the necessary configurations, launch files, and a Docker setup to deploy the ouster Lidar driver in a containerized environment.

## Description

This project provides an easy-to-use setup for integrating Ouster Lidar devices into ROS 2. The `ouster_ros` node supports configuration and the system can be run inside a Docker container for ease of use.

The repository includes the following components:
- Configuration files for the ouster Lidar device
- Launch files for running the ROS 2 node
- Docker setup for easy containerization
- A docker-compose based test environment to verify the topics published by the ouster driver (lidar hardware required)

## Guidelines for build and test the component 

### 1. **Build the Main Docker Image:**

In this step, we build the Docker image using the provided `Dockerfile`. The image is named `pilot02-physical-ouster-lidar`.

```bash
docker build -t pilot02-physical-ouster-lidar .
```

Make sure the path to your configuration and launch files is correctly mapped to the Docker container.

### 2. **Run the ROS 2 Container:**

After building the Docker image, you can run the container using the following command:

```bash
docker run -e LAUNCH_FILE=your_custom_launch_file.py pilot02-physical-ouster-lidar
```

This will start the container and launch the ROS 2 node with the configured launch file.

### 3. **Build and Run the test automation:**

Test automation is integrated by docker-compose file:

Run: 
```bash
docker-compose up --build
```

## Environment Variables List for Sensor Configuration

- **ros_namespace**
  - Description: Defines the namespace for all nodes.
  - Default: "ouster"
  - Example: 
    ```bash
    -e ros_namespace=ouster_ns
    ```

- **sensor_hostname**
  - Description: The IP address or hostname of the sensor (required).
  - Default: empty
  - Example: 
    ```bash
    -e sensor_hostname=192.168.1.100
    ```

- **udp_dest**
  - Description: The IP address of the destination for UDP packets.
  - Default: empty
  - Example: 
    ```bash
    -e udp_dest=192.168.1.200
    ```

- **lidar_port**
  - Description: The port on which the sensor will send lidar data.
  - Default: 0 (auto-assigned)
  - Example:
    ```bash
    -e lidar_port=8000
    ```

- **imu_port**
  - Description: The port on which the sensor will send IMU data.
  - Default: 0 (auto-assigned)
  - Example:
    ```bash
    -e imu_port=8001
    ```

- **lidar_mode**
  - Description: The lidar resolution and rate.
  - Possible values: `512x10`, `512x20`, `1024x10`, `1024x20`, `2048x10`, `4096x5`
  - Example:
    ```bash
    -e lidar_mode=1024x20
    ```

- **timestamp_mode**
  - Description: The method used to timestamp measurements.
  - Possible values: 
    - `time_from_internal_osc`
    - `time_from_sync_pulse_in`
    - `time_from_ptp_1588`
    - `time_from_ros_time`
  - Example:
    ```bash
    -e timestamp_mode=time_from_ptp_1588
    ```

- **ptp_utc_tai_offset**
  - Description: UTC/TAI offset in seconds when using `time_from_ptp_1588`.
  - Default: `-37.0`
  - Example:
    ```bash
    -e ptp_utc_tai_offset=-37.0
    ```

- **udp_profile_lidar**
  - Description: The lidar packet profile.
  - Possible values: 
    - `legacy`
    - `rng19_rfl8_sig16_nir16`
    - `rng15_rfl8_nir8`
    - `rng19_rfl8_sig16_nir16_dual`
    - `fusa_rng15_rfl8_nir8_dual`
  - Example:
    ```bash
    -e udp_profile_lidar=rng19_rfl8_sig16_nir16
    ```

- **metadata**
  - Description: Path to store the metadata file.
  - Default: empty
  - Example:
    ```bash
    -e metadata=/path/to/metadata.txt
    ```

### TF and Frame Configuration

- **sensor_frame**
  - Description: The reference frame name for the sensor.
  - Default: `os_sensor`
  - Example:
    ```bash
    -e sensor_frame=os_sensor
    ```

- **lidar_frame**
  - Description: The reference frame name for the lidar.
  - Default: `os_lidar`
  - Example:
    ```bash
    -e lidar_frame=os_lidar
    ```

- **imu_frame**
  - Description: The reference frame name for the IMU.
  - Default: `os_imu`
  - Example:
    ```bash
    -e imu_frame=os_imu
    ```

- **point_cloud_frame**
  - Description: The reference frame for publishing PointCloud2 or LaserScan messages.
  - Default: `os_lidar`
  - Example:
    ```bash
    -e point_cloud_frame=os_sensor
    ```

- **pub_static_tf**
  - Description: If set to `true`, the driver will broadcast TF transforms for imu/sensor/lidar frames.
  - Default: `true`
  - Example:
    ```bash
    -e pub_static_tf=true
    ```

### Additional Parameters

- **proc_mask**
  - Description: The processors to enable or disable (IMG, PCL, IMU, SCAN, RAW, TLM).
  - Default: `imu|pcl|scan|img|raw|tlm`
  - Example:
    ```bash
    -e proc_mask=imu|pcl|scan
    ```

- **scan_ring**
  - Description: The specific scan ring to use.
  - Default: `0`
  - Example:
    ```bash
    -e scan_ring=1
    ```

- **use_system_default_qos**
  - Description: Use default system Quality of Service settings.
  - Default: `false`
  - Example:
    ```bash
    -e use_system_default_qos=false
    ```

- **point_type**
  - Description: Choose the point type for generating the point cloud.
  - Possible values: 
    - `original`, `native`, `xyz`, `xyzi`, `o_xyzi`, `xyzir`
  - Example:
    ```bash
    -e point_type=xyz
    ```

- **azimuth_window_start**
  - Description: The start of the azimuth window in millidegrees.
  - Default: `0`
  - Example:
    ```bash
    -e azimuth_window_start=0
    ```

- **azimuth_window_end**
  - Description: The end of the azimuth window in millidegrees.
  - Default: `360000`
  - Example:
    ```bash
    -e azimuth_window_end=360000
    ```

- **persist_config**
  - Description: Request the sensor to persist settings.
  - Default: `false`
  - Example:
    ```bash
    -e persist_config=false
    ```

- **attempt_reconnect**
  - Description: Whether to attempt reconnecting to the sensor after connection loss.
  - Default: `false`
  - Example:
    ```bash
    -e attempt_reconnect=true
    ```

- **dormant_period_between_reconnects**
  - Description: Time in seconds to wait between reconnection attempts.
  - Default: `1.0`
  - Example:
    ```bash
    -e dormant_period_between_reconnects=1.0
    ```

- **max_failed_reconnect_attempts**
  - Description: Maximum number of attempts to reconnect to the sensor.
  - Default: `2147483647`
  - Example:
    ```bash
    -e max_failed_reconnect_attempts=10
    ```

- **auto_start**
  - Description: Automatically configure and activate the node.
  - Default: `true`
  - Example:
    ```bash
    -e auto_start=true
    ```

- **organized**
  - Description: Whether to generate an organized point cloud.
  - Default: `true`
  - Example:
    ```bash
    -e organized=true
    ```

- **destagger**
  - Description: Enable or disable point cloud destaggering.
  - Default: `true`
  - Example:
    ```bash
    -e destagger=true
    ```

- **min_range**
  - Description: Minimum lidar range to consider (in meters).
  - Default: `0.0`
  - Example:
    ```bash
    -e min_range=0.0
    ```

- **max_range**
  - Description: Maximum lidar range to consider (in meters).
  - Default: `1000.0`
  - Example:
    ```bash
    -e max_range=1000.0
    ```

- **v_reduction**
  - Description: Vertical beam reduction.
  - Possible values: `{1, 2, 4, 8, 16}`
  - Default: `1`
  - Example:
    ```bash
    -e v_reduction=2
    ```

- **min_scan_valid_columns_ratio**
  - Description: The minimum ratio of valid columns for processing LidarScan.
  - Default: `0.0`
  - Example:
    ```bash
    -e min_scan_valid_columns_ratio=0.1
    ```

## Example for cogniman pilot02

Build component: 
```bash
docker build  -t pilot02-physical-ouster-lidar .
```

Rebuild component (no cache):
```bash
docker build --no-cache -t pilot02-physical-ouster-lidar .
```

Run component using your own launch file and properties (example):
```bash
docker run -e LAUNCH_FILE=pilot2.sensor.composite.launch.xml pilot02-physical-ouster-lidar -- sensor_hostname:=192.168.1.115 lidar_port:=7502 imu_port:=7503 rviz:=false
```

Run component using your own launch file from the host and properties (example):
```bash
docker run -v /path/to/launch:/opt/ros2_ws/src/ouster-ros/launch -e LAUNCH_FILE=your.launch.file pilot02-physical-ouster-lidar -- sensor_hostname:=192.168.1.115 lidar_port:=7502 imu_port:=7503 rviz:=false
```

Build test: 
```bash
docker build -t test-pilot02-physical-ouster-lidar ./test
```

Rebuild test (no cache):
```bash
docker build --no-cache -t test-pilot02-physical-ouster-lidar ./test
```

Run test: 
```bash
docker run -it --rm test-pilot02-physical-ouster-lidar
```

## Contributing

Feel free to open issues or submit pull requests. Contributions are welcome!

## License

This project is licensed under the BSD-3-Clause and BSL-1.0 Licences - see the [LICENSE](LICENSE) file for details.