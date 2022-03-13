![total lines](https://img.shields.io/tokei/lines/github/amaarquadri/fydp)

# Autonomous Rail Inspection System's Fourth Year Design Project
An all-inclusive beach resort. Unscented and Extended Kalman Filters,
Controllers, and control systems for a 25 state autonomous plane.

ROS Topics:
- raw_image
- rectified_image
- rail_detection
- optical_flow_reading
- pressure_sensor_reading
- imu_reading
- gps_reading
- aircraft_state
- reference_command
- state_command
- control_inputs

ROS Nodes:
- camera_node
- pressure_sensor_node
- imu_node
- gps_node
- defisheye_node
- rail_detection_node
- optical_flow_node
- state_estimation_node
- global_path_planner_node
- local_path_planner_node
- low_level_control_loop_node
- propeller_node
- aileron_node
- elevator_node

Reference Frames:
- NED (north, east, down) absolute reference frame: 
  - X-axis points north
  - Y-axis points east
  - Z-axis points down
  - Origin is at the startup location of the robot at an altitude of zero
  - Used for /aircraft_state/pose/position
- NED robot reference frame (moving):
  - X-axis points forwards
  - Y-axis points to the right (starboard)
  - Z-axis points downwards
  - Origin is at the center of mass of the aircraft
  - Rotation from absolute NED to robot NED is given by /aircraft_state/pose/orientation
  - Used for /aircraft_state/twist
- NWU (north, west, up) absolute reference frame:
    - X-axis points north
    - Y-axis points west
    - Z-axis points upwards
    - Origin is at the startup location of the robot at an altitude of zero
    - Used by Gazebo
- NWU robot reference frame (moving):
  - X-axis points forwards
  - Y-axis points to the left (port side)
  - Z-axis points upwards
  - Origin is at the center of mass of the aircraft
  - Used in the Gazebo plugin
