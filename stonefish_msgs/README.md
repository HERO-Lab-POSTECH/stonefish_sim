# stonefish_msgs

ROS2 message and service definitions for the Stonefish marine robotics simulator.

## Overview

This package provides the core message and service interfaces for communicating with the Stonefish simulator. It includes messages for sensor data (DVL, INS, thruster state, beacon information) and services for runtime configuration of environmental conditions (ocean currents, waves, wind).

## Features

- **Sensor Messages**: DVL, INS, thruster state, beacon positioning
- **Environmental Services**: Runtime control of ocean currents, waves, and wind
- **Sonar Configuration**: Dynamic sonar parameter adjustment
- **Utility Messages**: Timestamped primitives and pose representations

## Message Definitions

### BeaconInfo.msg

Acoustic beacon positioning information.

```
std_msgs/Header header
uint8 beacon_id                           # Unique beacon identifier
float32 range                             # Distance to beacon [m]
float32 azimuth                           # Horizontal angle [rad]
float32 elevation                         # Vertical angle [rad]
geometry_msgs/Point relative_position    # Position in vehicle frame
geometry_msgs/Quaternion local_orientation
float32 local_depth                       # Beacon depth [m]
```

**Usage**: Acoustic positioning systems (USBL, LBL)

### DVL.msg

Doppler Velocity Log sensor data.

```
std_msgs/Header header
geometry_msgs/Vector3 velocity            # Measured velocity [m/s] in body frame
float64[9] velocity_covariance            # Row-major covariance (xyz)
float64 altitude                          # Altitude above seafloor [m]
stonefish_msgs/DVLBeam[] beams            # Individual beam measurements
```

**Notes**:
- Based on UUV Simulator DVL message
- If covariance is unknown, set to all zeros
- Invalid measurements have covariance = -1

### DVLBeam.msg

Individual DVL beam measurement.

```
float64 range                             # Beam range [m] (< 0 if invalid)
float64 range_covariance
float64 velocity                          # Beam velocity [m/s]
float64 velocity_covariance
geometry_msgs/PoseStamped pose            # Beam pose w.r.t. DVL link
```

### INS.msg

Inertial Navigation System data with global and local positioning.

```
std_msgs/Header header

# Global position (GPS)
float64 latitude                          # Degrees
float64 longitude                         # Degrees

# NED origin
float64 origin_latitude                   # Degrees
float64 origin_longitude                  # Degrees

# Local position (NED frame)
stonefish_msgs/NEDPose pose               # [N, E, D, roll, pitch, yaw]
stonefish_msgs/NEDPose pose_variance
float64 altitude                          # Altitude above seafloor [m]

# Velocities
geometry_msgs/Vector3 body_velocity       # Linear velocity in body frame [m/s]
geometry_msgs/Vector3 rpy_rate            # Angular velocity [rad/s]
```

**Coordinate System**: NED (North-East-Down)
- Positive Z = underwater
- Example: `[0, 0, 2.0]` = 2 meters depth

### Int32Stamped.msg

Timestamped integer value.

```
std_msgs/Header header
int32 data
```

**Usage**: Generic integer data with timestamp

### NEDPose.msg

Position and orientation in NED coordinates.

```
float64 north                             # X position [m]
float64 east                              # Y position [m]
float64 down                              # Z position [m]
float64 roll                              # Roll angle [rad]
float64 pitch                             # Pitch angle [rad]
float64 yaw                               # Yaw angle [rad]
```

**Coordinate Convention**:
- Position: NED frame (Z+ = down)
- Orientation: Roll-Pitch-Yaw (intrinsic ZYX Euler angles)

### ThrusterState.msg

Thruster array state feedback.

```
std_msgs/Header header
float64[] setpoint                        # Commanded thrust [N]
float64[] rpm                             # Actual RPM
float64[] thrust                          # Actual thrust [N]
float64[] torque                          # Actual torque [N⋅m]
```

**Array Size**: Number of thrusters (e.g., 8 for BlueROV2)

## Service Definitions

### SetMode.srv

Generic mode setting service.

**Request**:
```
string data                               # Mode name or parameter string
```

**Response**:
```
bool success                              # True if mode was set successfully
```

**Usage**: Switching operational modes or configurations

### SetOceanCurrent.srv

Control ocean current parameters at runtime.

**Request**:
```
int32 current_index                       # Ocean current index (default: 0)
bool enable                               # true: enable, false: disable
float64[3] velocity                       # [vx, vy, vz] in m/s (NED frame)
```

**Response**:
```
bool success
string message                            # Status or error description
```

**Example**:
```bash
# Enable 0.5 m/s northward current
ros2 service call /set_ocean_current stonefish_msgs/srv/SetOceanCurrent \
  "{current_index: 0, enable: true, velocity: [0.5, 0.0, 0.0]}"

# Disable current
ros2 service call /set_ocean_current stonefish_msgs/srv/SetOceanCurrent \
  "{current_index: 0, enable: false, velocity: [0.0, 0.0, 0.0]}"
```

### SetWaveHeight.srv

Adjust ocean wave height during simulation.

**Request**:
```
float64 height                            # Wave height in meters (0.0 - 10.0)
```

**Response**:
```
bool success
string message
```

**Example**:
```bash
# Set 2-meter waves
ros2 service call /set_wave_height stonefish_msgs/srv/SetWaveHeight \
  "{height: 2.0}"

# Calm sea
ros2 service call /set_wave_height stonefish_msgs/srv/SetWaveHeight \
  "{height: 0.0}"
```

### SetWindVelocity.srv

Set atmospheric wind velocity at runtime.

**Request**:
```
float64 x                                 # North component [m/s]
float64 y                                 # East component [m/s]
float64 z                                 # Down component [m/s] (typically 0.0)
```

**Response**:
```
bool success
string message
```

**Coordinate System**: NED (North-East-Down)

**Example**:
```bash
# 5 m/s wind from North-East
ros2 service call /set_wind_velocity stonefish_msgs/srv/SetWindVelocity \
  "{x: 3.5, y: 3.5, z: 0.0}"

# No wind
ros2 service call /set_wind_velocity stonefish_msgs/srv/SetWindVelocity \
  "{x: 0.0, y: 0.0, z: 0.0}"
```

### SonarSettings.srv

Configure sonar sensor parameters.

**Request**:
```
float64 range_min                         # Minimum detection range [m]
float64 range_max                         # Maximum detection range [m]
float64 gain                              # Sonar gain (0.0 - 1.0)
```

**Response**:
```
bool success
string message
```

**Example**:
```bash
ros2 service call /bluerov2/sonar/settings stonefish_msgs/srv/SonarSettings \
  "{range_min: 0.5, range_max: 50.0, gain: 0.8}"
```

### SonarSettings2.srv

Extended sonar configuration with rotation limits.

**Request**:
```
float64 range_min                         # Minimum detection range [m]
float64 range_max                         # Maximum detection range [m]
float64 rotation_min                      # Minimum rotation angle [rad]
float64 rotation_max                      # Maximum rotation angle [rad]
float64 gain                              # Sonar gain (0.0 - 1.0)
```

**Response**:
```
bool success
string message
```

**Usage**: Mechanical scanning sonars with limited field of view

## Installation

### Building from Source

```bash
cd /workspace/colcon_ws
colcon build --packages-select stonefish_msgs
source install/setup.bash
```

### Dependencies

- ROS2 Humble
- `std_msgs`
- `geometry_msgs`

## Usage Examples

### Publishing to Topics

```bash
# Publish beacon info
ros2 topic pub /beacon stonefish_msgs/msg/BeaconInfo \
  "{header: {frame_id: 'base_link'}, beacon_id: 1, range: 15.5, azimuth: 0.785, elevation: 0.0}"
```

### Calling Services

```bash
# List available services
ros2 service list | grep -E "(current|wave|wind)"

# Get service type
ros2 service type /set_ocean_current

# Call service with tab completion
ros2 service call /set_ocean_current stonefish_msgs/srv/SetOceanCurrent <TAB>
```

### Python API

```python
import rclpy
from rclpy.node import Node
from stonefish_msgs.msg import DVL, INS
from stonefish_msgs.srv import SetOceanCurrent

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')

        # Subscribe to DVL
        self.dvl_sub = self.create_subscription(
            DVL,
            '/bluerov2/dvl',
            self.dvl_callback,
            10
        )

        # Subscribe to INS
        self.ins_sub = self.create_subscription(
            INS,
            '/bluerov2/ins',
            self.ins_callback,
            10
        )

        # Service client for ocean current
        self.current_client = self.create_client(
            SetOceanCurrent,
            '/set_ocean_current'
        )

    def dvl_callback(self, msg):
        self.get_logger().info(f'DVL Velocity: {msg.velocity}')
        self.get_logger().info(f'Altitude: {msg.altitude:.2f} m')

    def ins_callback(self, msg):
        self.get_logger().info(
            f'Position (NED): [{msg.pose.north:.2f}, '
            f'{msg.pose.east:.2f}, {msg.pose.down:.2f}]'
        )

    def set_current(self, velocity):
        """Enable ocean current with specified velocity."""
        request = SetOceanCurrent.Request()
        request.current_index = 0
        request.enable = True
        request.velocity = velocity

        future = self.current_client.call_async(request)
        return future

def main():
    rclpy.init()
    node = SensorSubscriber()

    # Set northward current
    future = node.set_current([0.5, 0.0, 0.0])
    rclpy.spin_until_future_complete(node, future)

    response = future.result()
    node.get_logger().info(f'Current set: {response.success}')

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ API

```cpp
#include <rclcpp/rclcpp.hpp>
#include <stonefish_msgs/msg/dvl.hpp>
#include <stonefish_msgs/srv/set_ocean_current.hpp>

class SensorSubscriber : public rclcpp::Node {
public:
    SensorSubscriber() : Node("sensor_subscriber") {
        dvl_sub_ = this->create_subscription<stonefish_msgs::msg::DVL>(
            "/bluerov2/dvl", 10,
            std::bind(&SensorSubscriber::dvl_callback, this, std::placeholders::_1)
        );

        current_client_ = this->create_client<stonefish_msgs::srv::SetOceanCurrent>(
            "/set_ocean_current"
        );
    }

private:
    void dvl_callback(const stonefish_msgs::msg::DVL::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Altitude: %.2f m", msg->altitude);
    }

    rclcpp::Subscription<stonefish_msgs::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Client<stonefish_msgs::srv::SetOceanCurrent>::SharedPtr current_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

## Package Structure

```
stonefish_msgs/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package manifest
├── msg/                        # Message definitions
│   ├── BeaconInfo.msg
│   ├── DVL.msg
│   ├── DVLBeam.msg
│   ├── INS.msg
│   ├── Int32Stamped.msg
│   ├── NEDPose.msg
│   └── ThrusterState.msg
└── srv/                        # Service definitions
    ├── SetMode.srv
    ├── SetOceanCurrent.srv     # New: Ocean current control
    ├── SetWaveHeight.srv       # New: Wave height control
    ├── SetWindVelocity.srv     # New: Wind velocity control
    ├── SonarSettings.srv
    └── SonarSettings2.srv
```

## Coordinate Systems

All messages use the **NED (North-East-Down)** coordinate convention:

- **X-axis**: North (forward)
- **Y-axis**: East (right)
- **Z-axis**: Down (underwater)

**Important**:
- Positive Z values indicate underwater positions
- Example: `[0, 0, 2.0]` = 2 meters below surface
- Roll-Pitch-Yaw angles follow intrinsic ZYX Euler convention

## Related Packages

- **stonefish_ros2**: Core simulator interface (publishes these messages)
- **stonefish_description**: Robot and world definitions
- **stonefish_control_msgs**: Control-specific messages
- **stonefish_slam**: SLAM using DVL and INS data

## References

- **Stonefish Simulator**: https://stonefish.readthedocs.io
- **UUV Simulator** (original DVL message source): https://github.com/uuvsimulator/uuv_simulator
- **ROS2 Humble Messages**: https://docs.ros.org/en/humble/

## License

GPL v3.0 (consistent with Stonefish library)
