# stonefish_ros2

ROS2 interface package for the Stonefish marine robotics simulator.

## Overview

This package provides a complete ROS2 interface for the [Stonefish](https://github.com/patrykcieslak/stonefish) library, enabling realistic marine robotics simulation with advanced hydrodynamics, underwater rendering, and sensor modeling. It includes:

- **C++ ROS2 Interface Library**: Core communication layer between Stonefish and ROS2
- **Simulator Nodes**: GPU and no-GPU variants for running simulations
- **Scenario Parser**: Extended XML parser with ROS2 parameter resolution and resource search
- **Message Interfaces**: Complete sensor, actuator, and state publishing/subscription
- **Environmental Control Services**: Runtime adjustment of ocean currents, waves, and wind

## Features

- **Realistic Marine Physics**: Hydrodynamics, buoyancy, ocean currents, waves
- **Advanced Sensors**: DVL, INS, IMU, cameras, sonar (FLS, SSS, MSIS), multibeam
- **Actuators**: Thrusters, servos, propellers with realistic dynamics
- **GPU Rendering**: High-quality underwater visualization (OpenGL)
- **NED Coordinate System**: Consistent North-East-Down convention
- **Runtime Configuration**: Services for environmental parameter adjustment
- **Multi-Robot Support**: Multiple vehicles in single simulation

## Installation

### Prerequisites

1. **Install Stonefish Library** (same version as this package):

```bash
cd /workspace/stonefish/build
cmake ..
make -j8
sudo make install
```

The library installs to `/usr/local/lib/libStonefish.so`.

2. **ROS2 Dependencies**:

- ROS2 Humble
- `stonefish_msgs` (message definitions)
- `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_msgs`
- `image_transport`
- `tf2_ros`

### Building

```bash
cd /workspace/colcon_ws

# Build stonefish_msgs first (dependency)
colcon build --packages-select stonefish_msgs

# Build stonefish_ros2
colcon build --packages-select stonefish_ros2

source install/setup.bash
```

## Usage

### Quick Start - BlueROV2 Simulation

```bash
# Launch BlueROV2 with thruster manager
ros2 launch stonefish_ros2 bluerov2.launch.py

# Launch without thruster manager
ros2 launch stonefish_ros2 bluerov2.launch.py start_thruster_manager:=false
```

### Generic Simulator Launch

A single `simulator.launch.py` runs either the GPU or the headless build,
selected by the `gpu:=true|false` argument.

#### GPU-Accelerated (Recommended)

```bash
ros2 launch stonefish_ros2 simulator.launch.py \
    simulation_data:=$(ros2 pkg prefix stonefish_description)/share/stonefish_description \
    scenario_desc:=$(ros2 pkg prefix stonefish_description)/share/stonefish_description/scenarios/bluerov2_seabed.scn \
    simulation_rate:=100.0 \
    rendering_quality:=high
```

#### No-GPU (Headless)

```bash
ros2 launch stonefish_ros2 simulator.launch.py gpu:=false \
    scenario_desc:=$(ros2 pkg prefix stonefish_description)/share/stonefish_description/scenarios/bluerov2_empty.scn \
    simulation_rate:=100.0
```

## Launch Files

### bluerov2.launch.py

Complete BlueROV2 simulation with optional thruster manager.

**Arguments**:

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `vehicle_name` | string | `bluerov2` | Vehicle namespace |
| `scenario` | string | `bluerov2_empty` | Scenario name (without .scn) |
| `start_thruster_manager` | bool | `true` | Launch thruster manager |
| `simulation_rate` | float | `100.0` | Simulation update rate (Hz) |

**Includes**:
- Stonefish simulator (GPU)
- Thruster manager (optional)
- Static TF: `base_link` → `base_link_frd` (FRD frame correction)

**Example**:
```bash
# Seabed scenario with thruster manager
ros2 launch stonefish_ros2 bluerov2.launch.py \
    scenario:=bluerov2_seabed \
    simulation_rate:=200.0
```

### simulator.launch.py

Stonefish simulator (leaf launch). The `gpu` argument selects the rendered GPU
build (`gpu:=true`, default) or the headless build (`gpu:=false`). The window /
rendering arguments apply to the GPU build only. Replaces the former
`simulator_gpu.launch.py` / `simulator_nogpu.launch.py` pair.

**Arguments**:

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `simulation_data` | string | (required) | Package share directory path |
| `scenario_desc` | string | (required) | Absolute path to scenario file |
| `simulation_rate` | float | `100.0` | Simulation update rate (Hz) |
| `gpu` | bool | `true` | `true` = rendered GPU build, `false` = headless build |
| `window_res_x` | int | `960` | Window width (pixels, GPU build only) |
| `window_res_y` | int | `1056` | Window height (pixels, GPU build only) |
| `rendering_quality` | string | `high` | Rendering quality: low/medium/high (GPU build only) |

**Example (GPU)**:
```bash
ros2 launch stonefish_ros2 simulator.launch.py \
    simulation_data:=$(ros2 pkg prefix stonefish_description)/share/stonefish_description \
    scenario_desc:=$(ros2 pkg prefix stonefish_description)/share/stonefish_description/scenarios/bluerov2_infrastructure.scn \
    rendering_quality:=medium \
    window_res_x:=1920 \
    window_res_y:=1080
```

**Example (headless)**:
```bash
ros2 launch stonefish_ros2 simulator.launch.py gpu:=false \
    scenario_desc:=$(ros2 pkg prefix stonefish_description)/share/stonefish_description/scenarios/bluerov2_empty.scn \
    simulation_rate:=500.0
```

### blueboat.launch.py

Surface vehicle (BlueBoat) simulation.

**Arguments**: Similar to `bluerov2.launch.py`

**Example**:
```bash
ros2 launch stonefish_ros2 blueboat.launch.py scenario:=blueboat_sea
```

## Topics

### Published Topics (Example: BlueROV2)

#### Robot State

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/bluerov2/odometry` | `nav_msgs/Odometry` | 100 Hz | Robot pose and velocity in world_ned frame |
| `/bluerov2/thrusters` | `stonefish_msgs/ThrusterState` | 10 Hz | Thruster setpoints, RPM, thrust, torque |
| `/bluerov2/servos` | `sensor_msgs/JointState` | 10 Hz | Servo joint positions and velocities |

#### Sensors

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/bluerov2/imu` | `sensor_msgs/Imu` | 100 Hz | IMU acceleration and angular velocity |
| `/bluerov2/dvl` | `stonefish_msgs/DVL` | 10 Hz | DVL velocity and altitude |
| `/bluerov2/dvl/altitude` | `sensor_msgs/Range` | 10 Hz | DVL altitude only |
| `/bluerov2/ins` | `stonefish_msgs/INS` | 100 Hz | Inertial navigation (GPS + IMU fusion) |
| `/bluerov2/ins/odometry` | `nav_msgs/Odometry` | 100 Hz | INS as odometry message |
| `/bluerov2/pressure` | `sensor_msgs/FluidPressure` | 10 Hz | Pressure sensor (depth) |
| `/bluerov2/gps` | `sensor_msgs/NavSatFix` | 1 Hz | GPS position (surface only) |

#### Cameras

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/bluerov2/camera_front/image_color` | `sensor_msgs/Image` | 30 Hz | Front camera RGB image |
| `/bluerov2/camera_front/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | Camera calibration |
| `/bluerov2/camera_depth/image_depth` | `sensor_msgs/Image` | 30 Hz | Depth camera (range image) |

#### Sonar

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/bluerov2/fls/image` | `sensor_msgs/Image` | 10 Hz | Forward-looking sonar raw data |
| `/bluerov2/fls/display` | `sensor_msgs/Image` | 10 Hz | FLS processed display image |
| `/bluerov2/sss` | `sensor_msgs/PointCloud2` | 5 Hz | Side-scan sonar point cloud |
| `/bluerov2/multibeam` | `sensor_msgs/LaserScan` | 10 Hz | Multibeam sonar scan |
| `/bluerov2/multibeam/pcl` | `sensor_msgs/PointCloud2` | 10 Hz | Multibeam point cloud |

#### Communication

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/bluerov2/usbl` | `sensor_msgs/NavSatFix` | 1 Hz | USBL position estimate |
| `/bluerov2/usbl/beacon_info` | `stonefish_msgs/BeaconInfo` | 1 Hz | USBL beacon ranging data |

### Subscribed Topics (Example: BlueROV2)

| Topic | Type | Description |
|-------|------|-------------|
| `/bluerov2/setpoint/pwm` | `std_msgs/Float64MultiArray` | Thruster PWM commands [-1.0, 1.0] |
| `/bluerov2/servo/setpoint` | `std_msgs/Float64` | Individual servo setpoint |

**Note**: Actuator topics are dynamically generated based on scenario file.

## Services

### Global Simulation Services

| Service | Type | Description |
|---------|------|-------------|
| `/stonefish_ros2/stonefish_simulator/set_ocean_current` | `stonefish_msgs/srv/SetOceanCurrent` | Set ocean current velocity (NED frame) |
| `/stonefish_ros2/stonefish_simulator/set_wave_height` | `stonefish_msgs/srv/SetWaveHeight` | Adjust ocean wave height at runtime |
| `/stonefish_ros2/stonefish_simulator/set_wind_velocity` | `stonefish_msgs/srv/SetWindVelocity` | Set atmospheric wind velocity (NED) |
| `/enable_currents` | `std_srvs/srv/Trigger` | Enable all ocean currents |
| `/disable_currents` | `std_srvs/srv/Trigger` | Disable all ocean currents |

### Sensor Configuration Services

| Service | Type | Description |
|---------|------|-------------|
| `/bluerov2/fls/settings` | `stonefish_msgs/srv/SonarSettings` | Configure FLS range and gain |
| `/bluerov2/sss/settings` | `stonefish_msgs/srv/SonarSettings2` | Configure SSS range, rotation, gain |
| `/bluerov2/msis/settings` | `stonefish_msgs/srv/SonarSettings2` | Configure MSIS parameters |

### Actuator Control Services

| Service | Type | Description |
|---------|------|-------------|
| `/bluerov2/thruster_0/enable` | `std_srvs/srv/SetBool` | Enable/disable individual thruster |
| `/bluerov2/servo_0/enable` | `std_srvs/srv/SetBool` | Enable/disable servo |

**Note**: Actuator service names depend on scenario configuration.

## Service Usage Examples

### Set Ocean Current

```bash
# Enable 0.5 m/s northward current
ros2 service call /stonefish_ros2/stonefish_simulator/set_ocean_current \
    stonefish_msgs/srv/SetOceanCurrent \
    "{current_index: 0, enable: true, velocity: [0.5, 0.0, 0.0]}"

# Enable 0.3 m/s eastward current (index 1)
ros2 service call /stonefish_ros2/stonefish_simulator/set_ocean_current \
    stonefish_msgs/srv/SetOceanCurrent \
    "{current_index: 1, enable: true, velocity: [0.0, 0.3, 0.0]}"

# Disable current
ros2 service call /stonefish_ros2/stonefish_simulator/set_ocean_current \
    stonefish_msgs/srv/SetOceanCurrent \
    "{current_index: 0, enable: false, velocity: [0.0, 0.0, 0.0]}"
```

### Set Wave Height

```bash
# Set 2-meter waves
ros2 service call /stonefish_ros2/stonefish_simulator/set_wave_height \
    stonefish_msgs/srv/SetWaveHeight "{height: 2.0}"

# Calm sea
ros2 service call /stonefish_ros2/stonefish_simulator/set_wave_height \
    stonefish_msgs/srv/SetWaveHeight "{height: 0.0}"
```

### Set Wind Velocity

```bash
# 5 m/s wind from North-East
ros2 service call /stonefish_ros2/stonefish_simulator/set_wind_velocity \
    stonefish_msgs/srv/SetWindVelocity "{x: 3.5, y: 3.5, z: 0.0}"

# 10 m/s headwind (from North)
ros2 service call /stonefish_ros2/stonefish_simulator/set_wind_velocity \
    stonefish_msgs/srv/SetWindVelocity "{x: 10.0, y: 0.0, z: 0.0}"
```

### Configure Sonar

```bash
# FLS: 50m range, 0.8 gain
ros2 service call /bluerov2/fls/settings \
    stonefish_msgs/srv/SonarSettings \
    "{range_min: 0.5, range_max: 50.0, gain: 0.8}"

# SSS: 100m range, ±90° rotation, 0.7 gain
ros2 service call /bluerov2/sss/settings \
    stonefish_msgs/srv/SonarSettings2 \
    "{range_min: 1.0, range_max: 100.0, rotation_min: -1.57, rotation_max: 1.57, gain: 0.7}"
```

### Enable/Disable Actuators

```bash
# Disable thruster 0
ros2 service call /bluerov2/thruster_0/enable \
    std_srvs/srv/SetBool "{data: false}"

# Re-enable thruster 0
ros2 service call /bluerov2/thruster_0/enable \
    std_srvs/srv/SetBool "{data: true}"
```

## Parameters

### Simulator Node Parameters

The simulator node reads parameters from the scenario XML file. No ROS2 parameters are exposed directly (scenario-driven configuration).

### Coordinate System Parameters

**Global Frame**: `world_ned` (NED - North-East-Down)
- Defined in scenario: `data/worlds/common/ned.scn`
- Positive Z = underwater
- Roll-Pitch-Yaw: Intrinsic ZYX Euler angles

**Robot Frame**: `base_link` (FRD - Forward-Right-Down)
- X: Forward
- Y: Right (starboard)
- Z: Down

## Python API Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from stonefish_msgs.srv import SetOceanCurrent

class ROVController(Node):
    def __init__(self):
        super().__init__('rov_controller')

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/bluerov2/odometry',
            self.odom_callback,
            10
        )

        # Publishers
        self.thrust_pub = self.create_publisher(
            Float64MultiArray,
            '/bluerov2/setpoint/pwm',
            10
        )

        # Service clients
        self.current_client = self.create_client(
            SetOceanCurrent,
            '/stonefish_ros2/stonefish_simulator/set_ocean_current'
        )

    def odom_callback(self, msg):
        """Process odometry feedback."""
        pos = msg.pose.pose.position
        self.get_logger().info(
            f'Position (NED): [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]'
        )

    def send_thrust(self, thrust_values):
        """Send thruster commands (8 thrusters for BlueROV2)."""
        msg = Float64MultiArray()
        msg.data = thrust_values  # List of 8 values [-1.0, 1.0]
        self.thrust_pub.publish(msg)

    def enable_current(self, velocity):
        """Enable ocean current with specified velocity."""
        request = SetOceanCurrent.Request()
        request.current_index = 0
        request.enable = True
        request.velocity = velocity  # [vx, vy, vz] in NED

        future = self.current_client.call_async(request)
        return future

def main():
    rclpy.init()
    controller = ROVController()

    # Enable northward current
    future = controller.enable_current([0.5, 0.0, 0.0])
    rclpy.spin_until_future_complete(controller, future)

    if future.result().success:
        controller.get_logger().info('Current enabled')

    # Send forward thrust
    controller.send_thrust([0.3, 0.3, 0.3, 0.3, 0.0, 0.0, 0.0, 0.0])

    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## C++ API Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <stonefish_msgs/srv/set_ocean_current.hpp>

class ROVController : public rclcpp::Node {
public:
    ROVController() : Node("rov_controller") {
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/bluerov2/odometry", 10,
            std::bind(&ROVController::odom_callback, this, std::placeholders::_1)
        );

        // Publishers
        thrust_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/bluerov2/setpoint/pwm", 10
        );

        // Service clients
        current_client_ = this->create_client<stonefish_msgs::srv::SetOceanCurrent>(
            "/stonefish_ros2/stonefish_simulator/set_ocean_current"
        );
    }

    void send_thrust(const std::vector<double>& thrust) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = thrust;
        thrust_pub_->publish(msg);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Position: [%.2f, %.2f, %.2f]",
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_pub_;
    rclcpp::Client<stonefish_msgs::srv::SetOceanCurrent>::SharedPtr current_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<ROVController>();

    // Send forward thrust
    controller->send_thrust({0.3, 0.3, 0.3, 0.3, 0.0, 0.0, 0.0, 0.0});

    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
```

## Package Structure

```
stonefish_ros2/
├── include/stonefish_ros2/        # C++ headers
│   ├── ROS2Interface.h            # ROS2 message interface
│   ├── ROS2ScenarioParser.h       # XML scenario parser
│   └── ROS2SimulationManager.h    # Simulation manager
├── src/stonefish_ros2/            # C++ source
│   ├── ROS2Interface.cpp
│   ├── ROS2ScenarioParser.cpp
│   ├── ROS2SimulationManager.cpp
│   └── ROSSimulatorApp.cpp        # Main simulator application
├── launch/                        # Launch files
│   ├── bringup.launch.py          # Top-level: simulator + control
│   ├── bluerov2.launch.py         # Vehicle wrapper (BlueROV2)
│   ├── blueboat.launch.py         # Vehicle wrapper (BlueBoat)
│   ├── vehicle.launch.py          # Parameterized vehicle bringup
│   └── simulator.launch.py        # Simulator leaf (gpu:=true|false)
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Coordinate Systems

### World Frame: world_ned

- **Origin**: Defined in scenario (typically water surface)
- **X-axis**: North (forward)
- **Y-axis**: East (right)
- **Z-axis**: Down (underwater)

**Position Examples**:
- Surface: `[0, 0, 0]`
- 2m depth: `[0, 0, 2]`
- 5m North, 3m East, 10m depth: `[5, 3, 10]`

### Body Frame: base_link (FRD)

- **X-axis**: Forward (surge)
- **Y-axis**: Right/Starboard (sway)
- **Z-axis**: Down (heave)

### Frame Corrections

The `bluerov2.launch.py` publishes a static TF: `base_link` → `base_link_frd`

**Reason**: Stonefish's internal `base_link` orientation differs from standard FRD convention. The static TF corrects this for external consumers.

**Quaternion**: `[qx=0.5, qy=-0.5, qz=0.5, qw=-0.5]`

## Troubleshooting

### Simulator Fails to Start

**Error**: `Cannot load mesh: data/robots/...`

**Solution**: Ensure `simulation_data` argument points to package share directory:
```bash
ros2 launch stonefish_ros2 simulator.launch.py \
    simulation_data:=$(ros2 pkg prefix stonefish_description)/share/stonefish_description \
    scenario_desc:=...
```

### No Topics Published

**Issue**: Simulator running but no ROS2 topics

**Solution**:
1. Check scenario file includes robot definition
2. Verify robot namespace matches expected topics
3. Use `ros2 topic list` to see available topics
4. Check simulator logs for errors

### Rendering Window Blank

**Issue**: GPU simulator shows black screen

**Solution**:
1. Check GPU drivers and OpenGL support
2. Try lower rendering quality: `rendering_quality:=low`
3. Use no-GPU simulator for headless operation

### Service Call Fails

**Error**: `Service not available`

**Solution**:
1. Wait for simulator to fully initialize (~5 seconds)
2. Verify service name:
   ```bash
   ros2 service list | grep set_ocean_current
   ```
3. Check simulator logs for service registration

### Ocean Current Not Working

**Issue**: Current service succeeds but no effect

**Solution**:
1. Verify current is enabled in scenario XML
2. Check current magnitude is significant (> 0.1 m/s)
3. Ensure robot has hydrodynamic damping coefficients

## Related Packages

- **stonefish_msgs**: Message and service definitions
- **stonefish_description**: Robot models and scenarios
- **stonefish_thruster_manager**: Thrust allocation for control
- **stonefish_control**: Controllers for autonomous operation
- **stonefish_slam**: Sonar-based SLAM using simulator sensors

## References

### Documentation

- **Stonefish Library**: https://stonefish.readthedocs.io
- **ROS2 Humble**: https://docs.ros.org/en/humble/
- **Original Paper**: Cieślak, P. (2019). "Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface". *OCEANS 2019 - Marseille*.

### Citation

If you use this software in your research, please cite:

```bibtex
@inproceedings{stonefish,
   author = {Cie{\'s}lak, Patryk},
   booktitle = {OCEANS 2019 - Marseille},
   title = {{Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface}},
   month = jun,
   year = {2019},
   doi={10.1109/OCEANSE.2019.8867434}
}
```

### Support

For paid support on simulation setup, 3D modeling, custom sensors, or feature development, contact the original author at [patryk.cieslak@udg.edu](mailto:patryk.cieslak@udg.edu).

## Credits

This software was written and is continuously developed by **Patryk Cieślak**.

ROS2 port and enhancements by the HERO Lab, POSTECH.

## License

GPL v3.0 (consistent with Stonefish library)
