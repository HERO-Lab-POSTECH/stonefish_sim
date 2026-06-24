# stonefish_description

Resource package containing robot models, world environments, and simulation scenarios for the Stonefish marine robotics simulator.

## Overview

This package serves as a central repository for all physical descriptions, 3D models, and simulation configurations used with Stonefish. It follows a modular architecture where complex scenarios are built by composing reusable components (worlds, robots, objects) using XML includes.

## Features

- **Modular Scenario System**: Compose simulations from reusable XML components
- **Robot Models**: BlueROV2, BlueBoat, and legacy platforms
- **World Environments**: Empty, seabed, infrastructure, shipwreck, caig house
- **Object Library**: Environmental objects (gas canisters, oil drums, pipes, etc.)
- **NED Coordinate System**: Consistent North-East-Down convention
- **Material Definitions**: Realistic physical properties (density, friction, etc.)
- **Visual Assets**: Textures, materials, and appearance definitions

## Package Structure

```
stonefish_description/
├── data/                       # All simulation data
│   ├── robots/                 # Robot definitions
│   │   ├── bluerov2/          # BlueROV2 Heavy (8 thrusters)
│   │   │   ├── bluerov2.scn   # Robot scenario file
│   │   │   ├── meshes/        # 3D models (OBJ, textures)
│   │   │   └── config/        # Configuration files
│   │   │       └── TAM.yaml   # Thruster Allocation Matrix
│   │   ├── blueboat/          # BlueBoat surface vehicle
│   │   └── _legacy/           # Old robot definitions (girona500, etc.)
│   │
│   ├── worlds/                # World environments
│   │   ├── common/            # Shared configurations
│   │   │   ├── ned.scn        # NED coordinate setup
│   │   │   ├── ocean.scn      # Ocean physics
│   │   │   ├── atmosphere.scn # Atmospheric conditions
│   │   │   ├── materials.scn  # Material definitions
│   │   │   └── looks.scn      # Visual appearances
│   │   ├── world_empty.scn    # Empty ocean environment
│   │   ├── world_seabed.scn   # Ocean floor with terrain
│   │   ├── world_infrastructure.scn  # Underwater structures
│   │   └── meshes/            # Terrain and environment meshes
│   │
│   └── models/                # Reusable objects
│       ├── gas_canister/
│       ├── gas_tank/
│       ├── oil_drum/
│       └── rust_pipe/
│
├── scenarios/                 # Ready-to-run scenarios
│   ├── bluerov2_empty.scn     # BlueROV2 in empty ocean
│   ├── bluerov2_seabed.scn    # BlueROV2 over seabed
│   ├── bluerov2_infrastructure.scn
│   └── blueboat_sea.scn       # BlueBoat surface vehicle
│
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Coordinate Systems

### NED Convention (North-East-Down)

All scenarios use the **NED** coordinate system:

- **World Frame**: `world_ned`
  - **X-axis**: North (forward)
  - **Y-axis**: East (right)
  - **Z-axis**: Down (underwater)

- **Body Frame**: `base_link` (FRD - Forward-Right-Down)
  - **X-axis**: Forward
  - **Y-axis**: Right (starboard)
  - **Z-axis**: Down

**Important Notes**:
- Positive Z values indicate underwater positions
- Surface: `z = 0.0`
- 2 meters depth: `z = 2.0`
- 10 meters depth: `z = 10.0`

**RViz Compatibility**: A static transform is published between `world` (ENU) and `world_ned` (NED) for visualization compatibility.

### Example Positions

```xml
<!-- Spawn at surface -->
<arg name="xyz" value="0.0 0.0 0.0"/>

<!-- Spawn 2 meters underwater -->
<arg name="xyz" value="0.0 0.0 2.0"/>

<!-- Spawn 5m North, 3m East, 10m depth -->
<arg name="xyz" value="5.0 3.0 10.0"/>
```

## Available Robots

### BlueROV2 Heavy

8-thruster underwater ROV with full 6DOF control.

**Location**: `data/robots/bluerov2/`

**Specifications**:
- Mass: ~20 kg (including thrusters)
- Thrusters: 8 × T200 (4 horizontal + 4 vertical)
- Sensors: IMU, DVL, pressure sensor, cameras
- Control: Full 6DOF (surge, sway, heave, roll, pitch, yaw)

**Configuration Files**:
- `bluerov2.scn`: Robot definition
- `config/TAM.yaml`: Thruster Allocation Matrix (6×8)
- `meshes/`: 3D models and textures

**Usage**:
```xml
<include file="data/robots/bluerov2/bluerov2.scn">
    <arg name="vehicle_name" value="bluerov2"/>
    <arg name="xyz" value="0.0 0.0 2.0"/>
    <arg name="rpy" value="0.0 0.0 0.0"/>
</include>
```

### BlueBoat

Surface vehicle for testing USV operations.

**Location**: `data/robots/blueboat/`

**Usage**:
```xml
<include file="data/robots/blueboat/blueboat.scn">
    <arg name="vehicle_name" value="blueboat"/>
    <arg name="xyz" value="0.0 0.0 0.0"/>
    <arg name="rpy" value="0.0 0.0 0.0"/>
</include>
```

### Legacy Robots

Older robot definitions (Girona500, Sonobot, etc.) are available in `data/robots/_legacy/` but may require updates for ROS2 compatibility.

## Available Worlds

### world_empty.scn

Empty ocean environment with basic physics.

**Includes**:
- NED coordinate system
- Ocean physics (water properties)
- Atmosphere (air properties)
- Materials and visual looks

**Usage**: Clean testing environment without obstacles

### world_seabed.scn

Ocean floor with realistic terrain.

**Features**:
- Textured seabed mesh
- Ocean physics
- Ambient lighting
- Suitable for DVL testing and bottom-following

### world_infrastructure.scn

Underwater structures and obstacles.

**Contains**:
- Pipes, platforms, or structures
- Testing environment for navigation and collision avoidance

### world_shipwreck.scn / world_caighouse.scn

Complex environments with large structures.

**Usage**: Advanced navigation, SLAM testing, object detection

## Creating Custom Scenarios

### Basic Scenario Template

```xml
<?xml version="1.0"?>
<scenario>
    <!-- Include base world environment -->
    <include file="data/worlds/world_empty.scn"/>

    <!-- Include robot -->
    <include file="data/robots/bluerov2/bluerov2.scn">
        <arg name="vehicle_name" value="bluerov2"/>
        <arg name="xyz" value="0.0 0.0 2.0"/>
        <arg name="rpy" value="0.0 0.0 0.0"/>
    </include>

    <!-- Optional: Add objects -->
    <include file="data/models/gas_canister/gas_canister.scn">
        <arg name="xyz" value="5.0 0.0 10.0"/>
    </include>
</scenario>
```

### Scenario Arguments

Robot includes accept these arguments:

| Argument | Type | Description |
|----------|------|-------------|
| `vehicle_name` | string | Robot namespace (used in topic names) |
| `xyz` | float[3] | Position in world_ned frame [N, E, D] |
| `rpy` | float[3] | Orientation [roll, pitch, yaw] in radians |

### Multi-Robot Scenarios

```xml
<?xml version="1.0"?>
<scenario>
    <include file="data/worlds/world_seabed.scn"/>

    <!-- Robot 1 -->
    <include file="data/robots/bluerov2/bluerov2.scn">
        <arg name="vehicle_name" value="bluerov2_alpha"/>
        <arg name="xyz" value="0.0 0.0 2.0"/>
        <arg name="rpy" value="0.0 0.0 0.0"/>
    </include>

    <!-- Robot 2 -->
    <include file="data/robots/bluerov2/bluerov2.scn">
        <arg name="vehicle_name" value="bluerov2_beta"/>
        <arg name="xyz" value="10.0 10.0 2.0"/>
        <arg name="rpy" value="0.0 0.0 1.57"/>
    </include>
</scenario>
```

Each robot will have its own topic namespace:
- `/bluerov2_alpha/odometry`
- `/bluerov2_beta/odometry`

## XML Scenario Syntax

### Include Directive

```xml
<include file="relative/path/to/file.scn">
    <arg name="arg_name" value="arg_value"/>
</include>
```

**Notes**:
- Paths are relative to package share directory
- Arguments are substituted using `$(arg arg_name)` syntax
- Nested includes are supported

### Common Elements

#### Materials

```xml
<materials>
    <material name="Fiberglass" density="1500.0" restitution="0.5"/>
    <material name="Steel" density="7850.0" restitution="0.3"/>
</materials>
```

#### Visual Looks

```xml
<looks>
    <look name="blue" rgb="0.0 0.5 1.0" roughness="0.3"/>
    <look name="textured" gray="1.0" roughness="0.5"
          texture="data/path/to/texture.png"/>
</looks>
```

#### Static Objects

```xml
<static name="seabed" type="plane">
    <material name="Rock"/>
    <look name="sand"/>
    <world_transform rpy="0.0 0.0 0.0" xyz="0.0 0.0 50.0"/>
</static>
```

## Adding New Robots

### Directory Structure

```bash
mkdir -p data/robots/my_robot/meshes
mkdir -p data/robots/my_robot/config
```

### Required Files

1. **Robot Definition** (`my_robot.scn`):

```xml
<?xml version="1.0"?>
<scenario>
    <looks>
        <look name="my_robot_color" rgb="0.2 0.8 0.3" roughness="0.4"/>
    </looks>

    <robot name="$(arg vehicle_name)" fixed="false">
        <base_link name="base_link" type="compound" physics="submerged">
            <external_part name="Hull" type="model" physics="submerged">
                <physical>
                    <mesh filename="data/robots/my_robot/meshes/hull_phy.obj" scale="1"/>
                    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
                </physical>
                <visual>
                    <mesh filename="data/robots/my_robot/meshes/hull_visual.obj" scale="1"/>
                    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
                </visual>
                <material name="Fiberglass"/>
                <look name="my_robot_color"/>
            </external_part>

            <!-- Add inertia, mass, etc. -->
            <inertia xyz="0.0 0.0 0.0"/>
            <mass value="15.0"/>

            <!-- Add sensors, thrusters, etc. -->
        </base_link>
    </robot>
</scenario>
```

2. **3D Meshes** (`meshes/`):
   - `hull_phy.obj`: Collision mesh (simplified)
   - `hull_visual.obj`: Visual mesh (detailed)
   - Texture files (PNG/JPG)

3. **TAM Configuration** (`config/TAM.yaml`):

```yaml
tam:
  - [t1_fx, t2_fx, t3_fx, t4_fx]  # Row 0: X force
  - [t1_fy, t2_fy, t3_fy, t4_fy]  # Row 1: Y force
  - [t1_fz, t2_fz, t3_fz, t4_fz]  # Row 2: Z force
  - [t1_tx, t2_tx, t3_tx, t4_tx]  # Row 3: Roll torque
  - [t1_ty, t2_ty, t3_ty, t4_ty]  # Row 4: Pitch torque
  - [t1_tz, t2_tz, t3_tz, t4_tz]  # Row 5: Yaw torque
```

### Testing New Robot

```bash
# Create test scenario
cat > scenarios/my_robot_test.scn << 'EOF'
<?xml version="1.0"?>
<scenario>
    <include file="data/worlds/world_empty.scn"/>
    <include file="data/robots/my_robot/my_robot.scn">
        <arg name="vehicle_name" value="my_robot"/>
        <arg name="xyz" value="0.0 0.0 2.0"/>
        <arg name="rpy" value="0.0 0.0 0.0"/>
    </include>
</scenario>
EOF

# Launch simulation
ros2 launch stonefish_ros2 simulator.launch.py \
    scenario_desc:=$(ros2 pkg prefix stonefish_description)/share/stonefish_description/scenarios/my_robot_test.scn
```

## Installation

This is a resource package (no compilation required).

```bash
cd /workspace/colcon_ws
colcon build --packages-select stonefish_description
source install/setup.bash
```

### Package Installation

Files are installed to:
```
install/stonefish_description/share/stonefish_description/
├── data/
├── scenarios/
└── package.xml
```

### Accessing Resources in Launch Files

```python
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# Get package share directory
pkg_share = FindPackageShare('stonefish_description')

# Build scenario path
scenario_path = PathJoinSubstitution([
    pkg_share,
    'scenarios',
    'bluerov2_empty.scn'
])

# Build data directory path
data_dir = PathJoinSubstitution([pkg_share, ''])
```

## Usage Examples

### Launch with Default Scenario

```bash
ros2 launch stonefish_ros2 bluerov2.launch.py
```

### Launch with Custom Scenario

```bash
ros2 launch stonefish_ros2 simulator.launch.py \
    scenario_desc:=$(ros2 pkg prefix stonefish_description)/share/stonefish_description/scenarios/bluerov2_seabed.scn
```

### Specify Simulation Data Directory

```bash
ros2 launch stonefish_ros2 simulator.launch.py \
    simulation_data:=$(ros2 pkg prefix stonefish_description)/share/stonefish_description \
    scenario_desc:=$(ros2 pkg prefix stonefish_description)/share/stonefish_description/scenarios/bluerov2_infrastructure.scn
```

### Python Launch API

```python
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = FindPackageShare('stonefish_description')

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('stonefish_ros2'),
            '/launch/simulator.launch.py'
        ]),
        launch_arguments={
            'simulation_data': PathJoinSubstitution([pkg_share, '']),
            'scenario_desc': PathJoinSubstitution([
                pkg_share, 'scenarios', 'bluerov2_seabed.scn'
            ]),
        }.items()
    )

    return LaunchDescription([simulator])
```

## Mesh File Guidelines

### Physical Meshes (Collision)

- **Format**: OBJ
- **Polygon Count**: Low (< 1000 triangles)
- **Convexity**: Prefer convex or simple decomposition
- **Purpose**: Physics simulation, collision detection

### Visual Meshes

- **Format**: OBJ with MTL material file
- **Polygon Count**: Medium (1000-10000 triangles)
- **Textures**: PNG or JPG (power-of-2 dimensions recommended)
- **Purpose**: Rendering only

### Best Practices

1. **Separate Physical and Visual**: Use simplified meshes for physics
2. **Origin Alignment**: Place origin at robot center of mass
3. **Scale**: Use real-world units (meters)
4. **Coordinate System**: Align with FRD convention
5. **Texture Paths**: Use relative paths from package share directory

## Environmental Objects

Available objects in `data/models/`:

| Object | Description | Use Case |
|--------|-------------|----------|
| `gas_canister` | Industrial gas cylinder | Obstacle, manipulation target |
| `gas_tank` | Large storage tank | Environmental clutter |
| `oil_drum` | Oil barrel | Floating/sinking objects |
| `rust_pipe` | Corroded pipe | Infrastructure inspection |

### Adding Objects to Scenarios

```xml
<include file="data/models/gas_canister/gas_canister.scn">
    <arg name="xyz" value="10.0 5.0 10.0"/>
    <arg name="rpy" value="0.0 0.0 0.0"/>
</include>
```

## Troubleshooting

### Mesh Not Found

**Error**: `Cannot load mesh: data/robots/my_robot/meshes/hull.obj`

**Solution**: Ensure paths are relative to package share directory:
```xml
<!-- Correct -->
<mesh filename="data/robots/my_robot/meshes/hull.obj"/>

<!-- Incorrect -->
<mesh filename="/workspace/.../hull.obj"/>
```

### Robot Sinking/Floating

**Issue**: Incorrect buoyancy configuration

**Solution**: Check mass vs. volume:
```xml
<!-- Total mass should balance displaced water volume -->
<mass value="20.0"/>  <!-- kg -->

<!-- For neutral buoyancy: mass ≈ volume × 1000 kg/m³ -->
```

### Texture Not Displaying

**Issue**: Texture path or format incorrect

**Solution**:
1. Use relative paths: `data/robots/my_robot/meshes/texture.png`
2. Ensure texture is power-of-2 (256×256, 512×512, etc.)
3. Check MTL file references correct texture name

### Spawn Position Above Water

**Issue**: Robot spawns in air and falls

**Solution**: Set positive Z value for underwater:
```xml
<!-- Surface -->
<arg name="xyz" value="0.0 0.0 0.0"/>

<!-- 2m underwater (correct) -->
<arg name="xyz" value="0.0 0.0 2.0"/>
```

## Related Packages

- **stonefish_ros2**: Simulator interface (loads scenarios from this package)
- **stonefish_msgs**: Message definitions for sensor data
- **stonefish_thruster_manager**: Uses TAM.yaml for thrust allocation
- **stonefish_control**: Controllers for robots defined here

## References

- **Stonefish Documentation**: https://stonefish.readthedocs.io
- **Stonefish GitHub**: https://github.com/patrykcieslak/stonefish
- **BlueROV2 Specs**: https://bluerobotics.com/store/rov/bluerov2/

## License

GPL v3.0 (consistent with Stonefish library)
