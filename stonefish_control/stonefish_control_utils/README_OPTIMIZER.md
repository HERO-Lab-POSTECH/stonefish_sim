# PID Optimizer Launch Guide

## Overview

The PID optimizer automatically tunes PID gains for the ROV controller using optimization algorithms (GWO or SMAC3).

## Launch File

### Basic Usage

```bash
# Launch with default settings (GWO optimizer, BlueROV2)
ros2 launch stonefish_control_utils pid_optimizer.launch.py
```

### Launch Arguments

- **`vehicle_name`** (default: `bluerov2`)
  Vehicle namespace to optimize

- **`optimizer_config`** (default: `gwo_optimizer.yaml`)
  Optimizer configuration file in `config/optimizer/` directory
  - Options: `gwo_optimizer.yaml`, `smac3_optimizer.yaml`

- **`controller_config`** (default: `bluerov2/pid_params.yaml`)
  PID controller configuration file in `stonefish_control/config/` directory

### Examples

```bash
# Use SMAC3 optimizer instead of GWO
ros2 launch stonefish_control_utils pid_optimizer.launch.py \
    optimizer_config:=smac3_optimizer.yaml

# Optimize a different vehicle
ros2 launch stonefish_control_utils pid_optimizer.launch.py \
    vehicle_name:=my_robot \
    controller_config:=my_robot/pid_params.yaml
```

## What Gets Launched

The launch file starts:

1. **PID Controller** (`/bluerov2/pid_controller`)
   - Loaded from `stonefish_control/launch/pid_control.launch.py`
   - Uses specified controller config file

2. **PID Optimizer** (`/pid_optimizer`)
   - Runs optimization algorithm (GWO or SMAC3)
   - Evaluates PID gains by running test scenarios
   - Updates controller gains via ROS2 service calls

**Note:** The thruster allocator is NOT launched by this file. You must launch it separately via `stonefish_ros2` (e.g., in your simulation launch file).

## Optimization Configuration

Edit the config files to customize optimization:

### GWO Config (`config/optimizer/gwo_optimizer.yaml`)
```yaml
optimizer:
  method: "gwo"
  n_wolves: 50              # Population size
  max_iterations: 50        # Number of iterations
  test_duration: 10.0       # Evaluation duration per test (seconds)

  scenarios:                # Test scenarios for evaluation
    - name: "forward_step"
      type: "step"
      target: {x: 2.0, y: 0.0, z: -1.0, roll: 0.0, pitch: 0.0, yaw: 0.0}
    # ... more scenarios
```

### SMAC3 Config (`config/optimizer/smac3_optimizer.yaml`)
```yaml
optimizer:
  method: "smac3"
  n_trials: 100             # Number of evaluations
  # ... similar structure to GWO
```

## Workflow

1. **Start simulation with thruster allocator:**
   ```bash
   ros2 launch stonefish_ros2 bluerov2.launch.py
   ```

2. **Start optimizer (in separate terminal):**
   ```bash
   ros2 launch stonefish_control_utils pid_optimizer.launch.py
   ```

3. **Monitor progress:**
   - Console output shows optimization progress
   - Results saved to `config/optimizer/output/gwo/` or `config/optimizer/output/smac3/`
   - WandB dashboard (if enabled in config)

4. **Apply optimized gains:**
   - Best gains automatically saved to results directory
   - Copy to controller config file or use warm start in next optimization

## Results

Results are saved to:
- **GWO:** `config/optimizer/output/gwo/`
- **SMAC3:** `config/optimizer/output/smac3/`

Files include:
- `best_gains.yaml` - Optimized PID gains
- `optimization_history.csv` - Iteration history
- `final_report.txt` - Summary statistics
- WandB logs (if enabled)

## Troubleshooting

**Optimizer fails to start:**
- Ensure simulation is running first
- Check that thruster allocator is active
- Verify vehicle namespace matches

**Poor optimization results:**
- Increase `test_duration` for more stable evaluation
- Adjust search bounds in config file
- Add more diverse test scenarios

**Build errors:**
- Rebuild package: `colcon build --packages-select stonefish_control_utils`
- Source workspace: `source install/setup.bash`
