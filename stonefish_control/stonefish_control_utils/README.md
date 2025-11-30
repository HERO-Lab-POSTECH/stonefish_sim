# stonefish_control_utils

Utilities for PID controller tuning and optimization.

## Overview

This package provides automated PID tuning tools using optimization algorithms. It supports multiple optimization methods (GWO, SMAC3) and cost functions (ITSE, ITAE) to find optimal PID gains for underwater vehicle controllers.

**Key Features**:
- **Automated PID tuning**: No manual trial-and-error required
- **Multiple optimization methods**: Grey Wolf Optimizer (GWO), SMAC3
- **Cost functions**: ITSE (Integral Time-weighted Squared Error), ITAE (Integral Time-weighted Absolute Error)
- **Experiment tracking**: WandB integration for logging and visualization
- **Validation**: Automatic validation on test trajectories
- **Results export**: YAML files with optimal gains

---

## Installation

### Build from Source

```bash
cd /workspace/colcon_ws
colcon build --packages-select stonefish_control_utils
source install/setup.bash
```

### Dependencies

**ROS2 Packages**:
- `stonefish_control`
- `stonefish_control_msgs`
- `nav_msgs`

**Python Libraries**:
```bash
pip install numpy scipy pyyaml
pip install smac                  # For SMAC3 optimizer
pip install wandb                 # Optional: for experiment tracking
pip install transforms3d
```

---

## Optimization Methods

### 1. Grey Wolf Optimizer (GWO)

Bio-inspired metaheuristic optimization algorithm.

**How it works**:
- Simulates wolf pack hunting behavior
- 3 leadership levels: Alpha (best), Beta (2nd), Delta (3rd)
- Converges to optimal solution through pack dynamics

**Pros**:
- Good exploration of search space
- Relatively fast convergence
- No gradient information needed

**Cons**:
- Stochastic (results vary between runs)
- May require many iterations

**Use when**: You need robust exploration and don't have a good initial guess.

---

### 2. SMAC3

Sequential Model-based Algorithm Configuration (Bayesian optimization).

**How it works**:
- Builds surrogate model of cost function
- Selects next candidate using Expected Improvement
- Balances exploration vs exploitation

**Pros**:
- Sample-efficient (fewer evaluations)
- Handles noisy objectives
- Provides uncertainty estimates

**Cons**:
- Slower per iteration (model training)
- Requires more memory

**Use when**: You want sample efficiency and have computational budget constraints.

---

## Cost Functions

### ITSE (Integral Time-weighted Squared Error)

```
cost = ∫ t × e²(t) dt
```

**Characteristics**:
- Penalizes long settling times heavily
- Prioritizes fast convergence
- More sensitive to large errors

**Use when**: Fast response is critical, even if overshoot occurs.

---

### ITAE (Integral Time-weighted Absolute Error)

```
cost = ∫ t × |e(t)| dt
```

**Characteristics**:
- Balanced penalty on settling time
- Less sensitive to overshoot
- Smoother control signals

**Use when**: You want balanced performance with minimal overshoot.

---

## Usage

### Configuration File

Create a YAML config file defining optimization parameters:

```yaml
# pid_optimizer_config.yaml

optimizer:
  method: 'gwo'              # 'gwo' or 'smac3'
  cost_function: 'itae'      # 'itse' or 'itae'
  n_iterations: 30           # Number of optimization iterations
  population_size: 10        # GWO: wolves per iteration, SMAC3: ignored

bounds:
  # PID gain search ranges [min, max] for each DOF
  kp_min: [50.0, 50.0, 100.0, 50.0]    # [surge, sway, heave, yaw]
  kp_max: [500.0, 500.0, 600.0, 300.0]
  kd_min: [0.0, 50.0, 50.0, 30.0]
  kd_max: [300.0, 300.0, 400.0, 150.0]
  ki_min: [0.0, 0.0, 0.0, 0.0]
  ki_max: [100.0, 100.0, 150.0, 50.0]

scenario:
  vehicle_name: 'bluerov2'
  controller_namespace: 'position_controller'
  waypoint_file: '/path/to/waypoints.yaml'
  evaluation_time: 60.0      # Seconds to evaluate each candidate
  settling_threshold: 0.1    # Position error threshold (m)

wandb:
  enabled: true
  project: 'bluerov2_pid_tuning'
  entity: 'your_username'
  run_name_prefix: 'gwo_itae'

results:
  save_directory: '/workspace/results/pid_tuning'
```

---

### Launch File

```bash
ros2 launch stonefish_control_utils pid_optimizer.launch.py \
    config_file:=/path/to/pid_optimizer_config.yaml
```

---

### Full Workflow

**Step 1**: Prepare configuration

```bash
cd /workspace/colcon_ws/src/stonefish_control_utils/config
cp example_config.yaml my_tuning.yaml
# Edit my_tuning.yaml with your parameters
```

**Step 2**: Start simulator

```bash
# Terminal 1
ros2 launch stonefish_ros2 bluerov2.launch.py
```

**Step 3**: Start controller (will be updated by optimizer)

```bash
# Terminal 2
ros2 launch stonefish_control controller.launch.py controller_type:=position
```

**Step 4**: Run optimizer

```bash
# Terminal 3
ros2 launch stonefish_control_utils pid_optimizer.launch.py \
    config_file:=/workspace/config/my_tuning.yaml
```

**Step 5**: Monitor progress

- Watch terminal output for iteration progress
- If WandB enabled: Check dashboard at https://wandb.ai

**Step 6**: Apply optimal gains

After optimization completes, optimal gains are saved to:
```
/workspace/results/pid_tuning/<timestamp>/optimal_gains.yaml
```

Apply them:
```bash
ros2 service call /bluerov2/position_controller/set_pid_params \
    stonefish_control_msgs/srv/SetPIDParams \
    "$(cat /workspace/results/pid_tuning/<timestamp>/optimal_gains.yaml)"
```

---

## Parameters

### Optimizer Configuration

| Parameter | Type | Description |
|-----------|------|-------------|
| `optimizer.method` | string | Optimization method: `gwo` or `smac3` |
| `optimizer.cost_function` | string | Cost function: `itse` or `itae` |
| `optimizer.n_iterations` | int | Number of optimization iterations |
| `optimizer.population_size` | int | (GWO only) Number of candidates per iteration |

### Search Bounds

| Parameter | Type | Description |
|-----------|------|-------------|
| `bounds.kp_min` | float[4] | Minimum Kp values [surge, sway, heave, yaw] |
| `bounds.kp_max` | float[4] | Maximum Kp values |
| `bounds.kd_min` | float[4] | Minimum Kd values |
| `bounds.kd_max` | float[4] | Maximum Kd values |
| `bounds.ki_min` | float[4] | Minimum Ki values |
| `bounds.ki_max` | float[4] | Maximum Ki values |

### Scenario Settings

| Parameter | Type | Description |
|-----------|------|-------------|
| `scenario.vehicle_name` | string | Vehicle namespace |
| `scenario.controller_namespace` | string | Controller node namespace |
| `scenario.waypoint_file` | string | Path to waypoint YAML for evaluation |
| `scenario.evaluation_time` | float | Simulation time per candidate (seconds) |
| `scenario.settling_threshold` | float | Position error threshold for settling time |

### WandB Integration

| Parameter | Type | Description |
|-----------|------|-------------|
| `wandb.enabled` | bool | Enable WandB logging |
| `wandb.project` | string | WandB project name |
| `wandb.entity` | string | WandB username/team |
| `wandb.run_name_prefix` | string | Prefix for run name (timestamp appended) |

---

## Output

### Results Directory Structure

```
/workspace/results/pid_tuning/
└── 20250130_153022/
    ├── optimal_gains.yaml       # Best PID gains found
    ├── optimization_history.csv # All evaluated candidates
    ├── validation_results.yaml  # Performance on test trajectory
    └── config.yaml              # Copy of optimization config
```

### optimal_gains.yaml

```yaml
kp: [350.0, 320.0, 450.0, 210.0]
kd: [180.0, 165.0, 230.0, 110.0]
ki: [12.0, 11.0, 25.0, 6.0]
kb: [0.8, 0.8, 0.8, 0.8]

cost: 125.3
cost_function: 'itae'
method: 'gwo'
timestamp: '2025-01-30 15:42:18'
```

### optimization_history.csv

```csv
iteration,kp_surge,kp_sway,kp_heave,kp_yaw,kd_surge,...,cost
1,300.0,300.0,400.0,200.0,150.0,...,156.2
2,320.0,310.0,420.0,205.0,160.0,...,142.8
...
```

---

## Python API

### Using Optimizer Programmatically

```python
#!/usr/bin/env python3
from stonefish_control_utils.pid_optimizer.gwo import GWOOptimizer
from stonefish_control_utils.pid_optimizer.base import load_yaml_config

# Load config
config = load_yaml_config('/path/to/config.yaml')

# Create optimizer
optimizer = GWOOptimizer(config)

# Run optimization
best_gains, best_cost = optimizer.optimize()

# Print results
print(f"Optimal gains: {best_gains}")
print(f"Cost: {best_cost}")

# Save results
optimizer.save_results(best_gains, best_cost, '/path/to/results/')
```

---

## Tuning Strategies

### Quick Tuning (Small Search Space)

For fast iteration during development:

```yaml
optimizer:
  method: 'gwo'
  n_iterations: 20
  population_size: 8

bounds:
  # Narrow search around existing gains
  kp_min: [250.0, 250.0, 350.0, 180.0]
  kp_max: [350.0, 350.0, 450.0, 220.0]
  # ... etc
```

**Time**: ~30-60 minutes

---

### Thorough Tuning (Large Search Space)

For final production tuning:

```yaml
optimizer:
  method: 'smac3'
  n_iterations: 50

bounds:
  # Wide search
  kp_min: [50.0, 50.0, 100.0, 50.0]
  kp_max: [600.0, 600.0, 700.0, 350.0]
  # ... etc
```

**Time**: 2-4 hours

---

### Multi-Objective Tuning

Optimize for different cost functions and compare:

1. Run ITAE optimization (smooth control)
2. Run ITSE optimization (fast response)
3. Manually test both on your scenarios
4. Choose based on mission requirements

---

## Troubleshooting

### Optimizer diverges (cost increases)

**Causes**:
- Search bounds too wide
- Evaluation time too short
- Controller saturation

**Solutions**:
- Narrow search bounds
- Increase `evaluation_time`
- Check controller saturation logs

---

### No improvement after many iterations

**Causes**:
- Already near optimal
- Local minimum trap (GWO)
- Search bounds exclude optimal region

**Solutions**:
- Try SMAC3 instead of GWO
- Expand search bounds
- Check if current gains are already good

---

### Simulation crashes during optimization

**Causes**:
- Unstable gains evaluated
- Simulator timeout
- Memory leak

**Solutions**:
- Add stability constraints to bounds
- Restart simulator between iterations
- Monitor system resources

---

## Package Structure

```
stonefish_control_utils/
├── stonefish_control_utils/
│   ├── pid_optimizer/
│   │   ├── base.py           # Base optimizer class
│   │   ├── gwo.py            # Grey Wolf Optimizer
│   │   ├── smac3.py          # SMAC3 optimizer
│   │   └── node.py           # ROS2 node wrapper
│   └── pid_tools/
│       ├── analyzer.py       # Error analysis tools
│       ├── logger.py         # Data logging
│       ├── plotter.py        # Visualization
│       └── regulator.py      # PID utilities
├── launch/
│   └── pid_optimizer.launch.py
├── config/
│   └── example_config.yaml   # Example configuration
├── package.xml
├── setup.py
└── README.md
```

---

## Related Packages

- **stonefish_control**: Controllers to be tuned
- **stonefish_control_msgs**: Service interfaces for PID updates
- **stonefish_trajectory_manager**: Test trajectories for evaluation

---

## References

### Optimization Algorithms

- Mirjalili, S. et al. (2014). "Grey Wolf Optimizer". Advances in Engineering Software.
- Hutter, F. et al. (2011). "Sequential Model-Based Optimization for General Algorithm Configuration". LION 2011.

### Control Theory

- Åström, K.J., & Hägglund, T. (2006). "Advanced PID Control"
- Ziegler, J.G., & Nichols, N.B. (1942). "Optimum Settings for Automatic Controllers"

---

## License

Apache 2.0

---

## Notes

### Optimization Time Estimates

| Configuration | Method | Iterations | Population | Est. Time |
|---------------|--------|------------|------------|-----------|
| Quick | GWO | 20 | 8 | 30-60 min |
| Standard | GWO | 30 | 10 | 1-2 hours |
| Thorough | SMAC3 | 50 | - | 2-4 hours |

**Factors affecting time**:
- `evaluation_time`: Longer = more accurate but slower
- Trajectory complexity: More waypoints = longer evaluation
- Hardware: Faster CPU = faster simulation

### WandB Integration

If WandB is enabled, you can:
- Monitor optimization progress in real-time
- Compare multiple runs
- Visualize cost function landscape
- Share results with team

**Setup**:
```bash
pip install wandb
wandb login
# Follow prompts to authenticate
```

### Parallelization

Currently, optimization is **sequential** (one candidate at a time). Future versions may support parallel evaluation for faster tuning.
