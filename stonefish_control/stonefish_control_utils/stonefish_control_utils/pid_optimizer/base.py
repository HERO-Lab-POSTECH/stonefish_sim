#!/usr/bin/env python3
"""
Base Optimizer for PID Tuning

Provides common functionality for GWO and SMAC3 optimizers.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from abc import ABC, abstractmethod

import numpy as np
import time
import yaml
import subprocess
import os
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple, Optional

from stonefish_control_msgs.srv import SetPIDParams, GetPIDParams, ResetTrajectory, ResetController
from stonefish_control_msgs.msg import TrajectoryPoint
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg  # ROS message (avoid conflict with pathlib.Path)
from geometry_msgs.msg import WrenchStamped, Point, Quaternion, Vector3
from transforms3d.euler import quat2euler

# WandB for experiment tracking
try:
    import wandb
    WANDB_AVAILABLE = True
except ImportError:
    WANDB_AVAILABLE = False


def create_wandb_run_name(prefix: str, method: str) -> str:
    """Generate WandB run name with timestamp

    Args:
        prefix: User-defined prefix (e.g., "gwo_itae")
        method: Optimization method (e.g., "gwo", "smac3")

    Returns:
        Run name in format:
        - If prefix: "{prefix}_{timestamp}"
        - If no prefix: "{method}_{timestamp}"

    Examples:
        >>> create_wandb_run_name("gwo_itae", "gwo")
        'gwo_itae_20250107_143022'
        >>> create_wandb_run_name("", "gwo")
        'gwo_20250107_143022'
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if prefix:
        return f"{prefix}_{timestamp}"
    else:
        return f"{method}_{timestamp}"


def load_yaml_config(file_path: str) -> dict:
    """Load YAML configuration file

    Args:
        file_path: Path to YAML config file

    Returns:
        Dictionary with configuration
    """
    with open(file_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def create_results_directory(base_dir: str) -> Path:
    """Create timestamped results directory path (without creating the directory)

    Args:
        base_dir: Base directory path

    Returns:
        Path to results directory (not yet created)
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_dir = Path(base_dir) / timestamp
    # Don't create directory here - let save_results() create it when needed
    return results_dir


class BaseOptimizer(Node, ABC):
    """Abstract base class for PID optimizers

    Provides common functionality:
    - ROS communication (services, subscribers)
    - Data collection (odometry, errors, wrench)
    - Scenario management
    - Cost calculation (ITSE/ITAE)
    - Validation
    - Results saving
    """

    def __init__(self, config: dict, node_name: str):
        """Initialize base optimizer

        Args:
            config: Configuration dictionary from YAML
            node_name: Name for ROS node
        """
        super().__init__(node_name)

        # Store full config
        self.config = config
        opt_config = config['optimizer']

        # Common parameters
        self.method = opt_config['method']
        self.test_duration = opt_config['test_duration']
        self.vehicle_name = opt_config['vehicle_name']
        self.controller_type = opt_config.get('controller_type', 'single').lower()  # 'single' or 'cascaded'
        self.interpolation_method = opt_config.get('interpolation_method', 'lipb')  # Path interpolation method
        self.cost_function = opt_config['cost_function'].lower()
        self.use_wandb = opt_config['use_wandb']
        self.wandb_project = opt_config['wandb_project']
        self.wandb_run_name_prefix = opt_config['wandb_run_name_prefix']

        # Precomputed trajectory durations (calculated once at startup)
        # Key: scenario name, Value: test duration (seconds)
        # This ensures all evaluations use the SAME duration for fair comparison
        self._trajectory_durations = {}
        self.wandb_dir = opt_config.get('wandb_dir', None)  # Optional wandb directory

        # Weights for step response cost
        weights_config = opt_config['weights']
        self.weights = np.array([
            weights_config['x'],
            weights_config['y'],
            weights_config['z'],
            weights_config['roll'],
            weights_config['pitch'],
            weights_config['yaw']
        ])
        self.lambda_smoothness = opt_config.get('lambda_smoothness', 0.1)
        self.lambda_gain = opt_config.get('lambda_gain', 0.0)  # Gain penalty weight (0 = disabled)

        # Weights for trajectory cost (multi-objective) - Hybrid Approach
        # Priority 1 (CRITICAL - 85%): Path tracking (RMS only) + Completion
        # Priority 2 (IMPORTANT - 13%): Attitude + Heading stability
        # Priority 3 (NICE-TO-HAVE - 2%): Smoothness + Acceleration
        # Ref: Wiley IET 2021, MDPI AUV 2023 - Use RMS (not MAE+RMS+Max)
        # Ref: MPC literature (Mathworks) - Tracking 10-20x higher than control
        self.lambda_cte_rmse = opt_config.get('lambda_cte_rmse',
                                               opt_config.get('lambda_rms_cte', 200.0))  # P1: CTE RMSE (Perpendicular)
        self.lambda_completion = opt_config.get('lambda_completion', 300.0)   # P1: Path completion (arc-length based)
        self.lambda_rms_attitude = opt_config.get('lambda_rms_attitude', 30.0)  # P2: Roll/pitch RMS
        self.lambda_rms_heading = opt_config.get('lambda_rms_heading', 30.0)  # P2: Heading RMS
        self.lambda_linear_accel = opt_config.get('lambda_linear_accel', 10.0)  # P3: Linear acceleration RMS
        self.lambda_angular_accel = opt_config.get('lambda_angular_accel', 10.0)  # P3: Angular acceleration RMS
        self.lambda_trajectory_smoothness = opt_config.get('lambda_trajectory_smoothness', 20.0)  # P3: Control smoothness RMS

        # Expected maximum values for normalization (Ref: Mathworks MPC Tuning Guide)
        # Normalizes metrics to [0, ~1] range for proper weight balance
        expected_max_config = opt_config.get('expected_max', {})
        # CTE expected max: Fixed value based on lookahead distance (path-length independent)
        self.expected_max_cte_rmse = expected_max_config.get('cte_rmse',
                                                              expected_max_config.get('rms_cte', 0.5))  # 0.5m (lookahead/2)
        # Path completion expected max: 1.0 (100% uncompleted = total failure)
        self.expected_max_completion = expected_max_config.get('completion', 1.0)
        # Attitude/Heading: Tighter tolerance for better stability (10° instead of 30°)
        self.expected_max_attitude = expected_max_config.get('attitude', 0.175)  # 10° ≈ 0.175 rad
        self.expected_max_heading = expected_max_config.get('heading', 0.175)  # 10° heading error
        self.expected_max_linear_accel = expected_max_config.get('linear_accel', 1.0)  # 1 m/s²
        self.expected_max_angular_accel = expected_max_config.get('angular_accel', 10.0)  # 10 rad/s²
        self.expected_max_smoothness = expected_max_config.get('smoothness', 50.0)  # 50N wrench rate (tighter)

        # Results directory (with timestamp)
        self.results_base_dir = opt_config['results_dir']
        self.results_dir = create_results_directory(self.results_base_dir)

        # Validate cost function
        if self.cost_function not in ['itse', 'itae']:
            raise ValueError(f'Invalid cost_function: {self.cost_function}. Must be "itse" or "itae"')

        # Validate controller type
        if self.controller_type not in ['single', 'cascaded']:
            raise ValueError(f'Invalid controller_type: {self.controller_type}. Must be "single" or "cascaded"')

        # PID dimension (depends on controller type)
        # Single: 18 parameters (6 DOF × 3 gains)
        # Cascaded: 36 parameters (position 18 + velocity 18)
        self.dim = 36 if self.controller_type == 'cascaded' else 18

        # Parse bounds
        self.lb, self.ub = self._parse_bounds(opt_config['bounds'])

        # Warm start
        self.warm_start_file = opt_config.get('warm_start_file', '')
        self.warm_start_gains = None
        if self.warm_start_file:
            self.warm_start_gains = self._load_warm_start_gains(self.warm_start_file)

        # Checkpoint settings (with timestamp)
        self.checkpoint_file = opt_config.get('checkpoint_file', '')
        self.checkpoint_interval = opt_config.get('checkpoint_interval', 1)

        # Add timestamp to checkpoint directory (same as results_dir)
        checkpoint_base_dir = opt_config.get('checkpoint_dir', '')
        if checkpoint_base_dir:
            # If checkpoint_dir is specified, add timestamp subfolder
            self.checkpoint_dir = create_results_directory(checkpoint_base_dir)
        else:
            # Default: results_dir/checkpoints (timestamp already in results_dir)
            self.checkpoint_dir = self.results_dir / 'checkpoints'
        # Don't create checkpoint directory here - let save_checkpoint() create it when needed

        # Load scenarios
        self.scenarios = self._load_scenarios(opt_config['scenarios'])

        # Data storage (must be initialized before _setup_ros_interfaces)
        self.current_pos = np.zeros(3)
        self.current_quat = np.array([0, 0, 0, 1])  # [x, y, z, w]
        self.current_rpy = np.zeros(3)  # [roll, pitch, yaw] from odometry
        self.current_vel = np.zeros(6)
        self.errors_pos = []
        self.errors_ori = []
        self.wrench_history = []
        self.collecting_data = False

        # Trajectory tracking data (for new cost function)
        self.gt_path = None  # Ground truth path from path_generator (positions only)
        self.gt_path_poses = None  # Full GT path with timestamps (for ATE calculation)
        self.actual_trajectory = None  # Actual robot trajectory
        self.odometry_history = []  # Full odometry history (for attitude ITAE and jerk calculation)
        self.desired_yaw_history = []  # Desired yaw from cmd_pose (LOS guidance output)

        # Track last published reference for debugging
        self.last_reference_pos = None
        self.last_reference_rpy = None

        # Interrupt handling
        self._interrupted = False

        # Initialize WandB
        self.wandb_run = None
        self.wandb_run_id = None
        self.wandb_run_name = None
        self._initialize_wandb()

        # Log configuration
        cost_func_str = f'{self.cost_function.upper()} + Smoothness'
        if self.lambda_gain > 0:
            cost_func_str += ' + Gain Penalty'
        self.get_logger().info(f'📊 Cost Function: {cost_func_str}')
        self.get_logger().info(f'📊 Weights: x={self.weights[0]}, y={self.weights[1]}, z={self.weights[2]}, '
                               f'roll={self.weights[3]}, pitch={self.weights[4]}, yaw={self.weights[5]}')
        self.get_logger().info(f'📊 Smoothness penalty weight (λ_smooth): {self.lambda_smoothness}')
        if self.lambda_gain > 0:
            self.get_logger().info(f'📊 Gain penalty weight (λ_gain): {self.lambda_gain}')
        self.get_logger().info(f'📁 Results directory: {self.results_dir}')
        self.get_logger().info(f'🎯 Loaded {len(self.scenarios)} scenarios')

        # Auto-launch path_following if trajectory scenario exists
        # IMPORTANT: Must be done BEFORE _setup_ros_interfaces() to ensure PID controller is running
        self._trajectory_process = None
        self._controller_process = None
        self._auto_launch_trajectory_control()

        # Launch PID controller (separate from path_following)
        self._auto_launch_controller()

        # Setup ROS interfaces (AFTER launching PID controller)
        self._setup_ros_interfaces()

        # Precompute trajectory durations for all scenarios (once at startup)
        self._precompute_trajectory_durations()

    def __del__(self):
        """Cleanup hierarchical_control subprocess on deletion"""
        self.cleanup()

    def cleanup(self):
        """Cleanup resources (trajectory subprocess, controller subprocess, and WandB)"""
        # Terminate trajectory subprocess
        if self._trajectory_process is not None:
            try:
                self.get_logger().info('🛑 Terminating path_following subprocess...')
                self._trajectory_process.terminate()
                self._trajectory_process.wait(timeout=5)
            except:
                try:
                    self._trajectory_process.kill()
                except:
                    pass

        # Terminate controller subprocess
        if self._controller_process is not None:
            try:
                self.get_logger().info('🛑 Terminating controller subprocess...')
                self._controller_process.terminate()
                self._controller_process.wait(timeout=5)
            except:
                try:
                    self._controller_process.kill()
                except:
                    pass

        # Finish WandB run
        if self.use_wandb and WANDB_AVAILABLE and self.wandb_run is not None:
            try:
                self.get_logger().info('📊 Finishing WandB run...')
                wandb.finish()
            except Exception as e:
                self.get_logger().warning(f'⚠️  Failed to finish WandB: {e}')

    def _auto_launch_trajectory_control(self):
        """Auto-launch path_following.launch.py if config contains trajectory scenario"""
        # Check if any scenario is trajectory type
        trajectory_scenarios = [s for s in self.scenarios if s.get('type') == 'trajectory']

        self.get_logger().info(f'🔍 Checking for trajectory scenarios...')
        self.get_logger().info(f'   Total scenarios: {len(self.scenarios)}')
        for s in self.scenarios:
            self.get_logger().info(f'   - {s.get("name")}: type={s.get("type")}')

        if not trajectory_scenarios:
            self.get_logger().info('   No trajectory scenarios found, skipping auto-launch')
            return  # No trajectory scenarios, nothing to launch

        self.get_logger().info(f'   Found {len(trajectory_scenarios)} trajectory scenario(s)')

        # Get waypoint file from first trajectory scenario
        trajectory_scenario = trajectory_scenarios[0]
        waypoint_file = trajectory_scenario.get('waypoint_file')

        if not waypoint_file:
            self.get_logger().error('❌ Trajectory scenario has no waypoint_file specified')
            return

        self.get_logger().info(f'📁 Waypoint file: {waypoint_file}')

        # Check if waypoint file exists
        if not os.path.exists(waypoint_file):
            self.get_logger().error(f'❌ Waypoint file not found: {waypoint_file}')
            return

        self.get_logger().info(f'✓ Waypoint file exists')

        # Launch path_following launch file (depends on controller type)
        # Single: path_following.launch.py (path generator + PID + LOS guidance)
        # Cascaded: path_following_cascaded.launch.py (path generator + cascaded PID + LOS guidance)
        # This launch file handles sequential startup with delays
        try:
            # Select launch file based on controller type
            if self.controller_type == 'cascaded':
                launch_file = 'path_following_cascaded.launch.py'
                controller_desc = 'cascaded PID'
            else:
                launch_file = 'path_following.launch.py'
                controller_desc = 'single PID'

            # Source ROS2 setup and launch path_following
            # Suppress all output to reduce clutter during optimization
            cmd = [
                'bash', '-c',
                f'source /workspace/colcon_ws/install/setup.bash && '
                f'ros2 launch stonefish_trajectory_manager {launch_file} '
                f'vehicle_name:={self.vehicle_name} '
                f'waypoint_file:={waypoint_file} '
                f'lookahead_distance:=2.0 '  # Updated to match current setting
                f'acceptance_radius:=0.5 '
                f'robot_max_speed:=1.029 '  # 2 knots (updated from 1.0)
                f'robot_min_speed:=0.772 '  # 1.5 knots (added parameter)
                f'max_lateral_accel:=0.3 '
                f'min_speed_factor:=0.75 '  # Updated from 0.3 to match 1.5 knots min
                f'max_velocity_magnitude:=1.5 '  # Added parameter for total velocity limit
                f'update_rate:=20.0 '
                f'interpolation_method:={self.interpolation_method} '
                f'start_pid:=false '  # PID already launched by optimizer
                f'> /dev/null 2>&1'  # Suppress all output
            ]

            self.get_logger().info(f'🚀 Launching {launch_file} subprocess ({controller_desc})...')
            self.get_logger().info(f'   (path generator + path following node, controller managed by optimizer)')

            # Run subprocess with suppressed output
            self._trajectory_process = subprocess.Popen(
                cmd,
                env=os.environ.copy()
            )

            self.get_logger().info(f'✅ Subprocess started (PID: {self._trajectory_process.pid})')

            # Wait for path_following to start
            # path generator (0s) → path following (4s) + buffer (no PID launch since start_pid:=false)
            self.get_logger().info('⏳ Waiting 5s for path_following system to initialize...')
            time.sleep(5.0)

            # Check if subprocess is still alive
            if self._trajectory_process.poll() is not None:
                self.get_logger().error(f'❌ Subprocess exited with code {self._trajectory_process.returncode}')
                self._trajectory_process = None
            else:
                self.get_logger().info('✓ Path following subprocess running (generator + LOS guidance)')

        except Exception as e:
            self.get_logger().error(f'❌ Failed to launch path_following: {e}')
            import traceback
            traceback.print_exc()
            self._trajectory_process = None

    def _auto_launch_controller(self):
        """Auto-launch PID controller based on controller_type

        Single: Launch pid_control_node
        Cascaded: Launch cascaded_control_node
        """
        try:
            if self.controller_type == 'cascaded':
                # Launch cascaded_control_node
                cmd = [
                    'bash', '-c',
                    f'source /workspace/colcon_ws/install/setup.bash && '
                    f'ros2 run stonefish_control cascaded_control_node '
                    f'--ros-args -r __ns:=/{self.vehicle_name} '
                    f'--params-file /workspace/colcon_ws/src/stonefish_control/stonefish_control/config/bluerov2/cascaded_params.yaml '
                    f'> /dev/null 2>&1'
                ]
                controller_name = 'cascaded_control_node'
            else:
                # Launch single PID controller
                cmd = [
                    'bash', '-c',
                    f'source /workspace/colcon_ws/install/setup.bash && '
                    f'ros2 launch stonefish_control pid_control.launch.py '
                    f'vehicle_name:={self.vehicle_name} '
                    f'> /dev/null 2>&1'
                ]
                controller_name = 'pid_control'

            self.get_logger().info(f'🚀 Launching {controller_name}...')

            self._controller_process = subprocess.Popen(
                cmd,
                env=os.environ.copy()
            )

            self.get_logger().info(f'✅ Controller subprocess started (PID: {self._controller_process.pid})')

            # Wait for controller to initialize
            self.get_logger().info('⏳ Waiting 3s for controller to initialize...')
            time.sleep(3.0)

            # Check if subprocess is still alive
            if self._controller_process.poll() is not None:
                self.get_logger().error(f'❌ Controller subprocess exited with code {self._controller_process.returncode}')
                self._controller_process = None
            else:
                self.get_logger().info(f'✓ {controller_name} running')

        except Exception as e:
            self.get_logger().error(f'❌ Failed to launch controller: {e}')
            import traceback
            traceback.print_exc()
            self._controller_process = None

    def set_interrupted(self):
        """Set interrupt flag to gracefully stop optimization"""
        self._interrupted = True

    def is_interrupted(self):
        """Check if optimization should stop"""
        return self._interrupted

    def _parse_bounds(self, bounds_config: dict) -> Tuple[np.ndarray, np.ndarray]:
        """Parse bounds from config

        Args:
            bounds_config: Bounds configuration dict

        Returns:
            (lb, ub): Lower and upper bounds as numpy arrays
        """
        dofs = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        gains = ['kp', 'kd', 'ki']

        lb = []
        ub = []

        if self.controller_type == 'cascaded':
            # Cascaded: separate bounds for position and velocity
            # Check if config has position/velocity structure
            if 'position' in bounds_config and 'velocity' in bounds_config:
                # New format: separate position and velocity bounds
                # Position bounds (18 params)
                for gain in gains:
                    for dof in dofs:
                        bounds = bounds_config['position'][dof][gain]
                        lb.append(bounds[0])
                        ub.append(bounds[1])

                # Velocity bounds (18 params)
                for gain in gains:
                    for dof in dofs:
                        bounds = bounds_config['velocity'][dof][gain]
                        lb.append(bounds[0])
                        ub.append(bounds[1])
            else:
                # Old format: duplicate bounds (backward compatibility)
                for gain in gains:
                    for dof in dofs:
                        bounds = bounds_config[dof][gain]
                        lb.append(bounds[0])
                        ub.append(bounds[1])

                # Duplicate for velocity
                lb = lb + lb
                ub = ub + ub
        else:
            # Single PID: standard bounds
            for gain in gains:
                for dof in dofs:
                    bounds = bounds_config[dof][gain]
                    lb.append(bounds[0])
                    ub.append(bounds[1])

        return np.array(lb), np.array(ub)

    def _load_scenarios(self, scenarios_config: List[dict]) -> List[dict]:
        """Load scenarios from config

        Args:
            scenarios_config: List of scenario configurations

        Returns:
            List of scenario dictionaries
        """
        scenarios = []

        for scenario_config in scenarios_config:
            scenario_type = scenario_config.get('type', 'step')

            if scenario_type == 'step':
                scenarios.append(scenario_config)
            elif scenario_type == 'trajectory':
                # Trajectory following scenario
                scenarios.append(scenario_config)
            elif scenario_type == 'random':
                # Generate random scenarios
                count = scenario_config['count']
                ranges = scenario_config['ranges']
                for i in range(count):
                    random_scenario = self._generate_random_scenario(ranges, seed=i)
                    scenarios.append(random_scenario)

        return scenarios

    def _generate_random_scenario(self, ranges: dict, seed: Optional[int] = None) -> dict:
        """Generate random test scenario

        Args:
            ranges: Dictionary with ranges for each DOF
            seed: Random seed for reproducibility

        Returns:
            Scenario dictionary
        """
        if seed is not None:
            np.random.seed(seed)

        target = {}
        for dof, (min_val, max_val) in ranges.items():
            target[dof] = np.random.uniform(min_val, max_val)

        return {
            'name': f'random_{seed}' if seed is not None else 'random',
            'type': 'step',
            'target': target
        }

    def _setup_ros_interfaces(self):
        """Setup ROS clients, publishers, and subscribers"""
        # Service clients (controller-type specific)
        if self.controller_type == 'cascaded':
            # Cascaded controller has separate services for position and velocity
            self.set_pid_position_client = self.create_client(
                SetPIDParams, f'/{self.vehicle_name}/cascaded_position/set_pid_params')
            self.set_pid_velocity_client = self.create_client(
                SetPIDParams, f'/{self.vehicle_name}/cascaded_velocity/set_pid_params')
            self.get_pid_position_client = self.create_client(
                GetPIDParams, f'/{self.vehicle_name}/cascaded_position/get_pid_params')
            self.get_pid_velocity_client = self.create_client(
                GetPIDParams, f'/{self.vehicle_name}/cascaded_velocity/get_pid_params')
            self.reset_controller_client = self.create_client(
                ResetController, f'/{self.vehicle_name}/cascaded/reset_all')
        else:
            # Single PID controller
            self.set_pid_client = self.create_client(
                SetPIDParams, f'/{self.vehicle_name}/set_pid_params')
            self.get_pid_client = self.create_client(
                GetPIDParams, f'/{self.vehicle_name}/get_pid_params')
            self.reset_controller_client = self.create_client(
                ResetController, f'/{self.vehicle_name}/reset_controller')

        # Common services
        self.reset_trajectory_client = self.create_client(
            ResetTrajectory, f'/{self.vehicle_name}/reset_trajectory')
        self.reset_path_generator_client = self.create_client(
            ResetTrajectory, '/path_generator_node/reset_path')

        # Publisher for reference (changed from service to topic)
        self.cmd_pose_pub = self.create_publisher(
            TrajectoryPoint,
            f'/{self.vehicle_name}/cmd_pose',
            10
        )

        # QoS profile for subscribers (prevents "A message was lost" warnings)
        # Use BEST_EFFORT for real-time data streams to avoid message loss warnings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100  # Increased from 10 to handle bursts
        )

        # Subscribers with improved QoS
        self.odom_sub = self.create_subscription(
            Odometry, f'/{self.vehicle_name}/odometry',
            self._odom_callback, qos_profile)

        # Subscribe to tracking_error (closest point based) if available,
        # otherwise use regular error (cmd_pose based)
        # For path following optimization, tracking_error is preferred
        self.error_sub = self.create_subscription(
            TrajectoryPoint, f'/{self.vehicle_name}/tracking_error',
            self._error_callback, qos_profile)

        # Fallback: Also subscribe to regular error for station-keeping scenarios
        self.error_sub_fallback = self.create_subscription(
            TrajectoryPoint, f'/{self.vehicle_name}/error',
            self._error_callback, qos_profile)

        self.wrench_sub = self.create_subscription(
            WrenchStamped, f'/{self.vehicle_name}/thruster_manager/input_stamped',
            self._wrench_callback, qos_profile)

        # Trajectory tracking subscribers (for new cost function)
        self.gt_path_sub = self.create_subscription(
            PathMsg, '/path_generator_node/path',
            self._gt_path_callback, qos_profile)

        self.actual_trajectory_sub = self.create_subscription(
            PathMsg, f'/{self.vehicle_name}/actual_trajectory',
            self._actual_trajectory_callback, qos_profile)

        # Subscribe to cmd_pose (desired yaw from LOS guidance)
        self.cmd_pose_sub = self.create_subscription(
            TrajectoryPoint, f'/{self.vehicle_name}/cmd_pose',
            self._cmd_pose_callback, qos_profile)

        # Wait for services
        self.get_logger().info(f'⏳ Waiting for {self.controller_type} controller services...')
        if self.controller_type == 'cascaded':
            self.set_pid_position_client.wait_for_service(timeout_sec=10.0)
            self.set_pid_velocity_client.wait_for_service(timeout_sec=10.0)
            self.get_logger().info('✅ Connected to cascaded controller services')
        else:
            self.set_pid_client.wait_for_service(timeout_sec=10.0)
            self.get_pid_client.wait_for_service(timeout_sec=10.0)
            self.get_logger().info('✅ Connected to single PID controller services')

    def _initialize_wandb(self):
        """Initialize WandB logging"""
        if not self.use_wandb:
            return

        if not WANDB_AVAILABLE:
            self.get_logger().warning('⚠️  WandB not available. Install with: pip install wandb')
            self.get_logger().warning('   Continuing without WandB logging')
            self.use_wandb = False
            return

        try:
            # Finish any active run first (prevents "another run is active" error)
            if wandb.run is not None:
                self.get_logger().warning('⚠️  Found active WandB run, finishing it first')
                wandb.finish()

            # Create WandB config
            wandb_config = {
                'optimizer': self.method.upper(),
                'test_duration': self.test_duration,
                'vehicle': self.vehicle_name,
                'cost_function': self.cost_function,
                'weights': self.weights.tolist(),
                'lambda_smoothness': self.lambda_smoothness,
                'n_scenarios': len(self.scenarios),
            }

            # Initialize WandB with optional directory
            init_kwargs = {
                'project': self.wandb_project,
                'config': wandb_config
            }
            if self.wandb_dir:
                init_kwargs['dir'] = self.wandb_dir

            # Start fresh run
            run_name = create_wandb_run_name(self.wandb_run_name_prefix, self.method)
            init_kwargs['name'] = run_name
            self.wandb_run = wandb.init(**init_kwargs)
            self.wandb_run_id = self.wandb_run.id
            self.wandb_run_name = run_name
            self.get_logger().info(f'📊 WandB initialized: {run_name} (ID: {self.wandb_run_id})')

            if self.wandb_dir:
                self.get_logger().info(f'📁 WandB logs: {self.wandb_dir}')
        except Exception as e:
            self.get_logger().error(f'❌ WandB initialization failed: {e}')
            self.use_wandb = False

    def _load_warm_start_gains(self, file_path: str) -> Optional[np.ndarray]:
        """Load warm start gains from YAML file

        Supports multiple formats:
        - New format: Kp, Kd, Ki (direct keys)
        - Old format: best_gains.kp, best_gains.kd, best_gains.ki
        - ROS param format: /**:/ros__parameters/Kp, Kd, Ki

        Args:
            file_path: Path to YAML file with gains

        Returns:
            Gains array or None if loading failed
        """
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)

            kp = None
            kd = None
            ki = None

            # Try new format (Kp, Kd, Ki)
            if 'Kp' in data and 'Kd' in data and 'Ki' in data:
                kp = data['Kp']
                kd = data['Kd']
                ki = data['Ki']
                self.get_logger().info('   Format: New format (Kp, Kd, Ki)')

            # Try old format (best_gains.kp, kd, ki)
            elif 'best_gains' in data:
                kp = data['best_gains']['kp']
                kd = data['best_gains']['kd']
                ki = data['best_gains']['ki']
                self.get_logger().info('   Format: Old format (best_gains)')

            # Try ROS parameter format (/**:/ros__parameters/...)
            elif '/**' in data and 'ros__parameters' in data['/**']:
                params = data['/**']['ros__parameters']
                kp = params['Kp']
                kd = params['Kd']
                ki = params['Ki']
                self.get_logger().info('   Format: ROS parameter format')

            else:
                raise ValueError('Unknown YAML format. Expected: Kp/Kd/Ki, best_gains, or /**:/ros__parameters')

            # Validate
            if len(kp) != 6 or len(kd) != 6 or len(ki) != 6:
                raise ValueError(f'Invalid gain lengths: Kp={len(kp)}, Kd={len(kd)}, Ki={len(ki)} (expected 6 each)')

            gains = np.array(kp + kd + ki)
            self.get_logger().info(f'🔥 Loaded warm start gains from {file_path}')
            return gains

        except Exception as e:
            self.get_logger().error(f'❌ Failed to load warm start gains: {e}')
            import traceback
            traceback.print_exc()
            return None

    # ============================================================================
    # ROS Callbacks
    # ============================================================================

    def _odom_callback(self, msg: Odometry):
        """Odometry callback"""
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.current_quat = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.current_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

        # Convert quaternion to euler for logging
        from transforms3d.euler import quat2euler
        quat_wxyz = [self.current_quat[3], self.current_quat[0], self.current_quat[1], self.current_quat[2]]
        self.current_rpy = np.array(quat2euler(quat_wxyz))

        # Record odometry for trajectory cost calculation
        if self.collecting_data:
            # Store: timestamp, position (3), rpy (3), velocity (6)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            odom_data = {
                'timestamp': timestamp,
                'position': self.current_pos.copy(),
                'rpy': self.current_rpy.copy(),
                'velocity': self.current_vel.copy()
            }
            self.odometry_history.append(odom_data)

    def _error_callback(self, msg: TrajectoryPoint):
        """Error callback - collect error data during test"""
        if not self.collecting_data:
            return

        # TrajectoryPoint uses pose.position for position data
        pos_error = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        # Convert quaternion to euler for orientation error
        from transforms3d.euler import quat2euler
        quat = msg.pose.orientation
        roll, pitch, yaw = quat2euler([quat.w, quat.x, quat.y, quat.z])
        ori_error = np.array([roll, pitch, yaw])

        self.errors_pos.append(pos_error)
        self.errors_ori.append(ori_error)

        # Just mark that we received first error (no logging here)

    def _wrench_callback(self, msg: WrenchStamped):
        """Wrench callback - collect wrench data for smoothness penalty"""
        if not self.collecting_data:
            return

        wrench = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ])
        self.wrench_history.append(wrench)

    def _gt_path_callback(self, msg: PathMsg):
        """Ground truth path callback (from path_generator_node)"""
        # Extract path as array of [x, y, z] positions
        self.gt_path = np.array([
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for pose in msg.poses
        ])

        # Store full poses with timestamps for ATE calculation
        self.gt_path_poses = []
        for pose_stamped in msg.poses:
            timestamp = pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9
            position = np.array([
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z
            ])
            self.gt_path_poses.append({
                'timestamp': timestamp,
                'position': position
            })

        self.get_logger().info(f'📍 GT path received: {len(self.gt_path)} points', once=True)

    def _actual_trajectory_callback(self, msg: PathMsg):
        """Actual trajectory callback (from path_following_node)"""
        if not self.collecting_data:
            return

        # Extract trajectory as array of [x, y, z] positions
        self.actual_trajectory = np.array([
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for pose in msg.poses
        ])

    def _cmd_pose_callback(self, msg: TrajectoryPoint):
        """CMD pose callback (desired state from LOS guidance)"""
        if not self.collecting_data:
            return

        # Extract desired yaw from orientation quaternion
        quat = msg.pose.orientation
        _, _, desired_yaw = quat2euler([quat.w, quat.x, quat.y, quat.z])

        # Record timestamp and desired yaw
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.desired_yaw_history.append({
            'timestamp': timestamp,
            'desired_yaw': desired_yaw
        })

    # ============================================================================
    # PID and Reference Management
    # ============================================================================

    def set_pid_gains(self, gains: np.ndarray) -> bool:
        """Set PID gains via service

        Args:
            gains: For single: 18-element array [kp(6), kd(6), ki(6)]
                   For cascaded: 36-element array [pos_kp(6), pos_kd(6), pos_ki(6),
                                                    vel_kp(6), vel_kd(6), vel_ki(6)]

        Returns:
            True if successful
        """
        try:
            if self.controller_type == 'cascaded':
                # Split gains into position (0:18) and velocity (18:36)
                pos_gains = gains[0:18]
                vel_gains = gains[18:36]

                # Position PID
                pos_Kp = pos_gains[0:6].tolist()
                pos_Kd = pos_gains[6:12].tolist()
                pos_Ki = pos_gains[12:18].tolist()

                # Velocity PID
                vel_Kp = vel_gains[0:6].tolist()
                vel_Kd = vel_gains[6:12].tolist()
                vel_Ki = vel_gains[12:18].tolist()

                self.get_logger().info(f'📝 Setting cascaded PID gains:')
                self.get_logger().info(f'   Position Kp: [{pos_Kp[0]:.1f}, {pos_Kp[1]:.1f}, {pos_Kp[2]:.1f}, ...]')
                self.get_logger().info(f'   Velocity Kp: [{vel_Kp[0]:.1f}, {vel_Kp[1]:.1f}, {vel_Kp[2]:.1f}, ...]')

                # Set position PID
                pos_request = SetPIDParams.Request()
                pos_request.kp = pos_Kp
                pos_request.kd = pos_Kd
                pos_request.ki = pos_Ki

                pos_future = self.set_pid_position_client.call_async(pos_request)

                start_wait = time.time()
                while not pos_future.done() and (time.time() - start_wait) < 5.0:
                    time.sleep(0.01)

                if not pos_future.done() or pos_future.result() is None:
                    self.get_logger().error('❌ Position PID service call timeout or failed')
                    return False

                if not pos_future.result().success:
                    self.get_logger().error('❌ Position PID service returned failure')
                    return False

                # Set velocity PID
                vel_request = SetPIDParams.Request()
                vel_request.kp = vel_Kp
                vel_request.kd = vel_Kd
                vel_request.ki = vel_Ki

                vel_future = self.set_pid_velocity_client.call_async(vel_request)

                start_wait = time.time()
                while not vel_future.done() and (time.time() - start_wait) < 5.0:
                    time.sleep(0.01)

                if not vel_future.done() or vel_future.result() is None:
                    self.get_logger().error('❌ Velocity PID service call timeout or failed')
                    return False

                return vel_future.result().success

            else:
                # Single PID controller
                Kp = gains[0:6].tolist()
                Kd = gains[6:12].tolist()
                Ki = gains[12:18].tolist()

                # Log the gains being set
                self.get_logger().info(f'📝 Setting PID gains:')
                self.get_logger().info(f'   Kp: [{Kp[0]:.1f}, {Kp[1]:.1f}, {Kp[2]:.1f}, {Kp[3]:.1f}, {Kp[4]:.1f}, {Kp[5]:.1f}]')
                self.get_logger().info(f'   Kd: [{Kd[0]:.1f}, {Kd[1]:.1f}, {Kd[2]:.1f}, {Kd[3]:.1f}, {Kd[4]:.1f}, {Kd[5]:.1f}]')
                self.get_logger().info(f'   Ki: [{Ki[0]:.3f}, {Ki[1]:.3f}, {Ki[2]:.3f}, {Ki[3]:.3f}, {Ki[4]:.3f}, {Ki[5]:.3f}]')

                request = SetPIDParams.Request()
                request.kp = Kp
                request.kd = Kd
                request.ki = Ki

                future = self.set_pid_client.call_async(request)

                start_wait = time.time()
                while not future.done() and (time.time() - start_wait) < 5.0:
                    time.sleep(0.01)

                if not future.done() or future.result() is None:
                    self.get_logger().error('❌ Service call timeout or failed')
                    return False

                return future.result().success

        except Exception as e:
            self.get_logger().error(f'❌ Exception in set_pid_gains: {e}')
            return False

    def publish_reference(self, scenario: dict) -> bool:
        """Publish reference via cmd_pose topic.

        Changed from service call to topic publishing for consistency.

        Args:
            scenario: Scenario dictionary with target pose

        Returns:
            True if successful
        """
        # Check if interrupted
        if self._interrupted:
            return False

        try:
            target = scenario['target']

            # Create TrajectoryPoint message
            from geometry_msgs.msg import Point, Quaternion, Vector3

            msg = TrajectoryPoint()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world_ned'

            # Position (World frame - NED)
            msg.pose.position = Point(
                x=float(target['x']),
                y=float(target['y']),
                z=float(target['z'])
            )

            # Orientation (6DOF)
            from transforms3d.euler import euler2quat
            quat_wxyz = euler2quat(
                target.get('roll', 0.0),
                target.get('pitch', 0.0),
                target['yaw']
            )
            msg.pose.orientation = Quaternion(
                x=float(quat_wxyz[1]),
                y=float(quat_wxyz[2]),
                z=float(quat_wxyz[3]),
                w=float(quat_wxyz[0])
            )

            # Velocity: zero (station-keeping for optimization)
            msg.velocity.linear = Vector3(x=0.0, y=0.0, z=0.0)
            msg.velocity.angular = Vector3(x=0.0, y=0.0, z=0.0)

            # Acceleration: zero
            msg.acceleration.linear = Vector3(x=0.0, y=0.0, z=0.0)
            msg.acceleration.angular = Vector3(x=0.0, y=0.0, z=0.0)

            # Publish
            self.cmd_pose_pub.publish(msg)

            # Store last reference for debugging
            self.last_reference_pos = np.array([target['x'], target['y'], target['z']])
            self.last_reference_rpy = np.array([
                target.get('roll', 0.0),
                target.get('pitch', 0.0),
                target['yaw']
            ])

            # Log only first publish to avoid spam (step scenarios publish repeatedly)
            if not hasattr(self, '_reference_logged') or self._reference_logged != scenario.get('name'):
                self._reference_logged = scenario.get('name')
                self.get_logger().info(
                    f'📍 Published reference: pos=[{target["x"]:.2f}, {target["y"]:.2f}, {target["z"]:.2f}], '
                    f'ori=[roll={target.get("roll", 0.0):.2f}, pitch={target.get("pitch", 0.0):.2f}, yaw={target["yaw"]:.2f}]'
                )

            # Wait a bit for message to propagate
            time.sleep(0.1)

            return True

        except Exception as e:
            self.get_logger().error(f'❌ Exception in publish_reference: {e}')
            return False

    def return_to_origin(self):
        """Return vehicle to origin"""
        origin_scenario = {
            'name': 'origin',
            'type': 'step',
            'target': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'roll': 0.0,
                'pitch': 0.0,
                'yaw': 0.0
            }
        }
        self.publish_reference(origin_scenario)
        time.sleep(5.0)  # Wait for settling

    def _calculate_trajectory_duration(self, waypoint_file: str,
                                       use_velocity_profiler: bool = True) -> float:
        """Calculate expected duration for trajectory completion.

        Args:
            waypoint_file: Path to waypoint YAML file
            use_velocity_profiler: Whether velocity profiler is enabled

        Returns:
            Expected duration in seconds
        """
        try:
            import yaml
            with open(waypoint_file, 'r') as f:
                wp_data = yaml.safe_load(f)

            waypoints = wp_data.get('waypoints', [])
            if len(waypoints) < 2:
                self.get_logger().warning('Trajectory has less than 2 waypoints, using default duration')
                return 10.0

            total_distance = 0.0
            for i in range(len(waypoints) - 1):
                wp1 = waypoints[i]
                wp2 = waypoints[i + 1]

                # Get positions (supports both formats: position array or x/y/z keys)
                if 'position' in wp1:
                    pos1 = np.array(wp1['position'])
                    pos2 = np.array(wp2['position'])
                else:
                    pos1 = np.array([wp1['x'], wp1['y'], wp1['z']])
                    pos2 = np.array([wp2['x'], wp2['y'], wp2['z']])

                # Calculate 3D distance between waypoints
                distance = np.linalg.norm(pos2 - pos1)
                total_distance += distance

            # Use max_forward_speed for duration estimate
            # This provides reasonable test duration without being overly conservative
            # Try top-level first, then fallback to first waypoint
            max_speed = wp_data.get('max_forward_speed', waypoints[0].get('max_forward_speed', 1.0))
            if max_speed <= 0:
                max_speed = 1.0

            # Min speed for reference
            min_speed = wp_data.get('min_forward_speed', waypoints[0].get('min_forward_speed', 0.1))

            # Path generator adds WP0 if robot is far from first waypoint
            # Using actual distance threshold from path_generator_node (0.5m default)
            # Add small buffer for smooth approach
            wp0_distance_estimate = 1.0  # meters (small buffer for WP0 if added)

            # Total distance with WP0 buffer
            total_distance_with_wp0 = total_distance + wp0_distance_estimate

            # Calculate duration based on MAX speed
            # Buffer will be added later to account for slowdowns at turns
            duration = total_distance_with_wp0 / max_speed

            self.get_logger().info(f'Trajectory duration calculated (max_speed based):')
            self.get_logger().info(f'  YAML waypoints: {total_distance:.2f}m')
            self.get_logger().info(f'  + WP0 buffer: {wp0_distance_estimate:.1f}m')
            self.get_logger().info(f'  = Total distance: {total_distance_with_wp0:.2f}m')
            self.get_logger().info(f'  Max speed: {max_speed:.2f}m/s | Min speed: {min_speed:.2f}m/s')
            self.get_logger().info(f'  Duration (max_speed): {total_distance_with_wp0:.2f}m / {max_speed:.2f}m/s = {duration:.1f}s')

            return duration

        except Exception as e:
            self.get_logger().error(f'Failed to calculate trajectory duration: {e}')
            return 10.0  # Default fallback

    def _precompute_trajectory_durations(self):
        """Precompute trajectory durations for all scenarios at startup.

        This ensures all evaluations use the SAME duration for fair comparison.
        Duration is calculated once and reused for all wolves/iterations.
        """
        self.get_logger().info('=' * 70)
        self.get_logger().info('📏 Pre-computing trajectory durations for all scenarios...')
        self.get_logger().info('=' * 70)

        buffer = 10.0  # seconds (time buffer for settling)

        for scenario in self.scenarios:
            scenario_type = scenario.get('type', 'step')

            if scenario_type == 'trajectory':
                waypoint_file = scenario.get('waypoint_file')
                if waypoint_file:
                    base_duration = self._calculate_trajectory_duration(waypoint_file, use_velocity_profiler=False)
                    test_duration = base_duration + buffer

                    self._trajectory_durations[scenario['name']] = test_duration

                    self.get_logger().info(f'✓ {scenario["name"]}: {test_duration:.1f}s (base: {base_duration:.1f}s + buffer: {buffer:.1f}s)')
                else:
                    self.get_logger().warning(f'⚠️  {scenario["name"]}: No waypoint file, using default')
                    self._trajectory_durations[scenario['name']] = 30.0
            else:
                # Step response uses config value
                self._trajectory_durations[scenario['name']] = self.test_duration
                self.get_logger().info(f'✓ {scenario["name"]}: {self.test_duration:.1f}s (step response)')

        self.get_logger().info('=' * 70)
        self.get_logger().info('✅ All durations precomputed - will use same values for ALL evaluations')
        self.get_logger().info('=' * 70)

    def _reset_trajectory(self) -> bool:
        """Reset trajectory and path generator via service calls.

        Returns:
            True if successful
        """
        try:
            # 1. Reset path generator first (regenerate path from current position)
            self.get_logger().info('🔄 Resetting path generator...')
            if self.reset_path_generator_client.wait_for_service(timeout_sec=2.0):
                request = ResetTrajectory.Request()
                future = self.reset_path_generator_client.call_async(request)

                start_wait = time.time()
                while not future.done() and (time.time() - start_wait) < 5.0:
                    time.sleep(0.01)

                if future.done() and future.result() and future.result().success:
                    self.get_logger().info('✓ Path generator reset successful')
                    # Wait for path to be regenerated
                    time.sleep(1.0)
                else:
                    self.get_logger().warning('⚠️  Path generator reset failed, continuing anyway...')
            else:
                self.get_logger().warning('⚠️  Path generator reset service not available')

            # 2. Reset path following node
            self.get_logger().info('🔄 Resetting path following node...')
            if not self.reset_trajectory_client.wait_for_service(timeout_sec=15.0):
                self.get_logger().error('❌ reset_trajectory service not available after 15s')
                return False

            request = ResetTrajectory.Request()
            future = self.reset_trajectory_client.call_async(request)

            start_wait = time.time()
            while not future.done() and (time.time() - start_wait) < 10.0:
                time.sleep(0.01)

            if not future.done() or future.result() is None:
                self.get_logger().error('❌ Service call timeout or failed')
                return False

            response = future.result()
            if response.success:
                self.get_logger().info('✓ Path following reset successful')
            else:
                self.get_logger().error(f'❌ Path following reset failed: {response.message}')

            return response.success

        except Exception as e:
            self.get_logger().error(f'❌ Exception in _reset_trajectory: {e}')
            import traceback
            traceback.print_exc()
            return False

    def _is_trajectory_stuck(self, window_seconds=10.0, stuck_error_threshold=2.0,
                            stuck_improvement_threshold=0.1) -> bool:
        """Detect if robot is stuck (spinning in place, not following trajectory).

        Args:
            window_seconds: Time window to check for progress (seconds)
            stuck_error_threshold: Error threshold to consider stuck (meters)
            stuck_improvement_threshold: Minimum error improvement required (meters)

        Returns:
            True if stuck
        """
        # Need sufficient data (at least window_seconds worth)
        samples_per_second = 50  # Assuming 50Hz odometry
        required_samples = int(window_seconds * samples_per_second)

        if len(self.errors_pos) < required_samples:
            return False  # Not enough data yet

        # Get recent error samples
        recent_errors = self.errors_pos[-required_samples:]

        # Condition 1: Error consistently large (> threshold)
        avg_error = np.mean([np.linalg.norm(e) for e in recent_errors])
        if avg_error < stuck_error_threshold:
            return False  # Error is acceptable, not stuck

        # Condition 2: Error not decreasing (stuck or increasing)
        half_point = len(recent_errors) // 2
        first_half = recent_errors[:half_point]
        second_half = recent_errors[half_point:]

        avg_error_first = np.mean([np.linalg.norm(e) for e in first_half])
        avg_error_second = np.mean([np.linalg.norm(e) for e in second_half])

        # Error should decrease over time if robot is following trajectory
        # If error stays same or increases, robot is stuck
        error_improvement = avg_error_first - avg_error_second

        if error_improvement < stuck_improvement_threshold:
            self.get_logger().warning(f'   ⚠️ Robot appears stuck:')
            self.get_logger().warning(f'      Average error: {avg_error:.2f}m (threshold: {stuck_error_threshold:.2f}m)')
            self.get_logger().warning(f'      Error improvement: {error_improvement:.3f}m (threshold: {stuck_improvement_threshold:.3f}m)')
            return True

        return False

    def _reset_controller(self) -> bool:
        """Reset controller (clears integral term and error history).

        Returns:
            True if successful
        """
        try:
            request = ResetController.Request()
            future = self.reset_controller_client.call_async(request)

            start_wait = time.time()
            while not future.done() and (time.time() - start_wait) < 5.0:
                time.sleep(0.01)

            if not future.done() or future.result() is None:
                self.get_logger().warning('⚠️  reset_controller service timeout')
                return False

            response = future.result()
            if response.success:
                self.get_logger().debug('✓ Controller reset successful (integral cleared)')
            else:
                self.get_logger().warning(f'⚠️  Controller reset failed')

            return response.success

        except Exception as e:
            self.get_logger().warning(f'⚠️  Exception in _reset_controller: {e}')
            return False

    def _stop_thrusters(self, current_gains, settling_time=5.0):
        """Stop all thrusters by disabling PID controller.

        Args:
            current_gains: Current PID gains (18-element array) to restore later
            settling_time: Time to wait for vehicle to settle (seconds)

        Returns:
            bool: True if successful
        """
        self.get_logger().info('🛑 Stopping thrusters (setting PID gains to zero)...')

        # Set all PID gains to zero to disable controller
        # Format: Single: 18 elements, Cascaded: 36 elements (position + velocity)
        zero_gains = np.zeros(self.dim)  # Use self.dim (18 for single, 36 for cascaded)
        success = self.set_pid_gains(zero_gains)

        if not success:
            self.get_logger().error('❌ Failed to set zero gains')
            return False

        self.get_logger().info(f'✓ PID disabled, waiting {settling_time:.1f}s for vehicle to settle...')
        time.sleep(settling_time)

        # Restore original gains
        self.get_logger().info('🔄 Restoring PID gains...')
        success = self.set_pid_gains(current_gains)

        if success:
            self.get_logger().info('✓ PID gains restored')
        else:
            self.get_logger().error('❌ Failed to restore gains')

        return success

    # ============================================================================
    # Cost Calculation
    # ============================================================================

    def calculate_cost(self) -> float:
        """Calculate cost based on collected error data

        Returns:
            Cost value (lower is better)
        """
        if self.cost_function == 'itse':
            return self.calculate_itse()
        elif self.cost_function == 'itae':
            return self.calculate_itae()
        else:
            raise ValueError(f'Unknown cost function: {self.cost_function}')

    def calculate_itse(self) -> float:
        """Calculate ITSE (Integral of Time-weighted Squared Error)

        Returns:
            ITSE value with smoothness penalty (integrated control effort)
        """
        if len(self.errors_pos) == 0:
            return float('inf')

        errors_pos = np.array(self.errors_pos)
        errors_ori = np.array(self.errors_ori)

        dt = 0.02  # 50Hz control rate
        n = len(errors_pos)
        t = np.arange(n) * dt

        # Weighted squared errors (separated by pose and orientation)
        squared_error_pos = np.sum(errors_pos**2 * self.weights[:3], axis=1)
        squared_error_ori = np.sum(errors_ori**2 * self.weights[3:], axis=1)

        # ITSE = integral of t * e(t)^2 (separated)
        itse_pos = np.sum(t * squared_error_pos) * dt
        itse_ori = np.sum(t * squared_error_ori) * dt
        itse = itse_pos + itse_ori

        # Smoothness penalty (integrated control input rate, MPC-style)
        # Now scales with trajectory length for fair comparison across scenarios
        smoothness_integral = 0.0
        if len(self.wrench_history) > 1:
            wrench_array = np.array(self.wrench_history)
            wrench_diff = np.diff(wrench_array, axis=0)  # Control input rate (Δu)
            # Sum of squared wrench changes per time step: ||Δu||²
            wrench_rate_squared = np.sum(wrench_diff**2, axis=1)
            # Integral over trajectory (uniform weighting)
            smoothness_integral = np.sum(wrench_rate_squared) * dt

        # Gain penalty (L2 regularization to prevent unrealistic high gains)
        gain_penalty = 0.0
        if self.lambda_gain > 0 and hasattr(self, '_current_gains'):
            # L2 norm of gains
            gain_penalty = self.lambda_gain * np.sum(self._current_gains ** 2)

        # Total cost (all terms now scale with trajectory length)
        smoothness_weighted = self.lambda_smoothness * smoothness_integral
        total_cost = itse + smoothness_weighted + gain_penalty

        # Debug logging (only for first evaluation)
        if not hasattr(self, '_cost_logged'):
            self._cost_logged = True
            log_msg = (f'Cost breakdown - ITSE={itse:.2f} (Pose={itse_pos:.2f}, Ori={itse_ori:.2f}), '
                      f'Smoothness_integral={smoothness_integral:.2f}×{self.lambda_smoothness:.3f}={smoothness_weighted:.2f}')
            if gain_penalty > 0:
                log_msg += f', Gain_penalty={gain_penalty:.2f}'
            log_msg += f', Total={total_cost:.2f}'
            self.get_logger().info(log_msg)

        return total_cost

    def calculate_itae(self) -> float:
        """Calculate ITAE (Integral of Time-weighted Absolute Error)

        Returns:
            ITAE value with smoothness penalty (integrated control effort)
        """
        if len(self.errors_pos) == 0:
            return float('inf')

        errors_pos = np.array(self.errors_pos)
        errors_ori = np.array(self.errors_ori)

        dt = 0.02  # 50Hz control rate
        n = len(errors_pos)
        t = np.arange(n) * dt

        # Weighted absolute errors (separated by pose and orientation)
        abs_error_pos = np.sum(np.abs(errors_pos) * self.weights[:3], axis=1)
        abs_error_ori = np.sum(np.abs(errors_ori) * self.weights[3:], axis=1)

        # ITAE = integral of t * |e(t)| (separated)
        itae_pos = np.sum(t * abs_error_pos) * dt
        itae_ori = np.sum(t * abs_error_ori) * dt
        itae = itae_pos + itae_ori

        # Smoothness penalty (integrated control input rate, MPC-style)
        # Now scales with trajectory length for fair comparison across scenarios
        smoothness_integral = 0.0
        if len(self.wrench_history) > 1:
            wrench_array = np.array(self.wrench_history)
            wrench_diff = np.diff(wrench_array, axis=0)  # Control input rate (Δu)
            # Sum of squared wrench changes per time step: ||Δu||²
            wrench_rate_squared = np.sum(wrench_diff**2, axis=1)
            # Integral over trajectory (uniform weighting)
            smoothness_integral = np.sum(wrench_rate_squared) * dt

        # Gain penalty (L2 regularization to prevent unrealistic high gains)
        gain_penalty = 0.0
        if self.lambda_gain > 0 and hasattr(self, '_current_gains'):
            # L2 norm of gains
            gain_penalty = self.lambda_gain * np.sum(self._current_gains ** 2)

        # Total cost (all terms now scale with trajectory length)
        smoothness_weighted = self.lambda_smoothness * smoothness_integral
        total_cost = itae + smoothness_weighted + gain_penalty

        # Debug logging (only for first evaluation)
        if not hasattr(self, '_cost_logged'):
            self._cost_logged = True
            log_msg = (f'Cost breakdown - ITAE={itae:.2f} (Pose={itae_pos:.2f}, Ori={itae_ori:.2f}), '
                      f'Smoothness_integral={smoothness_integral:.2f}×{self.lambda_smoothness:.3f}={smoothness_weighted:.2f}')
            if gain_penalty > 0:
                log_msg += f', Gain_penalty={gain_penalty:.2f}'
            log_msg += f', Total={total_cost:.2f}'
            self.get_logger().info(log_msg)

        return total_cost


    def calculate_trajectory_cost(self) -> float:
        """Calculate trajectory tracking cost (Multi-objective) - TUM Benchmark Standard

        Based on 2021-2025 research (TUM/KITTI/EuRoC + MDPI, Nature, IEEE, Mathworks):
        - ATE (Absolute Trajectory Error): RMSE with timestamp alignment (TUM standard)
        - Path Completion: Arc-length based progress (monotonic)
        - Attitude Stability: RMS of roll/pitch (time-independent)
        - Heading Tracking: RMS of heading error (consistent weighting)
        - Acceleration: Separated Linear/Angular RMS (proper scaling)
        - Smoothness: RMS of control input rate

        Priority weights (Ref: MPC literature):
          P1 (85%): Tracking (ATE + Completion)
          P2 (13%): Stability (Attitude + Heading)
          P3 (2%):  Control effort (Smoothness + Accel)

        Returns:
            Total cost = λ_ate×ATE_RMSE + λ_completion×uncompleted
                       + λ_attitude×RMS_attitude + λ_heading×RMS_heading
                       + λ_linear×RMS_linear_accel + λ_angular×RMS_angular_accel
                       + λ_smooth×RMS_control
        """
        # Check if we have necessary data
        if self.gt_path is None or self.actual_trajectory is None:
            self.get_logger().warning('⚠️  Missing GT path or actual trajectory for cost calculation')
            return float('inf')

        if len(self.odometry_history) == 0:
            self.get_logger().warning('⚠️  No odometry history for cost calculation')
            return float('inf')

        if not self.gt_path_poses:
            self.get_logger().warning('⚠️  Missing GT path timestamps for ATE calculation')
            return float('inf')

        # ========================================================================
        # 1. Cross-Track Error (CTE) - Perpendicular distance (Frenet Frame)
        # ========================================================================
        # Calculate perpendicular distance from GT path for all robot positions
        # Frenet-Serret frame definition:
        # - d(t): Lateral deviation (cross-track, perpendicular to path)
        # - s(t): Longitudinal position (along-track, measured separately)
        #
        # For each robot position:
        # 1. Find closest point on GT path
        # 2. Compute path tangent at that point
        # 3. Decompose error into along-track and cross-track
        # 4. CTE = magnitude of cross-track component
        #
        # Ref: Fossen 2011 - "Handbook of Marine Craft Hydrodynamics"
        # Ref: Lekkas & Fossen 2014 - "Line-of-Sight Guidance"
        # Ref: Breivik & Fossen 2005 - "Principles of Guidance-based Path Following"

        cte_errors = []
        for odom in self.odometry_history:
            robot_pos = odom['position']

            # Find closest point on GT path
            distances = np.linalg.norm(self.gt_path - robot_pos, axis=1)
            closest_idx = np.argmin(distances)
            closest_point = self.gt_path[closest_idx]

            # Compute path tangent at closest point
            if closest_idx == 0:
                tangent = self.gt_path[1] - self.gt_path[0]
            elif closest_idx >= len(self.gt_path) - 1:
                tangent = self.gt_path[-1] - self.gt_path[-2]
            else:
                # Central difference for better accuracy
                tangent = self.gt_path[closest_idx + 1] - self.gt_path[closest_idx - 1]

            # Normalize tangent
            tangent_norm = np.linalg.norm(tangent)
            if tangent_norm > 1e-6:
                tangent = tangent / tangent_norm
            else:
                tangent = np.array([1.0, 0.0, 0.0])  # Default direction

            # Decompose error into along-track and cross-track components
            error_vec = robot_pos - closest_point
            along_track_error = np.dot(error_vec, tangent)
            along_track_vec = along_track_error * tangent

            # Cross-track component (perpendicular to path)
            lateral_vec = error_vec - along_track_vec
            cte = np.linalg.norm(lateral_vec)

            cte_errors.append(cte)

        cte_array = np.array(cte_errors)

        # CTE RMSE: Root Mean Square Error
        cte_rmse = np.sqrt(np.mean(cte_array ** 2))

        # ========================================================================
        # 2. Path Completion - Arc-length based (Monotonic Progress)
        # ========================================================================
        # Track maximum progress along path using arc-length
        # Prevents issues with backtracking/overshooting

        # Calculate total path arc-length
        total_path_length = 0.0
        path_arclengths = [0.0]  # Arc-length at each path point
        for i in range(len(self.gt_path) - 1):
            segment_length = np.linalg.norm(self.gt_path[i + 1] - self.gt_path[i])
            total_path_length += segment_length
            path_arclengths.append(total_path_length)
        path_arclengths = np.array(path_arclengths)

        # CTE expected max is fixed (not path-length dependent)
        # Log it once for user awareness
        if not hasattr(self, '_cte_expected_max_logged'):
            self._cte_expected_max_logged = True
            self.get_logger().info(f'📏 Expected max CTE RMSE: {self.expected_max_cte_rmse:.2f}m (fixed, path-length independent)', once=True)

        # For each odometry point, find closest path point and its arc-length
        max_progress = 0.0
        for odom in self.odometry_history:
            robot_pos = odom['position']
            distances = np.linalg.norm(self.gt_path - robot_pos, axis=1)
            closest_idx = np.argmin(distances)

            # Arc-length progress at closest point
            progress_arclength = path_arclengths[closest_idx]

            # Track maximum progress (monotonic - never decreases)
            max_progress = max(max_progress, progress_arclength)

        # Path completion ratio (0 to 1)
        if total_path_length > 1e-6:
            path_completion_ratio = max_progress / total_path_length
        else:
            path_completion_ratio = 0.0

        # Completion error: uncompleted ratio (0~1)
        # 0.0 = fully completed, 1.0 = not completed at all
        path_completion_error = 1.0 - path_completion_ratio

        # ========================================================================
        # 3. Attitude RMS (Roll, Pitch → 0) - Underwater vehicle stability
        # ========================================================================
        # RMS: Time-independent, consistent weighting
        # Underwater vehicles should maintain level attitude (roll=0, pitch=0)
        # Use Euclidean norm (L2 norm) for combined roll/pitch error
        # Ref: Standard vector magnitude for orthogonal rotations

        odom_array = np.array([odom['rpy'] for odom in self.odometry_history])
        roll_history = odom_array[:, 0]  # Roll
        pitch_history = odom_array[:, 1]  # Pitch

        # Combined RMS for roll and pitch (Euclidean norm)
        # sqrt(mean(roll² + pitch²)) instead of sqrt(mean(roll²)) + sqrt(mean(pitch²))
        # This correctly treats roll/pitch as orthogonal rotation axes
        rms_attitude = np.sqrt(np.mean(roll_history**2 + pitch_history**2))

        # ========================================================================
        # 4. Heading Tracking Error - RMS of heading error
        # ========================================================================
        # For each odometry point, calculate heading error:
        # - Robot heading (yaw) from odometry
        # - Desired heading (yaw) from LOS guidance (path_tangent + chi_los)
        # - Heading error = robot_yaw - desired_yaw
        # RMS: Time-independent, consistent weighting across trajectory

        # Check if we have desired_yaw data
        if len(self.desired_yaw_history) == 0:
            self.get_logger().warning('⚠️  No desired_yaw history for heading cost calculation')
            rms_heading = 0.0
        else:
            heading_errors = []

            # Match odometry timestamps with desired_yaw timestamps
            # For each odometry point, find closest desired_yaw in time
            for odom in self.odometry_history:
                odom_timestamp = odom['timestamp']
                robot_yaw = odom['rpy'][2]  # Yaw angle

                # Find closest desired_yaw by timestamp
                closest_desired_yaw = None
                min_time_diff = float('inf')
                for desired_data in self.desired_yaw_history:
                    time_diff = abs(desired_data['timestamp'] - odom_timestamp)
                    if time_diff < min_time_diff:
                        min_time_diff = time_diff
                        closest_desired_yaw = desired_data['desired_yaw']

                if closest_desired_yaw is not None:
                    # Heading error (normalized to [-pi, pi])
                    heading_error = np.arctan2(
                        np.sin(robot_yaw - closest_desired_yaw),
                        np.cos(robot_yaw - closest_desired_yaw)
                    )
                    heading_errors.append(heading_error)
                else:
                    # No matching desired_yaw found
                    heading_errors.append(0.0)

            # RMS for heading error
            heading_errors_array = np.array(heading_errors)
            rms_heading = np.sqrt(np.mean(heading_errors_array ** 2))

        # ========================================================================
        # 5. Acceleration Penalty - Separated Linear/Angular (Proper Scaling)
        # ========================================================================
        # Penalize high accelerations to prevent aggressive maneuvers
        # Separated: Linear (m/s²) vs Angular (rad/s²) have different physical units
        # Ref: MDPI Sensors 2024 - separate metrics for proper scaling

        rms_linear_accel = 0.0
        rms_angular_accel = 0.0

        if len(self.odometry_history) > 1:
            # Extract velocity history
            velocity_history = np.array([odom['velocity'] for odom in self.odometry_history])

            # Numerical differentiation: acceleration = dv/dt
            dt_odom = 0.02  # 50Hz odometry rate
            acceleration_history = np.diff(velocity_history, axis=0) / dt_odom

            # Separate linear (x,y,z) and angular (roll,pitch,yaw) accelerations
            linear_accel = acceleration_history[:, 0:3]  # m/s²
            angular_accel = acceleration_history[:, 3:6]  # rad/s²

            # RMS for linear acceleration (3 DOF)
            linear_accel_squared = np.sum(linear_accel ** 2, axis=1)
            rms_linear_accel = np.sqrt(np.mean(linear_accel_squared))

            # RMS for angular acceleration (3 DOF)
            angular_accel_squared = np.sum(angular_accel ** 2, axis=1)
            rms_angular_accel = np.sqrt(np.mean(angular_accel_squared))

        # ========================================================================
        # 6. Smoothness Penalty - Control input smoothness (MPC-style)
        # ========================================================================
        # Penalize high-frequency vibrations in control inputs
        # Use RMS (Root Mean Square) to measure average smoothness
        # RMS is time-independent and captures jerkiness/vibration
        # Ref: MPC literature - minimize ||Δu||² for smooth control

        smoothness_rms = 0.0
        if len(self.wrench_history) > 1:
            wrench_array = np.array(self.wrench_history)
            wrench_diff = np.diff(wrench_array, axis=0)  # Control input rate (Δu)
            # RMS of wrench changes (time-independent measure of jerkiness)
            wrench_rate_squared = np.sum(wrench_diff**2, axis=1)
            # RMS = sqrt(mean(||Δu||²))
            # Higher RMS = more jerky/vibrating control
            smoothness_rms = np.sqrt(np.mean(wrench_rate_squared))

        # ========================================================================
        # Total Cost (Multi-objective with weights) - Simplified & Normalized
        # ========================================================================
        # Normalization: Scale each metric by expected maximum value
        # Ref: Mathworks MPC Tuning Guide, OR Stack Exchange
        # This ensures all metrics are in [0, ~1] range for proper weight balance

        # Normalize each metric
        cte_rmse_norm = cte_rmse / self.expected_max_cte_rmse
        completion_norm = path_completion_error / self.expected_max_completion
        attitude_norm = rms_attitude / self.expected_max_attitude
        heading_norm = rms_heading / self.expected_max_heading
        linear_accel_norm = rms_linear_accel / self.expected_max_linear_accel
        angular_accel_norm = rms_angular_accel / self.expected_max_angular_accel
        smoothness_norm = smoothness_rms / self.expected_max_smoothness

        # Apply weights to normalized metrics
        cost_cte_rmse = self.lambda_cte_rmse * cte_rmse_norm
        cost_completion = self.lambda_completion * completion_norm
        cost_attitude = self.lambda_rms_attitude * attitude_norm
        cost_heading = self.lambda_rms_heading * heading_norm
        cost_linear_accel = self.lambda_linear_accel * linear_accel_norm
        cost_angular_accel = self.lambda_angular_accel * angular_accel_norm
        cost_smoothness = self.lambda_trajectory_smoothness * smoothness_norm

        total_cost = (cost_cte_rmse + cost_completion + cost_attitude + cost_heading +
                      cost_linear_accel + cost_angular_accel + cost_smoothness)

        # Logging (only first time)
        if not hasattr(self, '_trajectory_cost_logged'):
            self._trajectory_cost_logged = True
            self.get_logger().info(f'📊 Trajectory Cost Breakdown (Frenet Frame - Perpendicular CTE):')
            self.get_logger().info(f'   ┌─ Priority 1 (CRITICAL - Path Tracking) ────────────────────')
            self.get_logger().info(f'   │  CTE RMSE (Perpendicular): {cte_rmse:.3f}m / {self.expected_max_cte_rmse:.1f} = {cte_rmse_norm:.3f} → ×{self.lambda_cte_rmse:.0f} = {cost_cte_rmse:.2f}')
            self.get_logger().info(f'   │  Path Completion: {path_completion_error:.1%} uncompleted / {self.expected_max_completion:.1f} = {completion_norm:.3f} → ×{self.lambda_completion:.0f} = {cost_completion:.2f}')
            self.get_logger().info(f'   ├─ Priority 2 (IMPORTANT - Stability) ──────────────────────')
            self.get_logger().info(f'   │  Attitude RMS: {rms_attitude:.3f}rad ({np.rad2deg(rms_attitude):.1f}°) / {self.expected_max_attitude:.2f} = {attitude_norm:.3f} → ×{self.lambda_rms_attitude:.0f} = {cost_attitude:.2f}')
            self.get_logger().info(f'   │  Heading RMS: {rms_heading:.3f}rad ({np.rad2deg(rms_heading):.1f}°) / {self.expected_max_heading:.2f} = {heading_norm:.3f} → ×{self.lambda_rms_heading:.0f} = {cost_heading:.2f}')
            self.get_logger().info(f'   ├─ Priority 3 (NICE-TO-HAVE - Control) ────────────────────')
            self.get_logger().info(f'   │  Smoothness RMS: {smoothness_rms:.3f}N / {self.expected_max_smoothness:.1f} = {smoothness_norm:.3f} → ×{self.lambda_trajectory_smoothness:.0f} = {cost_smoothness:.2f}')
            self.get_logger().info(f'   │  Linear Accel: {rms_linear_accel:.3f}m/s² / {self.expected_max_linear_accel:.1f} = {linear_accel_norm:.3f} → ×{self.lambda_linear_accel:.0f} = {cost_linear_accel:.2f}')
            self.get_logger().info(f'   │  Angular Accel: {rms_angular_accel:.3f}rad/s² / {self.expected_max_angular_accel:.1f} = {angular_accel_norm:.3f} → ×{self.lambda_angular_accel:.0f} = {cost_angular_accel:.2f}')
            self.get_logger().info(f'   └────────────────────────────────────────────────────────────')
            self.get_logger().info(f'   Total Cost: {total_cost:.2f}')

        return total_cost

    # ============================================================================
    # Evaluation
    # ============================================================================

    def evaluate_on_scenarios(self, gains: np.ndarray, scenarios: List[dict]) -> float:
        """Evaluate PID gains on multiple scenarios

        Args:
            gains: 18-element PID gains array
            scenarios: List of scenario dictionaries

        Returns:
            Average cost across scenarios
        """
        # Initialize data collection (safety: clear any leftover data from previous wolf)
        self.errors_pos = []
        self.errors_ori = []
        self.wrench_history = []
        self.odometry_history = []  # For trajectory cost calculation
        self.desired_yaw_history = []  # For heading error calculation
        self.collecting_data = False

        # Store gains for cost calculation (used in gain penalty)
        self._current_gains = gains

        # Set gains
        if not self.set_pid_gains(gains):
            self.get_logger().error('❌ Failed to set PID gains')
            return float('inf')

        time.sleep(0.5)  # Let gains take effect

        costs = []

        for idx, scenario in enumerate(scenarios):
            # Reset controller (clear integral term) before each scenario
            # This prevents integral windup from previous scenario affecting current one
            self._reset_controller()

            # Clear data
            self.errors_pos = []
            self.errors_ori = []
            self.wrench_history = []
            self.odometry_history = []
            self.desired_yaw_history = []

            # Reset debug flags for each scenario
            if hasattr(self, '_reference_logged'):
                delattr(self, '_reference_logged')
            if hasattr(self, '_cost_logged'):
                delattr(self, '_cost_logged')
            if hasattr(self, '_trajectory_cost_logged'):
                delattr(self, '_trajectory_cost_logged')

            # Scenario header
            self.get_logger().info('┌' + '─' * 68 + '┐')
            self.get_logger().info(f'│ Scenario {idx + 1}/{len(scenarios)}: {scenario["name"]:<54} │')
            self.get_logger().info('└' + '─' * 68 + '┘')

            # Handle different scenario types
            scenario_type = scenario.get('type', 'step')

            if scenario_type == 'trajectory':
                # Trajectory following scenario
                self.get_logger().info(f'🚀 Starting trajectory scenario')

                waypoint_file = scenario.get('waypoint_file')
                if not waypoint_file:
                    self.get_logger().error(f'❌ No waypoint_file for trajectory scenario {scenario["name"]}')
                    costs.append(float('inf'))
                    continue

                # Use precomputed duration (calculated once at startup)
                # This ensures ALL wolves/iterations use SAME duration for fair comparison
                test_duration = self._trajectory_durations.get(scenario['name'], 30.0)

                self.get_logger().info(f'📏 Using precomputed duration: {test_duration:.1f}s')

                # Reset trajectory to start
                self.get_logger().info(f'🔄 Resetting trajectory...')
                if not self._reset_trajectory():
                    self.get_logger().error(f'❌ Failed to reset trajectory for {scenario["name"]}')
                    costs.append(float('inf'))
                    continue

                self.get_logger().info(f'📊 Collecting data for {test_duration:.1f}s...')

                # Wait a bit for trajectory to start
                time.sleep(1.0)

                # Collect data for fixed duration (all wolves in one iteration use same time!)
                # This ensures fair comparison between different PID gains
                self.collecting_data = True
                start_time = time.time()
                check_interval = 0.5  # Check every 0.5s

                # Status detection parameters (for logging only, not for early termination)
                settling_threshold = 0.5  # Position error threshold (m)
                settling_duration = 5.0  # Time to confirm settling (s)
                settled_start = None
                settled_logged = False

                stuck_check_delay = 15.0  # Start checking after 15s
                stuck_window = 10.0  # seconds
                stuck_error_threshold = 2.0  # meters
                stuck_improvement_threshold = 0.1  # meters
                stuck_logged = False

                # Fixed duration loop - ALL wolves must run for the same time!
                while True:
                    elapsed = time.time() - start_time

                    # Status monitoring (for logging only, does NOT terminate early)
                    if elapsed > stuck_check_delay and not stuck_logged:
                        if self._is_trajectory_stuck(
                            window_seconds=stuck_window,
                            stuck_error_threshold=stuck_error_threshold,
                            stuck_improvement_threshold=stuck_improvement_threshold
                        ):
                            self.get_logger().warning(f'   ⚠️ Robot stuck (will continue collecting until {test_duration:.1f}s)')
                            stuck_logged = True

                    if len(self.errors_pos) > 0 and not settled_logged:
                        recent_errors = self.errors_pos[-10:] if len(self.errors_pos) >= 10 else self.errors_pos
                        avg_error = np.mean([np.linalg.norm(e) for e in recent_errors])

                        if avg_error < settling_threshold:
                            if settled_start is None:
                                settled_start = time.time()
                            # Check if settled for long enough
                            if time.time() - settled_start > settling_duration:
                                self.get_logger().info(f'   ✓ Settled at error={avg_error:.3f}m (will continue until {test_duration:.1f}s for fair comparison)')
                                settled_logged = True
                        else:
                            settled_start = None

                    # ONLY termination condition: fixed test_duration
                    # This ensures all wolves in one iteration are evaluated for the same time
                    if elapsed > test_duration:
                        self.get_logger().info(f'   ✓ Evaluation complete ({test_duration:.1f}s)')
                        break

                    time.sleep(check_interval)

                self.collecting_data = False

                # Stop thrusters by disabling PID and wait for vehicle to settle
                # This prevents thruster interference between evaluations
                self._stop_thrusters(current_gains=gains, settling_time=5.0)

                # Debug: Check collected data
                total_time = time.time() - start_time
                self.get_logger().info(f'   Collected {len(self.errors_pos)} error samples in {total_time:.1f}s')

                # Log final 6DOF pose comparison
                if len(self.errors_pos) > 0:
                    final_pos_error = self.errors_pos[-1]
                    final_ori_error = self.errors_ori[-1]

                    self.get_logger().info('═' * 70)
                    self.get_logger().info(f'📊 Final State for {scenario["name"]}:')
                    self.get_logger().info('─' * 70)
                    self.get_logger().info(f'Odometry:  pos=[{self.current_pos[0]:6.2f}, {self.current_pos[1]:6.2f}, {self.current_pos[2]:6.2f}] | '
                                           f'ori=[{np.rad2deg(self.current_rpy[0]):6.1f}°, {np.rad2deg(self.current_rpy[1]):6.1f}°, {np.rad2deg(self.current_rpy[2]):6.1f}°]')
                    self.get_logger().info(f'Error:     pos=[{final_pos_error[0]:6.3f}, {final_pos_error[1]:6.3f}, {final_pos_error[2]:6.3f}] | '
                                           f'ori=[{np.rad2deg(final_ori_error[0]):6.1f}°, {np.rad2deg(final_ori_error[1]):6.1f}°, {np.rad2deg(final_ori_error[2]):6.1f}°]')
                    self.get_logger().info('═' * 70)

                # Calculate cost using trajectory-specific cost function
                # (Cross-track + Path completion + Attitude + Smoothness)
                cost = self.calculate_trajectory_cost()
                costs.append(cost)

                self.get_logger().info(f'✓ Scenario "{scenario["name"]}" completed - Total Cost: {cost:.2f}')
                self.get_logger().info('')  # Empty line for separation

            else:
                # Step response scenario (original behavior)
                self.get_logger().info(f'📊 Collecting data for {self.test_duration:.1f}s...')

                # Wait for settling
                time.sleep(2.0)

                # Collect data and continuously publish reference
                self.collecting_data = True
                start_time = time.time()
                while (time.time() - start_time) < self.test_duration:
                    # Continuously publish reference to avoid timeout
                    if not self.publish_reference(scenario):
                        self.get_logger().error(f'❌ Failed to publish reference for {scenario["name"]}')
                        break
                    time.sleep(0.5)  # Publish at 2Hz (well within 1s timeout)
                self.collecting_data = False

                # Debug: Check collected data
                self.get_logger().info(f'   Collected {len(self.errors_pos)} error samples, '
                                       f'{len(self.wrench_history)} wrench samples')

                # Log final 6DOF pose comparison
                if len(self.errors_pos) > 0 and self.last_reference_pos is not None:
                    final_pos_error = self.errors_pos[-1]
                    final_ori_error = self.errors_ori[-1]
                    final_actual_pos = self.last_reference_pos - final_pos_error
                    final_actual_rpy = self.last_reference_rpy - final_ori_error

                    self.get_logger().info('═' * 70)
                    self.get_logger().info(f'📊 Final 6DOF Pose for {scenario["name"]}:')
                    self.get_logger().info('─' * 70)
                    self.get_logger().info(f'Reference: pos=[{self.last_reference_pos[0]:6.2f}, {self.last_reference_pos[1]:6.2f}, {self.last_reference_pos[2]:6.2f}] | '
                                           f'ori=[{np.rad2deg(self.last_reference_rpy[0]):6.1f}°, {np.rad2deg(self.last_reference_rpy[1]):6.1f}°, {np.rad2deg(self.last_reference_rpy[2]):6.1f}°]')
                    self.get_logger().info(f'Actual:    pos=[{final_actual_pos[0]:6.2f}, {final_actual_pos[1]:6.2f}, {final_actual_pos[2]:6.2f}] | '
                                           f'ori=[{np.rad2deg(final_actual_rpy[0]):6.1f}°, {np.rad2deg(final_actual_rpy[1]):6.1f}°, {np.rad2deg(final_actual_rpy[2]):6.1f}°]')
                    self.get_logger().info(f'Error:     pos=[{final_pos_error[0]:6.3f}, {final_pos_error[1]:6.3f}, {final_pos_error[2]:6.3f}] | '
                                           f'ori=[{np.rad2deg(final_ori_error[0]):6.1f}°, {np.rad2deg(final_ori_error[1]):6.1f}°, {np.rad2deg(final_ori_error[2]):6.1f}°]')
                    self.get_logger().info('─' * 70)
                    self.get_logger().info(f'Odometry:  pos=[{self.current_pos[0]:6.2f}, {self.current_pos[1]:6.2f}, {self.current_pos[2]:6.2f}] | '
                                           f'ori=[{np.rad2deg(self.current_rpy[0]):6.1f}°, {np.rad2deg(self.current_rpy[1]):6.1f}°, {np.rad2deg(self.current_rpy[2]):6.1f}°]')
                    self.get_logger().info('═' * 70)

                # Calculate cost (step response - use ITAE/ITSE)
                cost = self.calculate_cost()
                costs.append(cost)

                self.get_logger().info(f'✓ Scenario "{scenario["name"]}" completed - Total Cost: {cost:.2f}')
                self.get_logger().info('')  # Empty line for separation

                # Skip return to origin to save time during optimization
                # self.return_to_origin()

        # Return average cost
        if len(costs) == 0:
            self.get_logger().error('❌ No valid costs collected (all scenarios failed)')
            return float('inf')

        avg_cost = np.mean(costs)

        # Debug: log if we got inf or nan
        if np.isnan(avg_cost) or np.isinf(avg_cost):
            self.get_logger().warning(f'⚠️  Invalid cost: {avg_cost}, costs={costs}')

        return avg_cost

    # ============================================================================
    # Results
    # ============================================================================

    def _create_latest_run_symlink(self):
        """Create 'latest-run' symlink pointing to current results directory

        This allows easy access to the most recent optimization results without
        needing to know the exact timestamp.
        """
        try:
            import os
            # latest-run symlink in base directory
            latest_link = Path(self.results_base_dir) / 'latest-run'

            # Remove old symlink if exists
            if latest_link.is_symlink() or latest_link.exists():
                latest_link.unlink()

            # Create new symlink pointing to current results_dir
            # Use relative path for portability
            relative_target = self.results_dir.name
            os.symlink(relative_target, latest_link)

            self.get_logger().info(f'🔗 Created symlink: {latest_link} → {relative_target}')

        except Exception as e:
            self.get_logger().warning(f'⚠️  Failed to create latest-run symlink: {e}')

    def save_results(self, best_gains: np.ndarray, best_cost: float):
        """Save optimization results

        Args:
            best_gains: Best PID gains found
            best_cost: Best cost achieved
        """
        # Create results directory only when saving
        self.results_dir.mkdir(parents=True, exist_ok=True)

        # Create latest-run symlink
        self._create_latest_run_symlink()

        # Format gains with appropriate precision
        # Convert to Python native types to avoid numpy scalar issues in YAML
        kp = best_gains[0:6].tolist()
        kd = best_gains[6:12].tolist()
        ki = best_gains[12:18].tolist()

        results = {
            'optimizer': self.method,
            'timestamp': datetime.now().isoformat(),
            'cost_function': self.cost_function,
            'best_cost': float(best_cost),
            'Kp': [round(float(x), 1) for x in kp],
            'Kd': [round(float(x), 1) for x in kd],
            'Ki': [round(float(x), 3) for x in ki],
            'config': self.config
        }

        results_file = self.results_dir / 'optimization_results.yaml'
        with open(results_file, 'w') as f:
            yaml.dump(results, f, default_flow_style=False)

        self.get_logger().info(f'💾 Results saved to {results_file}')

    # ============================================================================
    # Checkpoint Management
    # ============================================================================

    def save_checkpoint(self, checkpoint_data: dict, iteration: int):
        """Save checkpoint to file

        Args:
            checkpoint_data: Dictionary containing optimizer state
            iteration: Current iteration number
        """
        try:
            # Create checkpoint directory only when saving
            self.checkpoint_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f'📁 Checkpoint directory: {self.checkpoint_dir}')

            # Create latest-run symlink on first checkpoint save
            if not hasattr(self, '_latest_run_symlink_created'):
                self._create_latest_run_symlink()
                self._latest_run_symlink_created = True

            checkpoint_file = self.checkpoint_dir / f'checkpoint_iter_{iteration}.yaml'
            self.get_logger().info(f'💾 Saving checkpoint to: {checkpoint_file}')

            # Add metadata
            checkpoint_data['metadata'] = {
                'timestamp': datetime.now().isoformat(),
                'iteration': iteration,
                'method': self.method,
                'cost_function': self.cost_function,
            }

            # Save checkpoint
            with open(checkpoint_file, 'w') as f:
                yaml.dump(checkpoint_data, f, default_flow_style=False)

            self.get_logger().info(f'✅ Checkpoint saved successfully: {checkpoint_file}')

            # Keep only last 3 checkpoints to save disk space
            self._cleanup_old_checkpoints(keep_last=3)

            return checkpoint_file

        except Exception as e:
            self.get_logger().error(f'❌ Failed to save checkpoint: {e}')
            import traceback
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
            return None

    def load_checkpoint(self, checkpoint_file: str) -> dict:
        """Load checkpoint from file

        Args:
            checkpoint_file: Path to checkpoint file

        Returns:
            Dictionary containing optimizer state, or None if failed
        """
        try:
            checkpoint_path = Path(checkpoint_file)
            if not checkpoint_path.exists():
                self.get_logger().error(f'❌ Checkpoint file not found: {checkpoint_file}')
                return None

            with open(checkpoint_path, 'r') as f:
                checkpoint_data = yaml.safe_load(f)

            # Validate checkpoint
            if 'metadata' not in checkpoint_data:
                self.get_logger().error(f'❌ Invalid checkpoint: missing metadata')
                return None

            metadata = checkpoint_data['metadata']
            self.get_logger().info(f'📂 Loaded checkpoint from iteration {metadata["iteration"]}')
            self.get_logger().info(f'   Method: {metadata["method"]}, Cost: {metadata["cost_function"]}')
            self.get_logger().info(f'   Timestamp: {metadata["timestamp"]}')

            return checkpoint_data

        except Exception as e:
            self.get_logger().error(f'❌ Failed to load checkpoint: {e}')
            import traceback
            traceback.print_exc()
            return None

    def _cleanup_old_checkpoints(self, keep_last: int = 3):
        """Remove old checkpoint files, keeping only the most recent ones

        Args:
            keep_last: Number of recent checkpoints to keep
        """
        try:
            # Find all checkpoint files
            checkpoint_files = sorted(
                self.checkpoint_dir.glob('checkpoint_iter_*.yaml'),
                key=lambda p: p.stat().st_mtime,
                reverse=True
            )

            # Remove old checkpoints
            for old_checkpoint in checkpoint_files[keep_last:]:
                old_checkpoint.unlink()
                self.get_logger().debug(f'Removed old checkpoint: {old_checkpoint.name}')

        except Exception as e:
            self.get_logger().warning(f'⚠️  Failed to cleanup old checkpoints: {e}')

    # ============================================================================
    # Abstract Methods
    # ============================================================================

    @abstractmethod
    def optimize(self) -> Tuple[np.ndarray, float]:
        """Run optimization

        Returns:
            (best_gains, best_cost): Best PID gains and corresponding cost
        """
        pass
