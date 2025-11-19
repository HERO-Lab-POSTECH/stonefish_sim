#!/usr/bin/env python3
"""
SMAC3 Bayesian Optimizer for PID Tuning

Implements Bayesian optimization using SMAC3 for PID gain tuning.
"""

import numpy as np
import time
from typing import Tuple
from ConfigSpace import ConfigurationSpace, Float

from .base import BaseOptimizer

try:
    from smac import HyperparameterOptimizationFacade, Scenario
    SMAC3_AVAILABLE = True
except ImportError:
    SMAC3_AVAILABLE = False

try:
    import wandb
    WANDB_AVAILABLE = True
except ImportError:
    WANDB_AVAILABLE = False


class SMAC3Optimizer(BaseOptimizer):
    """SMAC3 Bayesian Optimizer for PID gain tuning

    Uses Bayesian optimization to efficiently explore the PID gain space.
    """

    def __init__(self, config: dict):
        """Initialize SMAC3 optimizer

        Args:
            config: Configuration dictionary from YAML
        """
        super().__init__(config, node_name='smac3_pid_optimizer')

        if not SMAC3_AVAILABLE:
            raise ImportError('SMAC3 not available. Install with: pip install smac')

        # SMAC3-specific parameters
        opt_config = config['optimizer']
        self.n_trials = opt_config['n_trials']
        self.random_fraction = opt_config.get('random_fraction', 0.2)
        self.settling_threshold = opt_config.get('settling_threshold', 0.05)

        # SMAC3 checkpoint/resume parameters
        self.smac_output_dir = str(self.results_dir / 'smac3_output')
        self.smac_overwrite = opt_config.get('smac_overwrite', False)  # False = resume if possible

        # Progress tracking
        self.trial_count = 0
        self.best_cost = float('inf')
        self.best_gains = None
        self.start_time = None
        self.validation_mode = False

        # Split scenarios into training and validation
        # Use all scenarios for training (validation uses same scenarios)
        # This ensures all defined scenarios are used during optimization
        self.training_scenarios = self.scenarios
        self.validation_scenarios = self.scenarios  # Same scenarios for final validation

        # Log configuration
        self.get_logger().info(f'🔬 SMAC3 Bayesian Optimizer')
        self.get_logger().info(f'   Trials: {self.n_trials}')
        self.get_logger().info(f'   Random fraction: {self.random_fraction}')
        self.get_logger().info(f'   Training scenarios: {len(self.training_scenarios)}')
        self.get_logger().info(f'   Validation scenarios: {len(self.validation_scenarios)}')

    def configspace(self) -> ConfigurationSpace:
        """Define SMAC3 configuration space

        Returns:
            ConfigurationSpace object with PID gain parameters
        """
        # Default gains (use warm start if available)
        if self.warm_start_gains is not None:
            Kp_default = self.warm_start_gains[0:6].tolist()
            Kd_default = self.warm_start_gains[6:12].tolist()
            Ki_default = self.warm_start_gains[12:18].tolist()
        else:
            # Use midpoint of bounds
            Kp_default = ((self.lb[0:6] + self.ub[0:6]) / 2).tolist()
            Kd_default = ((self.lb[6:12] + self.ub[6:12]) / 2).tolist()
            Ki_default = ((self.lb[12:18] + self.ub[12:18]) / 2).tolist()

        # Clamp defaults to bounds
        def clamp(val, min_val, max_val):
            return max(min_val, min(val, max_val))

        Kp_clamped = [clamp(Kp_default[i], self.lb[i], self.ub[i]) for i in range(6)]
        Kd_clamped = [clamp(Kd_default[i], self.lb[i+6], self.ub[i+6]) for i in range(6)]
        Ki_clamped = [clamp(Ki_default[i], self.lb[i+12], self.ub[i+12]) for i in range(6)]

        # Create configuration space
        cs = ConfigurationSpace(
            name="PID_Tuning",
            seed=42,
            space={
                # Kp (Proportional gains)
                "Kp_x": Float("Kp_x", (self.lb[0], self.ub[0]), default=Kp_clamped[0]),
                "Kp_y": Float("Kp_y", (self.lb[1], self.ub[1]), default=Kp_clamped[1]),
                "Kp_z": Float("Kp_z", (self.lb[2], self.ub[2]), default=Kp_clamped[2]),
                "Kp_roll": Float("Kp_roll", (self.lb[3], self.ub[3]), default=Kp_clamped[3]),
                "Kp_pitch": Float("Kp_pitch", (self.lb[4], self.ub[4]), default=Kp_clamped[4]),
                "Kp_yaw": Float("Kp_yaw", (self.lb[5], self.ub[5]), default=Kp_clamped[5]),
                # Kd (Derivative gains)
                "Kd_x": Float("Kd_x", (self.lb[6], self.ub[6]), default=Kd_clamped[0]),
                "Kd_y": Float("Kd_y", (self.lb[7], self.ub[7]), default=Kd_clamped[1]),
                "Kd_z": Float("Kd_z", (self.lb[8], self.ub[8]), default=Kd_clamped[2]),
                "Kd_roll": Float("Kd_roll", (self.lb[9], self.ub[9]), default=Kd_clamped[3]),
                "Kd_pitch": Float("Kd_pitch", (self.lb[10], self.ub[10]), default=Kd_clamped[4]),
                "Kd_yaw": Float("Kd_yaw", (self.lb[11], self.ub[11]), default=Kd_clamped[5]),
                # Ki (Integral gains)
                "Ki_x": Float("Ki_x", (self.lb[12], self.ub[12]), default=Ki_clamped[0]),
                "Ki_y": Float("Ki_y", (self.lb[13], self.ub[13]), default=Ki_clamped[1]),
                "Ki_z": Float("Ki_z", (self.lb[14], self.ub[14]), default=Ki_clamped[2]),
                "Ki_roll": Float("Ki_roll", (self.lb[15], self.ub[15]), default=Ki_clamped[3]),
                "Ki_pitch": Float("Ki_pitch", (self.lb[16], self.ub[16]), default=Ki_clamped[4]),
                "Ki_yaw": Float("Ki_yaw", (self.lb[17], self.ub[17]), default=Ki_clamped[5]),
            }
        )

        return cs

    def _config_to_gains(self, config) -> np.ndarray:
        """Convert SMAC3 configuration to gains array

        Args:
            config: SMAC3 Configuration object

        Returns:
            18-element gains array [kp(6), kd(6), ki(6)]
        """
        Kp = [config["Kp_x"], config["Kp_y"], config["Kp_z"],
              config["Kp_roll"], config["Kp_pitch"], config["Kp_yaw"]]
        Kd = [config["Kd_x"], config["Kd_y"], config["Kd_z"],
              config["Kd_roll"], config["Kd_pitch"], config["Kd_yaw"]]
        Ki = [config["Ki_x"], config["Ki_y"], config["Ki_z"],
              config["Ki_roll"], config["Ki_pitch"], config["Ki_yaw"]]

        gains = np.array(Kp + Kd + Ki)
        return gains

    def evaluate_configuration(self, config, seed: int = 0) -> float:
        """SMAC3 target function - evaluate PID configuration

        Args:
            config: SMAC3 Configuration object with PID gains
            seed: Random seed (not used)

        Returns:
            cost: Lower is better
        """
        # Check for interrupt
        if self.is_interrupted():
            self.get_logger().warning('⚠️  Optimization interrupted')
            # SMAC3 automatically saves state, no manual checkpoint needed
            return float('inf')

        # Track progress
        if not self.validation_mode:
            self.trial_count += 1
            progress_pct = (self.trial_count / self.n_trials) * 100

            # Estimate ETA
            if self.start_time is not None:
                elapsed = time.time() - self.start_time
                avg_time_per_trial = elapsed / self.trial_count
                remaining_trials = self.n_trials - self.trial_count
                eta_seconds = avg_time_per_trial * remaining_trials
                eta_minutes = eta_seconds / 60
                eta_str = f"{int(eta_minutes)}m {int(eta_seconds % 60)}s"
            else:
                self.start_time = time.time()
                eta_str = "calculating..."

            # Log progress
            self.get_logger().info('=' * 70)
            self.get_logger().info(f'🔍 Trial {self.trial_count}/{self.n_trials} ({progress_pct:.1f}%) | '
                                   f'Best: {self.best_cost:.4f} | ETA: {eta_str}')
            self.get_logger().info('=' * 70)

        try:
            # Convert config to gains
            gains = self._config_to_gains(config)

            # Log gains
            Kp = gains[0:6]
            Kd = gains[6:12]
            Ki = gains[12:18]

            self.get_logger().info(f'📊 PID Configuration:')
            self.get_logger().info(f'    Kp=[{Kp[0]:.1f}, {Kp[1]:.1f}, {Kp[2]:.1f}, '
                                   f'{Kp[3]:.1f}, {Kp[4]:.1f}, {Kp[5]:.1f}]')
            self.get_logger().info(f'    Kd=[{Kd[0]:.1f}, {Kd[1]:.1f}, {Kd[2]:.1f}, '
                                   f'{Kd[3]:.1f}, {Kd[4]:.1f}, {Kd[5]:.1f}]')
            self.get_logger().info(f'    Ki=[{Ki[0]:.2f}, {Ki[1]:.2f}, {Ki[2]:.2f}, '
                                   f'{Ki[3]:.2f}, {Ki[4]:.2f}, {Ki[5]:.2f}]')

            # Evaluate on training scenarios
            cost = self.evaluate_on_scenarios(gains, self.training_scenarios)

            # Update best
            if not self.validation_mode and cost < self.best_cost:
                self.best_cost = cost
                self.best_gains = gains
                improvement_marker = '🌟 NEW BEST!'

                # Save best gains
                self._save_best_gains(Kp, Kd, Ki, cost)
            else:
                improvement_marker = ''

            self.get_logger().info(f'  ➜ Cost={cost:.2f} {improvement_marker}')
            self.get_logger().info('  ' + '─' * 66)
            self.get_logger().info('')  # Empty line for separation

            # WandB logging
            if self.use_wandb and not self.validation_mode and WANDB_AVAILABLE:
                wandb.log({
                    'cost': cost,
                    'best_cost': self.best_cost,
                    'Kp_x': Kp[0], 'Kp_y': Kp[1], 'Kp_z': Kp[2],
                    'Kp_roll': Kp[3], 'Kp_pitch': Kp[4], 'Kp_yaw': Kp[5],
                    'Kd_x': Kd[0], 'Kd_y': Kd[1], 'Kd_z': Kd[2],
                    'Kd_roll': Kd[3], 'Kd_pitch': Kd[4], 'Kd_yaw': Kd[5],
                    'Ki_x': Ki[0], 'Ki_y': Ki[1], 'Ki_z': Ki[2],
                    'Ki_roll': Ki[3], 'Ki_pitch': Ki[4], 'Ki_yaw': Ki[5],
                })

            return cost

        except Exception as e:
            self.get_logger().error(f'❌ Exception in evaluate_configuration: {e}')
            import traceback
            traceback.print_exc()
            return float('inf')

    def _save_best_gains(self, Kp, Kd, Ki, cost):
        """Save current best gains to file

        Args:
            Kp, Kd, Ki: Gain arrays
            cost: Cost value
        """
        if not self.use_wandb or not WANDB_AVAILABLE:
            return

        try:
            wandb_dir = wandb.run.dir
            best_yaml_path = f'{wandb_dir}/best_gains.yaml'

            with open(best_yaml_path, 'w') as f:
                f.write(f'# Best PID Gains from SMAC3 Optimization\n')
                f.write(f'# Cost ({self.cost_function.upper()}): {cost:.3f}\n')
                f.write(f'# Trial: {self.trial_count}\n')
                f.write(f'#\n')
                f.write(f'/**:\n')
                f.write(f'  ros__parameters:\n')
                f.write(f'    Kp: [{", ".join([f"{x:.1f}" for x in Kp])}]\n')
                f.write(f'    Kd: [{", ".join([f"{x:.1f}" for x in Kd])}]\n')
                f.write(f'    Ki: [{", ".join([f"{x:.3f}" for x in Ki])}]\n')

        except Exception as e:
            self.get_logger().warning(f'⚠️  Failed to save best gains: {e}')

    def optimize(self) -> Tuple[np.ndarray, float]:
        """Run SMAC3 optimization

        Returns:
            (best_gains, best_cost): Tuple of best PID gains and corresponding cost
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('🚀 Starting SMAC3 Optimization')
        self.get_logger().info('=' * 60)

        # Create scenario with name and output directory for checkpoint support
        scenario = Scenario(
            configspace=self.configspace(),
            n_trials=self.n_trials,
            seed=42,
            name='pid_tuning',  # Required for checkpoint/resume
            output_directory=self.smac_output_dir,  # Where SMAC3 stores its state
        )

        # Create SMAC optimizer
        # overwrite=False allows resuming from previous runs
        smac = HyperparameterOptimizationFacade(
            scenario=scenario,
            target_function=self.evaluate_configuration,
            overwrite=self.smac_overwrite,  # False = resume if possible, True = start fresh
        )

        # Check if we're resuming
        if smac.runhistory.finished > 0:
            self.get_logger().info(f'🔄 Resuming SMAC3 optimization from trial {smac.runhistory.finished}')
            self.trial_count = smac.runhistory.finished

        # Run optimization
        self.start_time = time.time()
        incumbent = smac.optimize()

        # Extract best gains
        best_gains = self._config_to_gains(incumbent)

        # Final evaluation on validation set
        self.get_logger().info('=' * 60)
        if self.is_interrupted():
            self.get_logger().info('⚠️  OPTIMIZATION INTERRUPTED')
        else:
            self.get_logger().info('🎉 OPTIMIZATION COMPLETE!')
        self.get_logger().info('=' * 60)

        # Check if we have valid results
        if best_gains is None or self.best_cost == float('inf'):
            self.get_logger().error('❌ No valid solution found')
            return None, float('inf')

        self.get_logger().info(f'Training Cost: {self.best_cost:.2f}')

        # Validate (skip if interrupted)
        validation_cost = None
        if not self.is_interrupted() and len(self.validation_scenarios) > 0:
            self.get_logger().info(f'\n📊 Validating on {len(self.validation_scenarios)} scenarios...')
            validation_cost = self.evaluate_on_scenarios(best_gains, self.validation_scenarios)
            self.get_logger().info(f'   Validation Cost: {validation_cost:.2f}')

        # Log best gains
        Kp = best_gains[0:6]
        Kd = best_gains[6:12]
        Ki = best_gains[12:18]

        self.get_logger().info(f'\n🎯 Best Gains:')
        self.get_logger().info(f'  Kp: {Kp}')
        self.get_logger().info(f'  Kd: {Kd}')
        self.get_logger().info(f'  Ki: {Ki}')

        # WandB summary
        if self.use_wandb and WANDB_AVAILABLE:
            wandb.run.summary['final_cost'] = self.best_cost
            wandb.run.summary['validation_cost'] = validation_cost
            wandb.run.summary['interrupted'] = self.is_interrupted()
            wandb.run.summary['best_Kp'] = Kp.tolist()
            wandb.run.summary['best_Kd'] = Kd.tolist()
            wandb.run.summary['best_Ki'] = Ki.tolist()
            self.get_logger().info(f'📊 Results logged to WandB: {wandb.run.url}')

        # Save results
        self.save_results(best_gains, self.best_cost)

        # Cleanup (finish WandB run)
        self.cleanup()

        return best_gains, self.best_cost
