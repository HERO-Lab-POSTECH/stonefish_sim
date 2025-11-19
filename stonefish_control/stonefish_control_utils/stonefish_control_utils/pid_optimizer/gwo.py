#!/usr/bin/env python3
"""
Grey Wolf Optimizer (GWO) for PID Tuning

Implements the GWO algorithm for optimizing PID gains.
"""

import numpy as np
import time
from typing import Tuple
from scipy import special

from .base import BaseOptimizer

try:
    import wandb
    WANDB_AVAILABLE = True
except ImportError:
    WANDB_AVAILABLE = False


class GWOOptimizer(BaseOptimizer):
    """Grey Wolf Optimizer for PID gain tuning

    GWO maintains a pack of wolves representing PID configurations.
    The best three wolves (alpha, beta, delta) guide the pack toward
    optimal solutions.
    """

    def __init__(self, config: dict):
        """Initialize GWO optimizer

        Args:
            config: Configuration dictionary from YAML
        """
        super().__init__(config, node_name='gwo_pid_optimizer')

        # GWO-specific parameters
        opt_config = config['optimizer']
        self.n_wolves = opt_config['n_wolves']
        self.max_iterations = opt_config['max_iterations']

        # GWO state - best three wolves
        self.alpha_wolf = None  # Best
        self.beta_wolf = None   # Second best
        self.delta_wolf = None  # Third best
        self.alpha_score = float('inf')
        self.beta_score = float('inf')
        self.delta_score = float('inf')

        # Split scenarios into training and validation
        # Use all scenarios for training (validation uses same scenarios)
        # This ensures all defined scenarios are used during optimization
        self.training_scenarios = self.scenarios
        self.validation_scenarios = self.scenarios  # Same scenarios for final validation

        # Log configuration
        self.get_logger().info(f'🐺 Grey Wolf Optimizer (GWO)')
        self.get_logger().info(f'   Wolves: {self.n_wolves}')
        self.get_logger().info(f'   Max iterations: {self.max_iterations}')
        self.get_logger().info(f'   Training scenarios: {len(self.training_scenarios)}')
        self.get_logger().info(f'   Validation scenarios: {len(self.validation_scenarios)}')

        # Time estimate
        time_per_eval = len(self.training_scenarios) * (2 + self.test_duration)
        total_evals = self.n_wolves * self.max_iterations
        total_hours = total_evals * time_per_eval / 3600
        self.get_logger().info(f'⏱️  Estimated time: {total_hours:.1f} hours')
        self.get_logger().info(f'   (Early stopping may reduce this)')

    def levy_flight(self, beta: float = 1.5) -> np.ndarray:
        """Generate Lévy flight step using Mantegna algorithm

        Args:
            beta: Lévy exponent, typically 1.5 (range: 1.0-2.0)

        Returns:
            Lévy step vector of dimension self.dim
        """
        # Calculate sigma_u using Mantegna's formula
        numerator = special.gamma(1 + beta) * np.sin(np.pi * beta / 2)
        denominator = special.gamma((1 + beta) / 2) * beta * (2 ** ((beta - 1) / 2))
        sigma_u = (numerator / denominator) ** (1 / beta)
        sigma_v = 1.0

        # Generate random numbers from normal distributions
        u = np.random.normal(0, sigma_u, self.dim)
        v = np.random.normal(0, sigma_v, self.dim)

        # Calculate Lévy step
        step = u / (np.abs(v) ** (1 / beta))

        return step

    def cauchy_gaussian_mutation(self, wolf: np.ndarray, t: int, T: int,
                                  fitness: float, best_fitness: float) -> np.ndarray:
        """Apply Cauchy-Gaussian mutation to a wolf

        Based on CG-GWO algorithm from Scientific Reports (2022)

        Args:
            wolf: Current wolf position
            t: Current iteration
            T: Maximum iterations
            fitness: Current wolf fitness
            best_fitness: Best fitness so far

        Returns:
            Mutated wolf position
        """
        # Dynamic weights: Cauchy dominant early, Gaussian dominant late
        lambda1 = 1 - (t / T) ** 2  # Cauchy weight
        lambda2 = (t / T) ** 2       # Gaussian weight

        # Adaptive standard deviation
        if abs(best_fitness) > 1e-10:
            sigma = np.exp((fitness - best_fitness) / abs(best_fitness))
        else:
            sigma = 1.0

        # Generate Cauchy and Gaussian random numbers
        cauchy_step = np.random.standard_cauchy(self.dim) * sigma
        gaussian_step = np.random.normal(0, sigma, self.dim)

        # Apply mutation
        mutated_wolf = wolf + lambda1 * cauchy_step + lambda2 * gaussian_step

        # Enforce bounds
        mutated_wolf = np.clip(mutated_wolf, self.lb, self.ub)

        return mutated_wolf

    def initialize_wolves(self) -> np.ndarray:
        """Initialize wolf population with hybrid strategy

        Strategy:
        - First wolf: warm start (if available)
        - 30% of wolves: around warm start ±20% (local exploitation)
        - 70% of wolves: random across full bounds (global exploration)

        Returns:
            wolves: Array of shape (n_wolves, dim)
        """
        wolves = np.zeros((self.n_wolves, self.dim))

        if self.warm_start_gains is not None:
            # First wolf = warm start
            wolves[0] = self.warm_start_gains
            self.get_logger().info(f'🔥 Wolf 1 initialized with warm start gains')

            # Calculate split: 30% local, 70% global
            n_local = max(1, int(0.3 * (self.n_wolves - 1)))
            n_global = self.n_wolves - 1 - n_local

            self.get_logger().info(f'📊 Hybrid initialization:')
            self.get_logger().info(f'   - 1 warm start')
            self.get_logger().info(f'   - {n_local} local (warm ±20%)')
            self.get_logger().info(f'   - {n_global} global (full bounds)')

            # Local exploration around warm start (±20%)
            for i in range(1, 1 + n_local):
                wolves[i] = self.warm_start_gains * np.random.uniform(0.8, 1.2, self.dim)
                wolves[i] = np.clip(wolves[i], self.lb, self.ub)

            # Global exploration across full bounds
            for i in range(1 + n_local, self.n_wolves):
                wolves[i] = self.lb + np.random.rand(self.dim) * (self.ub - self.lb)
        else:
            # No warm start: random initialization within bounds
            self.get_logger().info(f'🎲 Random initialization across full bounds')
            for i in range(self.n_wolves):
                wolves[i] = self.lb + np.random.rand(self.dim) * (self.ub - self.lb)

        return wolves

    def evaluate_wolf(self, wolf: np.ndarray, wolf_idx: int) -> float:
        """Evaluate a single wolf (PID configuration)

        Args:
            wolf: PID gains array (18 elements)
            wolf_idx: Index of this wolf (for logging)

        Returns:
            Average cost across training scenarios
        """
        try:
            cost = self.evaluate_on_scenarios(wolf, self.training_scenarios)
            return cost

        except Exception as e:
            self.get_logger().error(f'❌ Exception evaluating wolf {wolf_idx}: {e}')
            return float('inf')

    def update_leaders(self, wolf: np.ndarray, fitness: float, iteration: int, wolf_idx: int) -> str:
        """Update alpha, beta, delta wolves

        Args:
            wolf: Current wolf position
            fitness: Fitness value
            iteration: Current iteration
            wolf_idx: Wolf index

        Returns:
            Marker string for logging ('🌟 NEW BEST!', '✨ Beta', '💫 Delta', or '')
        """
        marker = ''

        if fitness < self.alpha_score:
            # New best - shift down existing leaders
            self.delta_score = self.beta_score
            self.delta_wolf = np.copy(self.beta_wolf) if self.beta_wolf is not None else None
            self.beta_score = self.alpha_score
            self.beta_wolf = np.copy(self.alpha_wolf) if self.alpha_wolf is not None else None
            self.alpha_score = fitness
            self.alpha_wolf = np.copy(wolf)
            marker = '🌟 NEW BEST!'

            # Save best gains
            self._save_best_gains(iteration, wolf_idx)

        elif fitness < self.beta_score:
            # New second best
            self.delta_score = self.beta_score
            self.delta_wolf = np.copy(self.beta_wolf) if self.beta_wolf is not None else None
            self.beta_score = fitness
            self.beta_wolf = np.copy(wolf)
            marker = '✨ Beta'

        elif fitness < self.delta_score:
            # New third best
            self.delta_score = fitness
            self.delta_wolf = np.copy(wolf)
            marker = '💫 Delta'

        return marker

    def _save_best_gains(self, iteration: int, wolf_idx: int):
        """Save current best gains to file

        Args:
            iteration: Current iteration number
            wolf_idx: Wolf index
        """
        if not self.use_wandb or not WANDB_AVAILABLE:
            return

        try:
            wandb_dir = wandb.run.dir
            best_yaml_path = f'{wandb_dir}/best_gains.yaml'

            Kp = self.alpha_wolf[0:6]
            Kd = self.alpha_wolf[6:12]
            Ki = self.alpha_wolf[12:18]

            with open(best_yaml_path, 'w') as f:
                f.write(f'# Best PID Gains from GWO Optimization\n')
                f.write(f'# Cost ({self.cost_function.upper()}): {self.alpha_score:.3f}\n')
                f.write(f'# Iteration: {iteration + 1}\n')
                f.write(f'# Wolf: {wolf_idx + 1}\n')
                f.write(f'#\n')
                f.write(f'/**:\n')
                f.write(f'  ros__parameters:\n')
                f.write(f'    Kp: [{", ".join([f"{x:.1f}" for x in Kp])}]\n')
                f.write(f'    Kd: [{", ".join([f"{x:.1f}" for x in Kd])}]\n')
                f.write(f'    Ki: [{", ".join([f"{x:.3f}" for x in Ki])}]\n')

        except Exception as e:
            self.get_logger().warning(f'⚠️  Failed to save best gains: {e}')

    def _log_wolf_to_wandb(self, wolf: np.ndarray, fitness: float):
        """Log wolf evaluation to WandB

        Args:
            wolf: Wolf position (gains)
            fitness: Fitness value
        """
        if not self.use_wandb or not WANDB_AVAILABLE:
            return

        try:
            wolf_Kp = wolf[0:6]
            wolf_Kd = wolf[6:12]
            wolf_Ki = wolf[12:18]

            wandb.log({
                'cost': fitness,
                'best_cost': self.alpha_score,
                # Kp gains
                'Kp_x': wolf_Kp[0], 'Kp_y': wolf_Kp[1], 'Kp_z': wolf_Kp[2],
                'Kp_roll': wolf_Kp[3], 'Kp_pitch': wolf_Kp[4], 'Kp_yaw': wolf_Kp[5],
                # Kd gains
                'Kd_x': wolf_Kd[0], 'Kd_y': wolf_Kd[1], 'Kd_z': wolf_Kd[2],
                'Kd_roll': wolf_Kd[3], 'Kd_pitch': wolf_Kd[4], 'Kd_yaw': wolf_Kd[5],
                # Ki gains
                'Ki_x': wolf_Ki[0], 'Ki_y': wolf_Ki[1], 'Ki_z': wolf_Ki[2],
                'Ki_roll': wolf_Ki[3], 'Ki_pitch': wolf_Ki[4], 'Ki_yaw': wolf_Ki[5],
            })
        except Exception as e:
            self.get_logger().warning(f'⚠️  WandB logging failed: {e}')

    def optimize(self) -> Tuple[np.ndarray, float]:
        """Run GWO optimization

        Returns:
            (best_gains, best_cost): Tuple of best PID gains and corresponding cost
        """
        # Check for checkpoint resume
        start_iteration = 0
        checkpoint_data = None
        if self.checkpoint_file:
            checkpoint_data = self.load_checkpoint(self.checkpoint_file)

        if checkpoint_data:
            # Validate checkpoint data
            try:
                required_keys = ['wolves', 'alpha_wolf', 'alpha_score', 'beta_score', 'delta_score',
                                 'metadata', 'prev_alpha_score', 'no_improvement_count']
                for key in required_keys:
                    if key not in checkpoint_data:
                        raise ValueError(f'Missing required key: {key}')

                # Resume from checkpoint
                self.get_logger().info('=' * 60)
                self.get_logger().info('🔄 Resuming GWO Optimization from Checkpoint')
                self.get_logger().info('=' * 60)

                wolves = np.array(checkpoint_data['wolves'])

                # Validate wolves shape
                if wolves.shape != (self.n_wolves, self.dim):
                    raise ValueError(f'Invalid wolves shape: {wolves.shape}, expected ({self.n_wolves}, {self.dim})')

                self.alpha_wolf = np.array(checkpoint_data['alpha_wolf']) if checkpoint_data['alpha_wolf'] is not None else None
                self.beta_wolf = np.array(checkpoint_data['beta_wolf']) if checkpoint_data['beta_wolf'] is not None else None
                self.delta_wolf = np.array(checkpoint_data['delta_wolf']) if checkpoint_data['delta_wolf'] is not None else None

                # Validate leader wolves shape
                if self.alpha_wolf is not None and self.alpha_wolf.shape != (self.dim,):
                    raise ValueError(f'Invalid alpha_wolf shape: {self.alpha_wolf.shape}, expected ({self.dim},)')

                self.alpha_score = checkpoint_data['alpha_score']
                self.beta_score = checkpoint_data['beta_score']
                self.delta_score = checkpoint_data['delta_score']
                start_iteration = checkpoint_data['metadata']['iteration']
                prev_alpha_score = checkpoint_data['prev_alpha_score']
                no_improvement_count = checkpoint_data['no_improvement_count']

            except Exception as e:
                self.get_logger().error(f'❌ Checkpoint validation failed: {e}')
                self.get_logger().error('   Starting fresh instead...')
                checkpoint_data = None
                wolves = self.initialize_wolves()
                prev_alpha_score = float('inf')
                no_improvement_count = 0
                start_iteration = 0

            # Restore random state and log success (only if checkpoint_data is still valid)
            if checkpoint_data:
                if 'random_state' in checkpoint_data:
                    # Convert list back to proper format (string, numpy array, int, int, float)
                    state_data = checkpoint_data['random_state']
                    restored_state = (
                        state_data[0],  # 'MT19937' string
                        np.array(state_data[1], dtype=np.uint32),  # keys array
                        state_data[2],  # pos
                        state_data[3],  # has_gauss
                        state_data[4]   # cached_gaussian
                    )
                    np.random.set_state(restored_state)

                self.get_logger().info(f'✅ Resumed from iteration {start_iteration}')
                self.get_logger().info(f'   Best cost: {self.alpha_score:.2f}')

        else:
            # Initialize wolves
            wolves = self.initialize_wolves()

            self.get_logger().info('=' * 60)
            self.get_logger().info('🚀 Starting GWO Optimization')
            self.get_logger().info('=' * 60)

            # Early stopping variables
            prev_alpha_score = float('inf')
            no_improvement_count = 0

        start_time = time.time()
        # Relative improvement threshold (文献基盤)
        # Ref: ScienceDirect - "Threshold should be relative to current fitness"
        # Ref: ML Best Practice - "Improve by at least a certain fraction"
        relative_improvement_threshold = 0.01  # 1% relative improvement required

        # Main optimization loop
        for iteration in range(start_iteration, self.max_iterations):
            # Check for interrupt
            if self.is_interrupted():
                self.get_logger().warning('⚠️  Optimization interrupted by user')
                # Save checkpoint before exiting
                self.get_logger().info('💾 Saving checkpoint before exit...')
                # Get random state and convert to serializable format
                rng_state = np.random.get_state()
                random_state_serializable = [
                    rng_state[0],  # 'MT19937' string
                    rng_state[1].tolist(),  # keys array -> list of Python ints
                    int(rng_state[2]),  # pos -> Python int
                    int(rng_state[3]),  # has_gauss -> Python int
                    float(rng_state[4])  # cached_gaussian -> Python float
                ]

                checkpoint_data = {
                    'wolves': wolves.tolist(),
                    'alpha_wolf': self.alpha_wolf.tolist() if self.alpha_wolf is not None else None,
                    'beta_wolf': self.beta_wolf.tolist() if self.beta_wolf is not None else None,
                    'delta_wolf': self.delta_wolf.tolist() if self.delta_wolf is not None else None,
                    'alpha_score': float(self.alpha_score),
                    'beta_score': float(self.beta_score),
                    'delta_score': float(self.delta_score),
                    'prev_alpha_score': float(prev_alpha_score),
                    'no_improvement_count': int(no_improvement_count),
                    'random_state': random_state_serializable,
                }
                self.save_checkpoint(checkpoint_data, iteration)
                break

            # Nonlinear convergence factor: more exploration early, faster exploitation late
            # Ref: MDPI 2023, PMC 2024 - "Nonlinear better balances exploration/exploitation"
            # Standard GWO: a = 2 - 2*t/T (linear)
            # Improved: a = 2 * (1 - (t/T)^2) (nonlinear, more time for exploration)
            progress = iteration / self.max_iterations
            a = 2.0 * (1.0 - progress ** 2)  # Nonlinear: slower decrease early, faster late

            self.get_logger().info('=' * 70)
            self.get_logger().info(f'🔍 Iteration {iteration + 1}/{self.max_iterations} | a={a:.3f}')
            self.get_logger().info(f'   Best Cost: {self.alpha_score:.2f}')
            self.get_logger().info('=' * 70)

            # Evaluate all wolves and store fitness values
            fitness_values = []
            for i in range(self.n_wolves):
                # Check for interrupt
                if self.is_interrupted():
                    self.get_logger().warning('⚠️  Stopping wolf evaluation')
                    break

                self.get_logger().info(f'🐺 Wolf {i + 1}/{self.n_wolves}')
                fitness = self.evaluate_wolf(wolves[i], i)
                fitness_values.append(fitness)

                # Update leaders
                marker = self.update_leaders(wolves[i], fitness, iteration, i)

                # Log
                self.get_logger().info(f'  ➜ Cost={fitness:.2f} {marker}')
                self.get_logger().info('  ' + '─' * 66)

                # WandB logging
                self._log_wolf_to_wandb(wolves[i], fitness)

            # Check if interrupted or no valid leaders found
            if self.is_interrupted() or self.alpha_wolf is None:
                if self.is_interrupted():
                    self.get_logger().warning('⚠️  Stopping optimization')
                break

            # Elimination mechanism: Remove worst wolves and re-initialize
            # Based on Nature 2019 paper: "Improved GWO with Elimination Mechanism"
            # Ref: Nature 2019 - "After each iteration" but we use every 3 for efficiency
            # Applied every 3 iterations to maintain diversity without excessive overhead
            if iteration > 0 and iteration % 3 == 0 and len(fitness_values) == self.n_wolves:
                # Eliminate worst 15% wolves (balanced diversity vs convergence)
                n_eliminate = max(1, int(0.15 * self.n_wolves))
                fitness_array = np.array(fitness_values)

                # Find indices of worst wolves (highest cost)
                worst_indices = np.argsort(fitness_array)[-n_eliminate:]

                # Re-initialize worst wolves with random positions
                for idx in worst_indices:
                    wolves[idx] = self.lb + np.random.rand(self.dim) * (self.ub - self.lb)

                self.get_logger().info(f'♻️  Eliminated {n_eliminate} worst wolves ({int(100*n_eliminate/self.n_wolves)}%), re-initialized randomly')

            # Apply Cauchy-Gaussian mutation to leader wolves
            # Probability: high early (50%), low late (20%)
            mutation_prob = 0.5 - 0.3 * (iteration / self.max_iterations)

            if np.random.rand() < mutation_prob and self.n_wolves >= 3:
                # Mutate alpha wolf and replace last wolf for evaluation
                mutated_alpha = self.cauchy_gaussian_mutation(
                    self.alpha_wolf, iteration, self.max_iterations,
                    self.alpha_score, self.alpha_score
                )
                wolves[-1] = mutated_alpha  # Will be evaluated in next iteration

                # Mutate beta wolf and replace second-to-last wolf
                if self.beta_wolf is not None and self.n_wolves >= 4:
                    mutated_beta = self.cauchy_gaussian_mutation(
                        self.beta_wolf, iteration, self.max_iterations,
                        self.beta_score, self.alpha_score
                    )
                    wolves[-2] = mutated_beta

                # Mutate delta wolf and replace third-to-last wolf
                if self.delta_wolf is not None and self.n_wolves >= 5:
                    mutated_delta = self.cauchy_gaussian_mutation(
                        self.delta_wolf, iteration, self.max_iterations,
                        self.delta_score, self.alpha_score
                    )
                    wolves[-3] = mutated_delta

                self.get_logger().info(f'🧬 Cauchy-Gaussian mutation applied to leaders (prob={mutation_prob:.2f})')

            # Update wolf positions based on alpha, beta, delta
            # Adaptive random jump: balanced exploration-exploitation
            # Ref: ScienceDirect 2024 - "Exploitation over 90% recommended"
            # Early iterations (0-20%): 25% random jumps + 15% Lévy = 40% exploration, 60% exploitation
            # Mid iterations (20-50%): 15% random + 15% Lévy = 30% exploration, 70% exploitation
            # Late iterations (50-100%): 5% random + 15% Lévy = 20% exploration, 80% exploitation
            progress = iteration / self.max_iterations
            if progress < 0.2:
                random_jump_prob = 0.25  # Early: moderate exploration
            elif progress < 0.5:
                random_jump_prob = 0.15  # Mid: balanced
            else:
                random_jump_prob = 0.05  # Late: heavy exploitation

            levy_flight_prob = 0.15  # Lévy flight for medium-range jumps (consistent)
            n_levy_wolves = 0
            n_random_jumps = 0

            for i in range(self.n_wolves):
                # Three exploration strategies (prioritized):
                # 1. Random jump (15%): Completely random position in search space
                # 2. Lévy flight (20%): Large jumps from current position
                # 3. Standard GWO (65%): Guided by leaders

                rand_val = np.random.rand()

                if rand_val < random_jump_prob:
                    # Random jump: completely new random position across full search space
                    # This prevents wolves from getting stuck in local regions
                    wolves[i] = self.lb + np.random.rand(self.dim) * (self.ub - self.lb)
                    n_random_jumps += 1
                elif rand_val < (random_jump_prob + levy_flight_prob):
                    # Lévy flight: large jumps for exploration
                    levy_step = self.levy_flight(beta=1.5)
                    step_size = 0.01 * (self.ub - self.lb)  # Scale by search space
                    wolves[i] = wolves[i] + levy_step * step_size
                    n_levy_wolves += 1
                else:
                    # Standard GWO update with adaptive step size
                    # Adaptive distance: ensures minimum exploration even when alpha ≈ 0
                    # Reference scale = search space range
                    search_range = self.ub - self.lb

                    r1 = np.random.rand(self.dim)
                    r2 = np.random.rand(self.dim)
                    A1 = 2 * a * r1 - a
                    C1 = 2 * r2
                    D_alpha = np.abs(C1 * self.alpha_wolf - wolves[i])
                    # Add minimum step proportional to search range (10%)
                    D_alpha = np.maximum(D_alpha, 0.1 * search_range)
                    X1 = self.alpha_wolf - A1 * D_alpha

                    r1 = np.random.rand(self.dim)
                    r2 = np.random.rand(self.dim)
                    A2 = 2 * a * r1 - a
                    C2 = 2 * r2
                    D_beta = np.abs(C2 * self.beta_wolf - wolves[i])
                    D_beta = np.maximum(D_beta, 0.1 * search_range)
                    X2 = self.beta_wolf - A2 * D_beta

                    r1 = np.random.rand(self.dim)
                    r2 = np.random.rand(self.dim)
                    A3 = 2 * a * r1 - a
                    C3 = 2 * r2
                    D_delta = np.abs(C3 * self.delta_wolf - wolves[i])
                    D_delta = np.maximum(D_delta, 0.1 * search_range)
                    X3 = self.delta_wolf - A3 * D_delta

                    # Update position (average of three components)
                    wolves[i] = (X1 + X2 + X3) / 3.0

                # Enforce bounds
                wolves[i] = np.clip(wolves[i], self.lb, self.ub)

            if n_random_jumps > 0 or n_levy_wolves > 0:
                self.get_logger().info(
                    f'🎲 Exploration - Random: {n_random_jumps}/{self.n_wolves} ({random_jump_prob:.0%}), '
                    f'Lévy: {n_levy_wolves}/{self.n_wolves} ({levy_flight_prob:.0%})'
                )

            # Estimate remaining time
            elapsed = time.time() - start_time
            avg_time_per_iter = elapsed / (iteration + 1)
            remaining_iters = self.max_iterations - (iteration + 1)
            eta_seconds = avg_time_per_iter * remaining_iters
            eta_hours = eta_seconds / 3600
            eta_minutes = (eta_seconds % 3600) / 60
            self.get_logger().info(f'⏱️  ETA: {int(eta_hours)}h {int(eta_minutes)}m')

            # Iteration summary
            self.get_logger().info('─' * 70)
            self.get_logger().info(f'📊 Iteration {iteration + 1} Summary:')
            self.get_logger().info(f'   🥇 Alpha (Best):  {self.alpha_score:.2f}')
            if self.beta_score != float('inf'):
                self.get_logger().info(f'   🥈 Beta (2nd):    {self.beta_score:.2f}')
            if self.delta_score != float('inf'):
                self.get_logger().info(f'   🥉 Delta (3rd):   {self.delta_score:.2f}')
            self.get_logger().info('─' * 70)

            # Save checkpoint
            if (iteration + 1) % self.checkpoint_interval == 0:
                # Get random state and convert to serializable format
                rng_state = np.random.get_state()
                random_state_serializable = [
                    rng_state[0],  # 'MT19937' string
                    rng_state[1].tolist(),  # keys array -> list of Python ints
                    int(rng_state[2]),  # pos -> Python int
                    int(rng_state[3]),  # has_gauss -> Python int
                    float(rng_state[4])  # cached_gaussian -> Python float
                ]

                checkpoint_data = {
                    'wolves': wolves.tolist(),
                    'alpha_wolf': self.alpha_wolf.tolist() if self.alpha_wolf is not None else None,
                    'beta_wolf': self.beta_wolf.tolist() if self.beta_wolf is not None else None,
                    'delta_wolf': self.delta_wolf.tolist() if self.delta_wolf is not None else None,
                    'alpha_score': float(self.alpha_score),
                    'beta_score': float(self.beta_score),
                    'delta_score': float(self.delta_score),
                    'prev_alpha_score': float(prev_alpha_score),
                    'no_improvement_count': int(no_improvement_count),
                    'random_state': random_state_serializable,
                }
                self.save_checkpoint(checkpoint_data, iteration + 1)

            self.get_logger().info('')  # Empty line for separation

            # Early stopping check (after minimum iterations)
            # Ref: ScienceDirect - "Improvement-type criteria recommended"
            # Ref: PSO literature - "100+ iterations typical, but PID tuning converges faster"
            min_iterations = 15  # Minimum iterations before early stopping

            if iteration >= min_iterations:
                # Calculate relative improvement (scale-independent)
                improvement = prev_alpha_score - self.alpha_score

                # Relative improvement (percentage)
                if abs(prev_alpha_score) > 1e-6:
                    relative_improvement = improvement / abs(prev_alpha_score)
                else:
                    relative_improvement = 0.0

                # Adaptive patience: more patient early, stricter late
                progress = iteration / self.max_iterations
                if progress < 0.5:
                    patience = 7  # Early: allow more stagnation
                else:
                    patience = 5  # Late: converge faster

                if relative_improvement < relative_improvement_threshold:
                    no_improvement_count += 1
                    self.get_logger().info(f'⚠️  No significant improvement ({relative_improvement*100:.2f}% < {relative_improvement_threshold*100:.1f}%): {no_improvement_count}/{patience}')
                    if no_improvement_count >= patience:
                        self.get_logger().info('✅ Early stopping: Converged!')
                        break
                else:
                    no_improvement_count = 0  # Reset counter
                    self.get_logger().info(f'✓ Improvement: {relative_improvement*100:.2f}% (threshold: {relative_improvement_threshold*100:.1f}%)')

                prev_alpha_score = self.alpha_score

        # Final results
        self.get_logger().info('=' * 60)
        if self.is_interrupted():
            self.get_logger().info('⚠️  OPTIMIZATION INTERRUPTED')
        else:
            self.get_logger().info('🎉 OPTIMIZATION COMPLETE!')
        self.get_logger().info('=' * 60)

        # Check if we have valid results
        if self.alpha_wolf is None or self.alpha_score == float('inf'):
            self.get_logger().error('❌ No valid solution found')
            return None, float('inf')

        self.get_logger().info(f'Training Cost: {self.alpha_score:.2f}')

        # Validate on validation scenarios (skip if interrupted)
        validation_cost = None
        if not self.is_interrupted() and len(self.validation_scenarios) > 0:
            self.get_logger().info(f'\n📊 Validating on {len(self.validation_scenarios)} scenarios...')
            validation_cost = self.evaluate_on_scenarios(self.alpha_wolf, self.validation_scenarios)
            self.get_logger().info(f'   Validation Cost: {validation_cost:.2f}')

        # Log best gains
        self.get_logger().info(f'\n🎯 Best Gains:')
        self.get_logger().info(f'  Kp: {self.alpha_wolf[0:6]}')
        self.get_logger().info(f'  Kd: {self.alpha_wolf[6:12]}')
        self.get_logger().info(f'  Ki: {self.alpha_wolf[12:18]}')

        # WandB summary
        if self.use_wandb and WANDB_AVAILABLE:
            wandb.run.summary['final_cost'] = self.alpha_score
            wandb.run.summary['validation_cost'] = validation_cost
            wandb.run.summary['interrupted'] = self.is_interrupted()
            wandb.run.summary['best_Kp'] = self.alpha_wolf[0:6].tolist()
            wandb.run.summary['best_Kd'] = self.alpha_wolf[6:12].tolist()
            wandb.run.summary['best_Ki'] = self.alpha_wolf[12:18].tolist()
            self.get_logger().info(f'📊 Results logged to WandB: {wandb.run.url}')

        # Save results
        self.save_results(self.alpha_wolf, self.alpha_score)

        # Cleanup (finish WandB run)
        self.cleanup()

        return self.alpha_wolf, self.alpha_score
