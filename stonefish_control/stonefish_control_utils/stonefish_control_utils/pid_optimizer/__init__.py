"""
PID Optimizer Package

Unified package for PID gain optimization using different methods.
Supports GWO (Grey Wolf Optimizer) and SMAC3 (Bayesian Optimization).
"""

from .base import BaseOptimizer, create_wandb_run_name, load_yaml_config
from .gwo import GWOOptimizer
from .smac3 import SMAC3Optimizer
from .node import PIDOptimizerNode, main

__all__ = [
    'BaseOptimizer',
    'GWOOptimizer',
    'SMAC3Optimizer',
    'PIDOptimizerNode',
    'main',
    'create_wandb_run_name',
    'load_yaml_config',
]

__version__ = '1.0.0'
