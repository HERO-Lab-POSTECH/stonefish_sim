#!/usr/bin/env python3
# Copyright 2025

"""
PID Data Plotter

Generates plots from logged PID data.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path


class DataPlotter:
    """Plotter for PID tuning data.

    Generates publication-quality plots showing:
    - Position errors over time
    - Orientation errors over time
    """

    def __init__(self, csv_file: str):
        """Initialize plotter with data file.

        Args:
            csv_file: Path to CSV file with logged data
        """
        self.csv_file = Path(csv_file)
        self.df = pd.read_csv(csv_file)

    def plot(self, output_file: str = None) -> str:
        """Generate plots and save to file.

        Args:
            output_file: Path to save plot (default: <csv_file>_plot.png)

        Returns:
            Path to saved plot file
        """
        if output_file is None:
            output_file = str(self.csv_file).replace('.csv', '_plot.png')

        fig = self._create_figure()
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        plt.close(fig)

        return output_file

    def _create_figure(self):
        """Create matplotlib figure with subplots.

        Returns:
            matplotlib figure object
        """
        fig, axes = plt.subplots(2, 3, figsize=(18, 10))
        fig.suptitle('PID Tuning Results', fontsize=16, fontweight='bold')

        # Position errors (top row)
        for i, axis in enumerate(['x', 'y', 'z']):
            ax = axes[0, i]
            ax.plot(self.df['timestamp'], self.df[f'err_{axis}'],
                    linewidth=1.5, color='steelblue')
            ax.axhline(0, color='red', linestyle='--', alpha=0.5, linewidth=1)
            ax.set_title(f'Position Error - {axis.upper()}', fontsize=12, fontweight='bold')
            ax.set_ylabel('Error (m)', fontsize=10)
            ax.grid(True, alpha=0.3)

            # Add RMS annotation
            rms = np.sqrt((self.df[f'err_{axis}']**2).mean())
            ax.text(0.02, 0.98, f'RMS: {rms:.4f}m',
                    transform=ax.transAxes, fontsize=9,
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # Orientation errors (bottom row)
        for i, axis in enumerate(['roll', 'pitch', 'yaw']):
            ax = axes[1, i]
            error_deg = self.df[f'err_{axis}'] * 180 / np.pi
            ax.plot(self.df['timestamp'], error_deg,
                    linewidth=1.5, color='darkorange')
            ax.axhline(0, color='red', linestyle='--', alpha=0.5, linewidth=1)
            ax.set_title(f'Orientation Error - {axis.upper()}', fontsize=12, fontweight='bold')
            ax.set_ylabel('Error (deg)', fontsize=10)
            ax.set_xlabel('Time (s)', fontsize=10)
            ax.grid(True, alpha=0.3)

            # Add RMS annotation
            rms_rad = np.sqrt((self.df[f'err_{axis}']**2).mean())
            rms_deg = np.rad2deg(rms_rad)
            ax.text(0.02, 0.98, f'RMS: {rms_deg:.2f}°',
                    transform=ax.transAxes, fontsize=9,
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()
        return fig

    def show(self):
        """Display plot interactively (for manual inspection)."""
        self._create_figure()
        plt.show()
