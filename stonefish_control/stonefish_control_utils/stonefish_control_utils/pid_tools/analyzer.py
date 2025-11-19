#!/usr/bin/env python3
# Copyright 2025

"""
PID Data Analyzer

Analyzes logged PID data and computes performance metrics.
"""

import numpy as np
import pandas as pd
from typing import Dict


class DataAnalyzer:
    """Analyzer for PID tuning data.

    Computes performance metrics from logged data:
    - RMS error (position and orientation)
    - Oscillation count (zero crossings)
    - Settling time
    - Overshoot
    """

    def __init__(self, csv_file: str):
        """Initialize analyzer with data file.

        Args:
            csv_file: Path to CSV file with logged data
        """
        self.df = pd.read_csv(csv_file)
        self.metrics = None

    def analyze(self) -> Dict:
        """Analyze data and compute metrics.

        Returns:
            Dictionary with analysis results
        """
        metrics = {
            'samples': len(self.df),
            'duration': self.df['timestamp'].max(),
            'position_rms': self._compute_position_rms(),
            'orientation_rms': self._compute_orientation_rms(),
            'oscillation': self._compute_oscillation(),
        }

        self.metrics = metrics
        return metrics

    def _compute_position_rms(self) -> Dict[str, float]:
        """Compute RMS error for position (X, Y, Z).

        Returns:
            Dict with 'x', 'y', 'z' RMS errors in meters
        """
        rms = {}
        for axis in ['x', 'y', 'z']:
            rms[axis] = np.sqrt((self.df[f'err_{axis}']**2).mean())
        return rms

    def _compute_orientation_rms(self) -> Dict[str, float]:
        """Compute RMS error for orientation (Roll, Pitch, Yaw).

        Returns:
            Dict with 'roll', 'pitch', 'yaw' RMS errors in degrees
        """
        rms = {}
        for axis in ['roll', 'pitch', 'yaw']:
            rms_rad = np.sqrt((self.df[f'err_{axis}']**2).mean())
            rms[axis] = np.rad2deg(rms_rad)
        return rms

    def _compute_oscillation(self) -> Dict[str, int]:
        """Compute oscillation (zero crossings) for each axis.

        Returns:
            Dict with zero crossing counts for each DOF
        """
        oscillation = {}

        # Position axes
        for axis in ['x', 'y', 'z']:
            crossings = np.sum(np.diff(np.sign(self.df[f'err_{axis}'].values)) != 0)
            oscillation[axis] = int(crossings)

        # Orientation axes
        for axis in ['roll', 'pitch', 'yaw']:
            crossings = np.sum(np.diff(np.sign(self.df[f'err_{axis}'].values)) != 0)
            oscillation[axis] = int(crossings)

        return oscillation

    def print_report(self):
        """Print analysis report to console."""
        if self.metrics is None:
            self.analyze()

        print("\n" + "=" * 70)
        print("PID TUNING ANALYSIS REPORT")
        print("=" * 70)
        print(f"Samples: {self.metrics['samples']}")
        print(f"Duration: {self.metrics['duration']:.2f}s")

        print("\nPOSITION ERROR (RMS):")
        for axis in ['x', 'y', 'z']:
            rms = self.metrics['position_rms'][axis]
            print(f"  {axis.upper()}: {rms:.4f} m")

        print("\nORIENTATION ERROR (RMS):")
        for axis in ['roll', 'pitch', 'yaw']:
            rms = self.metrics['orientation_rms'][axis]
            print(f"  {axis.upper()}: {rms:.2f}°")

        print("\nOSCILLATION (Zero Crossings):")
        print("  Position:")
        for axis in ['x', 'y', 'z']:
            count = self.metrics['oscillation'][axis]
            print(f"    {axis.upper()}: {count}")
        print("  Orientation:")
        for axis in ['roll', 'pitch', 'yaw']:
            count = self.metrics['oscillation'][axis]
            print(f"    {axis.upper()}: {count}")

        print("=" * 70)

    def get_summary(self) -> str:
        """Get analysis summary as string.

        Returns:
            Multi-line string with analysis results
        """
        if self.metrics is None:
            self.analyze()

        lines = []
        lines.append("=" * 70)
        lines.append("PID TUNING ANALYSIS")
        lines.append("=" * 70)
        lines.append(f"Duration: {self.metrics['duration']:.2f}s, Samples: {self.metrics['samples']}")

        # Position
        lines.append("\nPosition RMS Error:")
        for axis in ['x', 'y', 'z']:
            rms = self.metrics['position_rms'][axis]
            lines.append(f"  {axis.upper()}: {rms:.4f}m")

        # Orientation
        lines.append("\nOrientation RMS Error:")
        for axis in ['roll', 'pitch', 'yaw']:
            rms = self.metrics['orientation_rms'][axis]
            lines.append(f"  {axis.upper()}: {rms:.2f}°")

        # Oscillation summary
        pos_osc = sum(self.metrics['oscillation'][ax] for ax in ['x', 'y', 'z'])
        ori_osc = sum(self.metrics['oscillation'][ax] for ax in ['roll', 'pitch', 'yaw'])
        lines.append(f"\nOscillation: Position={pos_osc}, Orientation={ori_osc}")

        lines.append("=" * 70)

        return '\n'.join(lines)
