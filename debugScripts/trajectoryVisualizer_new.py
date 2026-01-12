#!/usr/bin/env python3
"""
Trajectory Interpolation Comparison Visualizer

Compares linear and spline interpolation methods on the same graph.
Groups tests by input case (e.g., case1_position_only) and plots both
interpolation methods together for easy comparison.

Usage:
    python3 trajectoryVisualizer_new.py [log_file]
"""

import sys
import re
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Dict, Optional
import argparse
from collections import defaultdict


@dataclass
class TrajectoryPoint:
    time: float
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    efforts: List[float] = field(default_factory=list)
    accelerations: List[float] = field(default_factory=list)


@dataclass
class SampledPoint:
    time: float
    pos: List[float] = field(default_factory=list)
    vel: List[float] = field(default_factory=list)
    acc: List[float] = field(default_factory=list)
    eff: List[float] = field(default_factory=list)
    cmd_pos: List[float] = field(default_factory=list)
    cmd_vel: List[float] = field(default_factory=list)
    cmd_eff: List[float] = field(default_factory=list)


@dataclass
class TrajectoryLog:
    config_name: str = ""
    interpolation_method: str = "splines"
    num_joints: int = 0
    joint_names: List[str] = field(default_factory=list)
    input_points: List[TrajectoryPoint] = field(default_factory=list)
    sampled_points: List[SampledPoint] = field(default_factory=list)


def clean_line(line: str) -> str:
    """Remove CTest prefix (e.g., '3:  ') from line."""
    cleaned = re.sub(r'^\d+:\s*', '', line)
    return cleaned.strip()


def parse_float_list(s: str) -> List[float]:
    """Parse comma-separated floats from string."""
    s = s.strip()
    if not s:
        return []
    try:
        return [float(x.strip()) for x in s.split(',') if x.strip()]
    except ValueError:
        return []


def parse_log(log_text: str) -> List[TrajectoryLog]:
    """Parse the trajectory log text and return list of TrajectoryLog objects."""
    logs = []
    current_log = None
    current_input_point = None
    current_sample = None
    in_input_section = False
    in_sampled_section = False

    lines = log_text.strip().split('\n')
    
    for raw_line in lines:
        line = clean_line(raw_line)
        
        if not line:
            continue
        
        # New log section
        if line.startswith("CONFIG:"):
            if current_log is not None:
                logs.append(current_log)
            current_log = TrajectoryLog()
            current_log.config_name = line.replace("CONFIG:", "").strip()
            in_input_section = False
            in_sampled_section = False
            
        elif line.startswith("INTERPOLATION:"):
            if current_log:
                current_log.interpolation_method = line.replace("INTERPOLATION:", "").strip()
            
        elif line == "--- INPUT_TRAJECTORY_START ---":
            in_input_section = True
            in_sampled_section = False
            
        elif line == "--- INPUT_TRAJECTORY_END ---":
            in_input_section = False
            if current_input_point is not None:
                current_log.input_points.append(current_input_point)
                current_input_point = None
                
        elif line == "--- SAMPLED_TRAJECTORY_START ---":
            in_sampled_section = True
            in_input_section = False
            
        elif line == "--- SAMPLED_TRAJECTORY_END ---":
            in_sampled_section = False
            if current_sample is not None:
                current_log.sampled_points.append(current_sample)
                current_sample = None
                
        elif current_log is not None:
            # Parse input trajectory section
            if in_input_section:
                if line.startswith("NUM_JOINTS:"):
                    current_log.num_joints = int(line.replace("NUM_JOINTS:", "").strip())
                elif line.startswith("JOINT_NAMES:"):
                    names = line.replace("JOINT_NAMES:", "").strip()
                    current_log.joint_names = [n.strip() for n in names.split(',')]
                elif line.startswith("POINT "):
                    if current_input_point is not None:
                        current_log.input_points.append(current_input_point)
                    current_input_point = TrajectoryPoint(time=0.0)
                elif line.startswith("TIME:") and current_input_point is not None:
                    current_input_point.time = float(line.replace("TIME:", "").strip())
                elif line.startswith("POSITIONS:") and current_input_point is not None:
                    current_input_point.positions = parse_float_list(line.replace("POSITIONS:", ""))
                elif line.startswith("VELOCITIES:") and current_input_point is not None:
                    current_input_point.velocities = parse_float_list(line.replace("VELOCITIES:", ""))
                elif line.startswith("EFFORTS:") and current_input_point is not None:
                    current_input_point.efforts = parse_float_list(line.replace("EFFORTS:", ""))
                        
            # Parse sampled trajectory section
            elif in_sampled_section:
                if line.startswith("SAMPLE "):
                    if current_sample is not None:
                        current_log.sampled_points.append(current_sample)
                    current_sample = SampledPoint(time=0.0)
                elif line.startswith("TIME:") and current_sample is not None:
                    current_sample.time = float(line.replace("TIME:", "").strip())
                elif line.startswith("POS:") and current_sample is not None:
                    current_sample.pos = parse_float_list(line.replace("POS:", ""))
                elif line.startswith("VEL:") and current_sample is not None:
                    current_sample.vel = parse_float_list(line.replace("VEL:", ""))
                elif line.startswith("EFF:") and current_sample is not None:
                    current_sample.eff = parse_float_list(line.replace("EFF:", ""))
                elif line.startswith("ACC:") and current_sample is not None:
                    current_sample.acc = parse_float_list(line.replace("ACC:", ""))
                elif line.startswith("CMD_POS:") and current_sample is not None:
                    current_sample.cmd_pos = parse_float_list(line.replace("CMD_POS:", ""))
                elif line.startswith("CMD_VEL:") and current_sample is not None:
                    current_sample.cmd_vel = parse_float_list(line.replace("CMD_VEL:", ""))
                elif line.startswith("CMD_EFF:") and current_sample is not None:
                    current_sample.cmd_eff = parse_float_list(line.replace("CMD_EFF:", ""))

    # Don't forget the last log
    if current_log is not None:
        if current_input_point is not None:
            current_log.input_points.append(current_input_point)
        if current_sample is not None:
            current_log.sampled_points.append(current_sample)
        logs.append(current_log)

    return logs


def group_logs_by_case(logs: List[TrajectoryLog]) -> Dict[str, Dict[str, TrajectoryLog]]:
    """Group logs by test case name and interpolation method."""
    grouped = defaultdict(dict)
    for log in logs:
        case_name = log.config_name
        interp_method = log.interpolation_method
        grouped[case_name][interp_method] = log
    return grouped


def extract_data(log: TrajectoryLog, joint_idx: int = 0):
    """Extract numpy arrays from log data for a specific joint."""
    if not log.sampled_points:
        return None
    
    n_samples = len(log.sampled_points)
    n_inputs = len(log.input_points)
    
    data = {
        'times': np.array([s.time for s in log.sampled_points]),
        'pos': np.array([s.pos[joint_idx] if joint_idx < len(s.pos) else 0.0 for s in log.sampled_points]),
        'vel': np.array([s.vel[joint_idx] if joint_idx < len(s.vel) else 0.0 for s in log.sampled_points]),
        'acc': np.array([s.acc[joint_idx] if joint_idx < len(s.acc) else 0.0 for s in log.sampled_points]),
        'eff': np.array([s.eff[joint_idx] if joint_idx < len(s.eff) else 0.0 for s in log.sampled_points]),
        'input_times': np.array([p.time for p in log.input_points]) if n_inputs > 0 else np.array([]),
        'input_pos': np.array([p.positions[joint_idx] if joint_idx < len(p.positions) else 0.0 for p in log.input_points]) if n_inputs > 0 else np.array([]),
        'input_vel': np.array([p.velocities[joint_idx] if joint_idx < len(p.velocities) else 0.0 for p in log.input_points]) if n_inputs > 0 else np.array([]),
        'input_eff': np.array([p.efforts[joint_idx] if joint_idx < len(p.efforts) else 0.0 for p in log.input_points]) if n_inputs > 0 else np.array([]),
        'has_input_vel': any(p.velocities for p in log.input_points),
        'has_input_eff': any(p.efforts for p in log.input_points),
    }
    
    return data


def plot_comparison(case_name: str, logs_dict: Dict[str, TrajectoryLog], joint_idx: int = 0, save_path: Optional[str] = None):
    """
    Plot linear and spline interpolation comparison for a single test case.
    Shows 4 subplots: Position, Velocity, Acceleration, Effort.
    """
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f'{case_name} - Interpolation Comparison (Joint {joint_idx})', 
                 fontsize=14, fontweight='bold')
    
    # Color scheme
    color_pos = '#1f77b4'  # Blue
    color_vel = '#2ca02c'  # Green
    color_acc = '#ff7f0e'  # Orange
    color_eff = '#9467bd'  # Purple
    color_waypoint = '#d62728'  # Red
    
    linear_log = logs_dict.get('linear')
    spline_log = logs_dict.get('splines')
    
    # Use whichever is available for waypoint markers
    reference_log = spline_log if spline_log else linear_log
    if not reference_log:
        print(f"No data for case {case_name}")
        return None
    
    ref_data = extract_data(reference_log, joint_idx)
    if ref_data is None:
        return None
    
    # Plot linear interpolation (if available)
    if linear_log:
        linear_data = extract_data(linear_log, joint_idx)
        if linear_data:
            axes[0].plot(linear_data['times'], linear_data['pos'], ':', color=color_pos, 
                        linewidth=3, label='Linear', alpha=0.9)
            axes[1].plot(linear_data['times'], linear_data['vel'], ':', color=color_vel, 
                        linewidth=3, label='Linear', alpha=0.9)
            axes[2].plot(linear_data['times'], linear_data['acc'], ':', color=color_acc, 
                        linewidth=3, label='Linear', alpha=0.9)
            axes[3].plot(linear_data['times'], linear_data['eff'], ':', color=color_eff, 
                        linewidth=3, label='Linear', alpha=0.9)
    
    # Plot spline interpolation (if available)
    if spline_log:
        spline_data = extract_data(spline_log, joint_idx)
        if spline_data:
            axes[0].plot(spline_data['times'], spline_data['pos'], '-', color=color_pos, 
                        linewidth=2.5, label='Spline')
            axes[1].plot(spline_data['times'], spline_data['vel'], '-', color=color_vel, 
                        linewidth=2.5, label='Spline')
            axes[2].plot(spline_data['times'], spline_data['acc'], '-', color=color_acc, 
                        linewidth=2.5, label='Spline')
            axes[3].plot(spline_data['times'], spline_data['eff'], '-', color=color_eff, 
                        linewidth=2.5, label='Spline')
    
    # Add waypoint markers (using reference data)
    if len(ref_data['input_times']) > 0:
        # Position waypoints (always shown)
        axes[0].scatter(ref_data['input_times'], ref_data['input_pos'], 
                       color=color_waypoint, s=100, marker='o', 
                       edgecolors='black', linewidths=1.5, zorder=5, label='Waypoints')
        
        # Velocity waypoints (only if specified)
        if ref_data['has_input_vel']:
            axes[1].scatter(ref_data['input_times'], ref_data['input_vel'], 
                           color=color_waypoint, s=100, marker='^', 
                           edgecolors='black', linewidths=1.5, zorder=5, label='Specified')
        
        # Effort waypoints (only if specified)
        if ref_data['has_input_eff']:
            axes[3].scatter(ref_data['input_times'], ref_data['input_eff'], 
                           color=color_waypoint, s=100, marker='s', 
                           edgecolors='black', linewidths=1.5, zorder=5, label='Specified')
    
    # Configure axes
    axes[0].set_ylabel('Position', fontsize=11, fontweight='bold')
    axes[0].legend(loc='upper right', fontsize=10)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Position', fontsize=11, loc='left', fontweight='bold')
    
    axes[1].set_ylabel('Velocity', fontsize=11, fontweight='bold')
    axes[1].axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    axes[1].legend(loc='upper right', fontsize=10)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('Velocity', fontsize=11, loc='left', fontweight='bold')
    
    axes[2].set_ylabel('Acceleration', fontsize=11, fontweight='bold')
    axes[2].axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    axes[2].legend(loc='upper right', fontsize=10)
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title('Acceleration', fontsize=11, loc='left', fontweight='bold')
    
    axes[3].set_ylabel('Effort', fontsize=11, fontweight='bold')
    axes[3].set_xlabel('Time [s]', fontsize=11, fontweight='bold')
    axes[3].axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    axes[3].legend(loc='upper right', fontsize=10)
    axes[3].grid(True, alpha=0.3)
    axes[3].set_title('Effort', fontsize=11, loc='left', fontweight='bold')
    
    # Add vertical lines at waypoint times
    for t in ref_data['input_times']:
        for ax in axes:
            ax.axvline(x=t, color='gray', linestyle=':', alpha=0.4)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def main():
    parser = argparse.ArgumentParser(
        description='Visualize trajectory interpolation comparison (linear vs spline)')
    parser.add_argument('log_file', nargs='?', default=None,
                       help='Log file to parse. If not provided, reads from stdin.')
    parser.add_argument('--save', '-s', type=str, default=None,
                       help='Save plots to files with this prefix')
    parser.add_argument('--no-show', action='store_true',
                       help='Do not display plots (only save)')
    parser.add_argument('--joint', '-j', type=int, default=0,
                       help='Joint index for plots (default: 0)')
    parser.add_argument('--list', '-l', action='store_true',
                       help='List found configurations without plotting')
    args = parser.parse_args()
    
    # Read log data
    if args.log_file:
        print(f"Reading from file: {args.log_file}")
        with open(args.log_file, 'r') as f:
            log_text = f.read()
    else:
        print("Reading from stdin (paste log and press Ctrl+D when done)...")
        log_text = sys.stdin.read()
    
    # Parse logs
    logs = parse_log(log_text)
    
    if not logs:
        print("No trajectory logs found!")
        print("\nExpected format with lines like:")
        print("  CONFIG: case1_position_only")
        print("  INTERPOLATION: linear")
        print("  --- INPUT_TRAJECTORY_START ---")
        print("  --- SAMPLED_TRAJECTORY_START ---")
        return 1
    
    # Group logs by case
    grouped = group_logs_by_case(logs)
    
    print(f"\nFound {len(logs)} trajectory log(s) in {len(grouped)} test case(s):")
    for case_name, methods in grouped.items():
        methods_str = ", ".join(methods.keys())
        print(f"  - {case_name}: {methods_str}")
    
    if args.list:
        return 0
    
    # Generate comparison plots for each case
    for case_name, logs_dict in grouped.items():
        print(f"\nPlotting: {case_name}")
        
        safe_name = re.sub(r'[^\w\-_]', '_', case_name)
        save_path = f"{args.save}_{safe_name}_joint{args.joint}.png" if args.save else None
        
        plot_comparison(case_name, logs_dict, args.joint, save_path)
    
    if not args.no_show:
        plt.show()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
