#!/usr/bin/env python3
"""
FANUC Die Scanner Heat Map Visualizer

This program creates heat maps and visualizations from diescan CSV results.
It displays height data as color-coded heat maps with overlays showing
the calculated die center and fitted circle.

Author: Generated for USA Forge Cell project
"""

import os
import re
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import LinearSegmentedColormap
from scipy.interpolate import griddata
import pandas as pd
from pathlib import Path
import argparse
from typing import Dict, List, Tuple, Optional

class DiescanParser:
    """Parser for FANUC diescan CSV result files."""
    
    def __init__(self, csv_file: str):
        self.csv_file = csv_file
        self.summary_data = {}
        self.accuracy_data = {}
        self.scan_points = []
        self.edge_points = []
        self.data_quality = {}
        self.edge_detection = {}
        self.coverage_analysis = {}
        
        self._parse_csv()
    
    def _parse_csv(self):
        """Parse the CSV file and extract all sections."""
        with open(self.csv_file, 'r') as f:
            lines = f.readlines()
        
        current_section = None
        
        for i, line in enumerate(lines):
            line = line.strip()
            
            if not line:
                continue
                
            # Identify sections
            if line == "SUMMARY RESULTS":
                current_section = "summary"
                continue
            elif line == "ACCURACY ANALYSIS":
                current_section = "accuracy"
                continue
            elif line == "DATA QUALITY":
                current_section = "data_quality"
                continue
            elif line == "EDGE DETECTION":
                current_section = "edge_detection"
                continue
            elif line == "COVERAGE ANALYSIS":
                current_section = "coverage"
                continue
            elif line == "SCAN POINTS DATA":
                current_section = "scan_points"
                continue
            elif line == "EDGE POINTS DATA":
                current_section = "edge_points"
                continue
            elif line == "DIE SURFACE GRID":
                current_section = "grid"
                break
            
            # Parse data based on current section
            if current_section == "summary":
                self._parse_summary_line(line)
            elif current_section == "accuracy":
                self._parse_accuracy_line(line)
            elif current_section == "data_quality":
                self._parse_data_quality_line(line)
            elif current_section == "edge_detection":
                self._parse_edge_detection_line(line)
            elif current_section == "coverage":
                self._parse_coverage_line(line)
            elif current_section == "scan_points":
                self._parse_scan_point_line(line)
            elif current_section == "edge_points":
                self._parse_edge_point_line(line)
    
    def _parse_summary_line(self, line: str):
        """Parse summary results section."""
        if ',' in line:
            parts = line.split(',')
            if len(parts) >= 2:
                key = parts[0].strip()
                value = parts[1].strip()
                try:
                    self.summary_data[key] = float(value)
                except ValueError:
                    self.summary_data[key] = value
    
    def _parse_accuracy_line(self, line: str):
        """Parse accuracy analysis section."""
        if ',' in line:
            parts = line.split(',')
            if len(parts) >= 2:
                key = parts[0].strip()
                value = parts[1].strip()
                try:
                    self.accuracy_data[key] = float(value)
                except ValueError:
                    self.accuracy_data[key] = value
    
    def _parse_data_quality_line(self, line: str):
        """Parse data quality section."""
        if ',' in line:
            parts = line.split(',')
            if len(parts) >= 2:
                key = parts[0].strip()
                value = parts[1].strip()
                try:
                    self.data_quality[key] = float(value)
                except ValueError:
                    self.data_quality[key] = value
    
    def _parse_edge_detection_line(self, line: str):
        """Parse edge detection section."""
        if ',' in line:
            parts = line.split(',')
            if len(parts) >= 2:
                key = parts[0].strip()
                value = parts[1].strip()
                try:
                    self.edge_detection[key] = float(value)
                except ValueError:
                    self.edge_detection[key] = value
    
    def _parse_coverage_line(self, line: str):
        """Parse coverage analysis section."""
        if ',' in line:
            parts = line.split(',')
            if len(parts) >= 2:
                key = parts[0].strip()
                value = parts[1].strip()
                try:
                    self.coverage_analysis[key] = float(value)
                except ValueError:
                    self.coverage_analysis[key] = value
    
    def _parse_scan_point_line(self, line: str):
        """Parse scan points data."""
        if line.startswith("Point_ID,"):
            return  # Skip header
        
        parts = line.split(',')
        if len(parts) >= 7:
            try:
                point_data = {
                    'id': int(parts[0]),
                    'x': float(parts[1]),
                    'y': float(parts[2]),
                    'z': float(parts[3]),
                    'height': float(parts[4]),
                    'valid': parts[5].strip() == 'Yes',
                    'timestamp': float(parts[6])
                }
                self.scan_points.append(point_data)
            except (ValueError, IndexError):
                pass
    
    def _parse_edge_point_line(self, line: str):
        """Parse edge points data."""
        if line.startswith("Edge_ID,"):
            return  # Skip header
        
        parts = line.split(',')
        if len(parts) >= 6:
            try:
                edge_data = {
                    'id': parts[0],
                    'x': float(parts[1]),
                    'y': float(parts[2]),
                    'type': parts[3],
                    'confidence': float(parts[4]),
                    'classification': parts[5]
                }
                self.edge_points.append(edge_data)
            except (ValueError, IndexError):
                pass
    
    def get_die_center(self) -> Tuple[float, float]:
        """Get the calculated die center coordinates."""
        return (self.summary_data.get('Die Center X (mm)', 0),
                self.summary_data.get('Die Center Y (mm)', 0))
    
    def get_die_diameter(self) -> float:
        """Get the calculated die diameter."""
        return self.summary_data.get('Diameter (mm)', 0)
    
    def get_scan_points_array(self) -> np.ndarray:
        """Get scan points as numpy array [x, y, height]."""
        points = []
        for point in self.scan_points:
            if point['valid']:
                points.append([point['x'], point['y'], point['height']])
        return np.array(points)
    
    def get_edge_points_by_type(self) -> Dict[str, List[Dict]]:
        """Get edge points grouped by classification."""
        grouped = {'Outer': [], 'Interior': []}
        for edge in self.edge_points:
            classification = edge['classification']
            if classification in grouped:
                grouped[classification].append(edge)
        return grouped

class DiescanVisualizer:
    """Visualizer for diescan data with heat maps and overlays."""
    
    def __init__(self, parser: DiescanParser):
        self.parser = parser
        self.fig = None
        self.ax = None
        
        # Color maps for different visualizations
        self.height_cmap = LinearSegmentedColormap.from_list(
            'die_height', ['blue', 'cyan', 'yellow', 'orange', 'red']
        )
    
    def create_heat_map(self, interpolation_resolution: int = 100, 
                       figsize: Tuple[int, int] = (12, 8)):
        """Create the main heat map visualization."""
        self.fig, self.ax = plt.subplots(figsize=figsize)
        
        # Get scan points
        points = self.parser.get_scan_points_array()
        if len(points) == 0:
            print("No valid scan points found")
            return
        
        x, y, heights = points[:, 0], points[:, 1], points[:, 2]
        
        # Create interpolation grid
        xi = np.linspace(x.min(), x.max(), interpolation_resolution)
        yi = np.linspace(y.min(), y.max(), interpolation_resolution)
        xi_grid, yi_grid = np.meshgrid(xi, yi)
        
        # Interpolate height data
        zi = griddata((x, y), heights, (xi_grid, yi_grid), method='cubic')
        
        # Create heat map
        im = self.ax.imshow(zi, extent=[x.min(), x.max(), y.min(), y.max()],
                           origin='lower', cmap=self.height_cmap, aspect='equal')
        
        # Add colorbar
        cbar = plt.colorbar(im, ax=self.ax, shrink=0.8)
        cbar.set_label('Height (mm)', rotation=270, labelpad=20)
        
        # Add contour lines
        contour = self.ax.contour(xi_grid, yi_grid, zi, levels=10, colors='white', alpha=0.6)
        self.ax.clabel(contour, inline=True, fontsize=8)
        
        # Set labels and title
        self.ax.set_xlabel('X Position (mm)')
        self.ax.set_ylabel('Y Position (mm)')
        
        # Extract timestamp from filename for title
        filename = Path(self.parser.csv_file).stem
        timestamp_match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', filename)
        timestamp = timestamp_match.group(1) if timestamp_match else 'Unknown'
        
        self.ax.set_title(f'Die Scanner Heat Map - {timestamp}')
        
        return self.fig, self.ax
    
    def add_circle_overlay(self):
        """Add die center and fitted circle overlay."""
        if self.ax is None:
            return
        
        # Get die center and diameter
        center_x, center_y = self.parser.get_die_center()
        diameter = self.parser.get_die_diameter()
        radius = diameter / 2
        
        # Add crosshair for center
        self.ax.plot(center_x, center_y, 'r+', markersize=15, markeredgewidth=3,
                    label=f'Calculated Center ({center_x:.1f}, {center_y:.1f})')
        
        # Add fitted circle
        if diameter > 0:
            circle = patches.Circle((center_x, center_y), radius, 
                                  fill=False, color='red', linewidth=2,
                                  label=f'Fitted Circle (Ø{diameter:.1f}mm)')
            self.ax.add_patch(circle)
        
        # Add true center if available
        true_center_x = self.parser.accuracy_data.get('True Center X (mm)')
        true_center_y = self.parser.accuracy_data.get('True Center Y (mm)')
        
        if true_center_x is not None and true_center_y is not None:
            self.ax.plot(true_center_x, true_center_y, 'g+', markersize=15, 
                        markeredgewidth=3, label=f'True Center ({true_center_x:.1f}, {true_center_y:.1f})')
        
        # Add edge points
        edge_groups = self.parser.get_edge_points_by_type()
        
        # Plot outer edges
        outer_edges = edge_groups['Outer']
        if outer_edges:
            outer_x = [edge['x'] for edge in outer_edges]
            outer_y = [edge['y'] for edge in outer_edges]
            self.ax.scatter(outer_x, outer_y, c='yellow', s=100, marker='o',
                          edgecolors='black', linewidth=2, label='Outer Edges')
        
        # Plot interior edges
        interior_edges = edge_groups['Interior']
        if interior_edges:
            interior_x = [edge['x'] for edge in interior_edges]
            interior_y = [edge['y'] for edge in interior_edges]
            self.ax.scatter(interior_x, interior_y, c='orange', s=60, marker='s',
                          edgecolors='black', linewidth=1, label='Interior Edges')
        
        # Add legend
        self.ax.legend(loc='upper right', bbox_to_anchor=(1.0, 1.0))
    
    def add_statistics_text(self):
        """Add statistics text box to the plot."""
        if self.ax is None:
            return
        
        # Prepare statistics text
        stats_text = []
        
        # Add accuracy metrics
        total_error = self.parser.accuracy_data.get('Total Error (mm)', 0)
        target_achieved = self.parser.accuracy_data.get('Target Achieved', 'Unknown')
        stats_text.append(f'Total Error: {total_error:.1f} mm')
        stats_text.append(f'Target Achieved: {target_achieved}')
        
        # Add data quality
        valid_points = self.parser.data_quality.get('Valid Scan Points', 0)
        total_points = self.parser.data_quality.get('Total Scan Points', 0)
        data_quality = self.parser.data_quality.get('Data Quality (%)', 0)
        stats_text.append(f'Data Quality: {data_quality:.1f}%')
        stats_text.append(f'Valid Points: {valid_points}/{total_points}')
        
        # Add edge detection
        edges_found = self.parser.edge_detection.get('Total Edges Found', 0)
        outer_edges = self.parser.edge_detection.get('Outer Edges (Used)', 0)
        stats_text.append(f'Edges Found: {edges_found} ({outer_edges} outer)')
        
        # Add coverage
        edge_coverage = self.parser.coverage_analysis.get('Edge Coverage (% perimeter)', 0)
        stats_text.append(f'Edge Coverage: {edge_coverage:.1f}%')
        
        # Create text box
        text_box = '\n'.join(stats_text)
        self.ax.text(0.02, 0.98, text_box, transform=self.ax.transAxes,
                    verticalalignment='top', bbox=dict(boxstyle='round', 
                    facecolor='white', alpha=0.8), fontsize=9)
    
    def save_plot(self, output_path: str, dpi: int = 300):
        """Save the current plot to file."""
        if self.fig is None:
            return
        
        self.fig.savefig(output_path, dpi=dpi, bbox_inches='tight')
        print(f"Plot saved to: {output_path}")
    
    def show_plot(self):
        """Display the plot."""
        if self.fig is None:
            return
        plt.show()

def create_comparison_plot(csv_files: List[str], output_dir: str = None):
    """Create a comparison plot of multiple diescan results."""
    n_files = len(csv_files)
    if n_files == 0:
        print("No CSV files provided")
        return
    
    # Calculate subplot layout
    cols = min(3, n_files)
    rows = (n_files + cols - 1) // cols
    
    fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 5*rows))
    if n_files == 1:
        axes = [axes]
    elif rows == 1:
        axes = axes.reshape(1, -1)
    
    # Flatten axes for easier iteration
    axes_flat = axes.flatten() if n_files > 1 else axes
    
    for i, csv_file in enumerate(csv_files):
        try:
            parser = DiescanParser(csv_file)
            
            # Get scan points
            points = parser.get_scan_points_array()
            if len(points) == 0:
                continue
            
            x, y, heights = points[:, 0], points[:, 1], points[:, 2]
            
            # Create heat map on subplot
            ax = axes_flat[i]
            
            # Simple scatter plot for comparison
            scatter = ax.scatter(x, y, c=heights, cmap='viridis', s=1, alpha=0.7)
            
            # Add die center and circle
            center_x, center_y = parser.get_die_center()
            diameter = parser.get_die_diameter()
            
            ax.plot(center_x, center_y, 'r+', markersize=10, markeredgewidth=2)
            
            if diameter > 0:
                circle = patches.Circle((center_x, center_y), diameter/2, 
                                      fill=False, color='red', linewidth=1)
                ax.add_patch(circle)
            
            # Set title and labels
            filename = Path(csv_file).stem
            timestamp = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', filename)
            title = timestamp.group(1) if timestamp else filename
            ax.set_title(title, fontsize=10)
            ax.set_xlabel('X (mm)')
            ax.set_ylabel('Y (mm)')
            ax.set_aspect('equal')
            
            # Add colorbar
            plt.colorbar(scatter, ax=ax, shrink=0.8)
            
        except Exception as e:
            print(f"Error processing {csv_file}: {e}")
    
    # Hide unused subplots
    for i in range(n_files, len(axes_flat)):
        axes_flat[i].set_visible(False)
    
    plt.tight_layout()
    
    if output_dir:
        output_path = os.path.join(output_dir, 'diescan_comparison.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Comparison plot saved to: {output_path}")
    
    plt.show()

def main():
    """Main function for command-line interface."""
    parser = argparse.ArgumentParser(description='FANUC Die Scanner Heat Map Visualizer')
    parser.add_argument('csv_files', nargs='+', help='CSV files to process')
    parser.add_argument('--output-dir', '-o', help='Output directory for saved images')
    parser.add_argument('--comparison', '-c', action='store_true', 
                       help='Create comparison plot of multiple files')
    parser.add_argument('--resolution', '-r', type=int, default=100,
                       help='Interpolation resolution (default: 100)')
    parser.add_argument('--dpi', type=int, default=300, help='Output DPI (default: 300)')
    
    args = parser.parse_args()
    
    # Create output directory if specified
    if args.output_dir:
        os.makedirs(args.output_dir, exist_ok=True)
    
    if args.comparison:
        create_comparison_plot(args.csv_files, args.output_dir)
    else:
        # Process each file individually
        for csv_file in args.csv_files:
            try:
                print(f"Processing {csv_file}...")
                
                # Parse CSV file
                csv_parser = DiescanParser(csv_file)
                
                # Create visualizer
                visualizer = DiescanVisualizer(csv_parser)
                
                # Create heat map
                visualizer.create_heat_map(interpolation_resolution=args.resolution)
                
                # Add overlays
                visualizer.add_circle_overlay()
                visualizer.add_statistics_text()
                
                # Save or show plot
                if args.output_dir:
                    filename = Path(csv_file).stem
                    output_path = os.path.join(args.output_dir, f'{filename}_heatmap.png')
                    visualizer.save_plot(output_path, dpi=args.dpi)
                else:
                    visualizer.show_plot()
                
            except Exception as e:
                print(f"Error processing {csv_file}: {e}")

if __name__ == "__main__":
    main()