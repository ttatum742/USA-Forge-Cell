"""
Die Scanner Debug Visualization Tool

This program provides real-time visualization and debugging capabilities 
for the Keyence die scanner system. It displays scan results, post-processing
results, and allows for interactive analysis of measurement data.
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, Slider, CheckButtons
import numpy as np
import json
import time
import logging
from typing import List, Dict, Optional, Tuple
from dataclasses import asdict
import serial
import threading
from queue import Queue, Empty

# Import the main scanner classes
from dieScanner_keyence import KeyenceDieMeasurement, MeasurementPoint, DieResults, WellCenterResult, ZoneClassification

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DieScannerDebugger:
    """Debug visualization tool for die scanner system"""
    
    def __init__(self, robot_ip: str = "192.168.0.1", arduino_port: str = "COM3"):
        self.robot_ip = robot_ip
        self.arduino_port = arduino_port
        
        # Data storage
        self.measurement_history: List[List[MeasurementPoint]] = []
        self.results_history: List[DieResults] = []
        self.well_results_history: List[WellCenterResult] = []
        self.current_measurements: List[MeasurementPoint] = []
        self.current_results: Optional[DieResults] = None
        self.current_well_result: Optional[WellCenterResult] = None
        self.current_zones: Optional[ZoneClassification] = None
        
        # Live data monitoring
        self.live_monitoring = False
        self.arduino_connection = None
        self.data_queue = Queue()
        self.monitoring_thread = None
        
        # Visualization state
        self.show_grid = True
        self.show_valid_points = True
        self.show_invalid_points = True
        self.show_center = True
        self.show_confidence_regions = True
        
        # Initialize GUI
        self.setup_gui()
        
    def setup_gui(self):
        """Setup the debugging GUI"""
        # Create main figure with subplots
        self.fig = plt.figure(figsize=(16, 12))
        self.fig.suptitle('Die Scanner Debug Visualization', fontsize=16)
        
        # Create subplot grid
        gs = self.fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        
        # Main 3D visualization
        self.ax_3d = self.fig.add_subplot(gs[0:2, 0:2], projection='3d')
        self.ax_3d.set_title('3D Measurement Visualization')
        
        # 2D height map
        self.ax_2d = self.fig.add_subplot(gs[0, 2])
        self.ax_2d.set_title('Height Map')
        
        # Statistics plot
        self.ax_stats = self.fig.add_subplot(gs[1, 2])
        self.ax_stats.set_title('Measurement Statistics')
        
        # Live sensor reading
        self.ax_live = self.fig.add_subplot(gs[2, 0])
        self.ax_live.set_title('Live Sensor Reading')
        
        # Analysis results
        self.ax_results = self.fig.add_subplot(gs[2, 1])
        self.ax_results.set_title('Analysis Results')
        self.ax_results.axis('off')
        
        # Control panel
        self.ax_controls = self.fig.add_subplot(gs[2, 2])
        self.ax_controls.set_title('Controls')
        self.ax_controls.axis('off')
        
        # Setup controls
        self.setup_controls()
        
        # Initialize plots
        self.initialize_plots()
        
    def setup_controls(self):
        """Setup control buttons and checkboxes"""
        # Control buttons
        ax_load = plt.axes([0.7, 0.15, 0.08, 0.04])
        ax_save = plt.axes([0.79, 0.15, 0.08, 0.04])
        ax_clear = plt.axes([0.88, 0.15, 0.08, 0.04])
        ax_live = plt.axes([0.7, 0.10, 0.08, 0.04])
        ax_arduino = plt.axes([0.79, 0.10, 0.08, 0.04])
        
        self.btn_load = Button(ax_load, 'Load')
        self.btn_save = Button(ax_save, 'Save')
        self.btn_clear = Button(ax_clear, 'Clear')
        self.btn_live = Button(ax_live, 'Live Mon')
        self.btn_arduino = Button(ax_arduino, 'Arduino')
        
        # Connect button callbacks
        self.btn_load.on_clicked(self.load_data)
        self.btn_save.on_clicked(self.save_data)
        self.btn_clear.on_clicked(self.clear_data)
        self.btn_live.on_clicked(self.toggle_live_monitoring)
        self.btn_arduino.on_clicked(self.test_arduino)
        
        # Visualization checkboxes
        ax_checks = plt.axes([0.7, 0.02, 0.25, 0.06])
        self.checkboxes = CheckButtons(ax_checks, 
                                     ['Grid', 'Valid', 'Invalid', 'Center', 'Confidence'],
                                     [True, True, True, True, True])
        self.checkboxes.on_clicked(self.update_visualization_options)
        
    def initialize_plots(self):
        """Initialize empty plots"""
        # Initialize 3D plot
        self.ax_3d.set_xlabel('X (mm)')
        self.ax_3d.set_ylabel('Y (mm)')
        self.ax_3d.set_zlabel('Height (mm)')
        
        # Initialize 2D plot
        self.ax_2d.set_xlabel('X Index')
        self.ax_2d.set_ylabel('Y Index')
        
        # Initialize live plot
        self.live_data_x = []
        self.live_data_y = []
        self.ax_live.set_xlabel('Time (s)')
        self.ax_live.set_ylabel('Height (mm)')
        
        # Initialize statistics plot
        self.ax_stats.set_xlabel('Measurement')
        self.ax_stats.set_ylabel('Count')
        
    def load_scanner_data(self, scanner: KeyenceDieMeasurement):
        """Load data from a scanner instance"""
        if scanner.measurement_points:
            self.current_measurements = scanner.measurement_points.copy()
            self.measurement_history.append(self.current_measurements)
            
            # Try to get enhanced results if available
            if hasattr(scanner, 'well_result') and scanner.well_result:
                self.current_well_result = scanner.well_result
                self.well_results_history.append(self.current_well_result)
                
            if hasattr(scanner, 'zone_classification') and scanner.zone_classification:
                self.current_zones = scanner.zone_classification
            
            self.update_visualization()
            logger.info(f"Loaded {len(self.current_measurements)} measurement points with enhanced analysis")
            
    def update_visualization(self):
        """Update all visualization plots"""
        if not self.current_measurements:
            return
            
        # Clear previous plots
        self.ax_3d.clear()
        self.ax_2d.clear()
        self.ax_stats.clear()
        
        # Separate valid and invalid points
        valid_points = [p for p in self.current_measurements if p.is_valid]
        invalid_points = [p for p in self.current_measurements if not p.is_valid]
        
        # 3D Visualization
        self.plot_3d_measurements(valid_points, invalid_points)
        
        # 2D Height map
        self.plot_2d_heatmap(valid_points, invalid_points)
        
        # Statistics
        self.plot_statistics(valid_points, invalid_points)
        
        # Results text
        self.display_results()
        
        # Refresh the plot
        plt.draw()
        
    def plot_3d_measurements(self, valid_points: List[MeasurementPoint], 
                           invalid_points: List[MeasurementPoint]):
        """Plot 3D measurement visualization"""
        
        # Plot valid points
        if valid_points and self.show_valid_points:
            x_valid = [p.x for p in valid_points]
            y_valid = [p.y for p in valid_points]
            z_valid = [p.height for p in valid_points]
            
            scatter = self.ax_3d.scatter(x_valid, y_valid, z_valid, 
                                       c=z_valid, cmap='viridis', 
                                       s=50, alpha=0.7, label='Valid')
            
        # Plot invalid points (out of range)
        if invalid_points and self.show_invalid_points:
            x_invalid = [p.x for p in invalid_points]
            y_invalid = [p.y for p in invalid_points]
            z_invalid = [p.z for p in invalid_points]  # Use robot Z position
            
            self.ax_3d.scatter(x_invalid, y_invalid, z_invalid, 
                             c='red', s=100, alpha=0.8, 
                             marker='x', label='Out of Range')
        
        # Plot estimated center
        if self.current_results and self.show_center:
            center_x = self.current_results.center_x
            center_y = self.current_results.center_y
            # Use average robot Z position for center
            avg_z = np.mean([p.z for p in self.current_measurements])
            
            self.ax_3d.scatter([center_x], [center_y], [avg_z], 
                             c='orange', s=200, marker='*', 
                             label=f'Center ({center_x:.1f}, {center_y:.1f})')
        
        self.ax_3d.set_xlabel('X (mm)')
        self.ax_3d.set_ylabel('Y (mm)')
        self.ax_3d.set_zlabel('Height (mm)')
        self.ax_3d.legend()
        self.ax_3d.set_title('3D Measurement Data')
        
    def plot_2d_heatmap(self, valid_points: List[MeasurementPoint], 
                       invalid_points: List[MeasurementPoint]):
        """Plot 2D height map"""
        
        if not valid_points:
            self.ax_2d.text(0.5, 0.5, 'No valid measurements', 
                          ha='center', va='center', transform=self.ax_2d.transAxes)
            return
            
        # Create grid for interpolation
        all_points = valid_points + invalid_points
        x_coords = [p.x for p in all_points]
        y_coords = [p.y for p in all_points]
        
        if len(set(x_coords)) < 2 or len(set(y_coords)) < 2:
            self.ax_2d.text(0.5, 0.5, 'Insufficient data for grid', 
                          ha='center', va='center', transform=self.ax_2d.transAxes)
            return
        
        # Create regular grid
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        
        grid_size = int(np.sqrt(len(all_points))) or 5
        x_grid = np.linspace(x_min, x_max, grid_size)
        y_grid = np.linspace(y_min, y_max, grid_size)
        X, Y = np.meshgrid(x_grid, y_grid)
        
        # Create height map (use -999 for invalid points)
        Z = np.full_like(X, np.nan)
        
        for i, x_val in enumerate(x_grid):
            for j, y_val in enumerate(y_grid):
                # Find closest measurement point
                distances = [np.sqrt((p.x - x_val)**2 + (p.y - y_val)**2) 
                           for p in all_points]
                closest_idx = np.argmin(distances)
                closest_point = all_points[closest_idx]
                
                if distances[closest_idx] < 10:  # Within 10mm
                    if closest_point.is_valid:
                        Z[j, i] = closest_point.height
                    else:
                        Z[j, i] = -999  # Mark as out of range
        
        # Plot heatmap
        im = self.ax_2d.imshow(Z, extent=[x_min, x_max, y_min, y_max], 
                              origin='lower', cmap='viridis', alpha=0.8)
        
        # Add colorbar
        plt.colorbar(im, ax=self.ax_2d, shrink=0.8)
        
        # Overlay measurement points
        if self.show_valid_points:
            x_valid = [p.x for p in valid_points]
            y_valid = [p.y for p in valid_points]
            self.ax_2d.scatter(x_valid, y_valid, c='white', s=20, alpha=0.8)
            
        if self.show_invalid_points:
            x_invalid = [p.x for p in invalid_points]
            y_invalid = [p.y for p in invalid_points]
            self.ax_2d.scatter(x_invalid, y_invalid, c='red', s=30, marker='x')
        
        # Mark center
        if self.current_results and self.show_center:
            self.ax_2d.scatter([self.current_results.center_x], 
                             [self.current_results.center_y], 
                             c='orange', s=100, marker='*')
        
        self.ax_2d.set_xlabel('X (mm)')
        self.ax_2d.set_ylabel('Y (mm)')
        self.ax_2d.set_title('2D Height Map')
        
    def plot_statistics(self, valid_points: List[MeasurementPoint], 
                       invalid_points: List[MeasurementPoint]):
        """Plot measurement statistics"""
        
        if valid_points:
            heights = [p.height for p in valid_points]
            self.ax_stats.hist(heights, bins=min(20, len(heights)//3), 
                             alpha=0.7, label=f'Valid ({len(valid_points)})')
        
        # Add statistics text
        stats_text = f"Total Points: {len(self.current_measurements)}\\n"
        stats_text += f"Valid: {len(valid_points)}\\n"
        stats_text += f"Invalid: {len(invalid_points)}\\n"
        
        if valid_points:
            heights = [p.height for p in valid_points]
            stats_text += f"Height Range: {min(heights):.1f} - {max(heights):.1f} mm\\n"
            stats_text += f"Mean Height: {np.mean(heights):.1f} mm\\n"
            stats_text += f"Std Dev: {np.std(heights):.1f} mm"
        
        self.ax_stats.text(0.05, 0.95, stats_text, transform=self.ax_stats.transAxes,
                         verticalalignment='top', fontsize=10,
                         bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
        
        self.ax_stats.set_xlabel('Height (mm)')
        self.ax_stats.set_ylabel('Count')
        self.ax_stats.legend()
        
    def display_results(self):
        """Display analysis results"""
        self.ax_results.clear()
        self.ax_results.axis('off')
        
        if self.current_well_result:
            # Enhanced well center results
            results_text = "ENHANCED WELL ANALYSIS\\n"
            results_text += "=" * 20 + "\\n"
            results_text += f"Well Center X: {self.current_well_result.center_x:.1f} mm\\n"
            results_text += f"Well Center Y: {self.current_well_result.center_y:.1f} mm\\n"
            results_text += f"Well Diameter: {self.current_well_result.well_diameter:.1f} mm\\n"
            results_text += f"Confidence: {self.current_well_result.confidence:.1f}%\\n"
            
            if self.current_zones:
                results_text += f"\\nZONE CLASSIFICATION:\\n"
                results_text += f"Die Face: {len(self.current_zones.die_face)} pts\\n"
                results_text += f"Central Well: {len(self.current_zones.central_well)} pts\\n"
                results_text += f"Outer Edges: {len(self.current_zones.outer_edges)} pts\\n"
                results_text += f"Face Height: {self.current_zones.die_face_height:.1f} mm"
            
            confidence_color = ('green' if self.current_well_result.confidence >= 70 else 
                              'orange' if self.current_well_result.confidence >= 50 else 'red')
            
        elif self.current_results:
            # Legacy results
            results_text = "LEGACY ANALYSIS RESULTS\\n"
            results_text += "=" * 20 + "\\n"
            results_text += f"Center X: {self.current_results.center_x:.1f} mm\\n"
            results_text += f"Center Y: {self.current_results.center_y:.1f} mm\\n"
            results_text += f"Avg Height: {self.current_results.average_height:.1f} mm\\n"
            results_text += f"Confidence: {self.current_results.confidence:.1f}%\\n"
            results_text += f"Face Points: {len(self.current_results.die_face_points)}\\n"
            results_text += f"Center Points: {len(self.current_results.center_area_points)}"
            
            confidence_color = ('green' if self.current_results.confidence >= 70 else 
                              'orange' if self.current_results.confidence >= 50 else 'red')
            
        else:
            results_text = "NO ANALYSIS RESULTS\\n"
            results_text += "Run enhanced scanner to generate results"
            confidence_color = 'gray'
        
        self.ax_results.text(0.05, 0.95, results_text, transform=self.ax_results.transAxes,
                           verticalalignment='top', fontsize=9,
                           bbox=dict(boxstyle='round', facecolor=confidence_color, alpha=0.3))
        
    def toggle_live_monitoring(self, event):
        """Toggle live sensor monitoring"""
        self.live_monitoring = not self.live_monitoring
        
        if self.live_monitoring:
            self.start_live_monitoring()
            self.btn_live.label.set_text('Stop')
        else:
            self.stop_live_monitoring()
            self.btn_live.label.set_text('Live Mon')
            
    def start_live_monitoring(self):
        """Start live monitoring thread"""
        try:
            self.arduino_connection = serial.Serial(self.arduino_port, 115200, timeout=1)
            time.sleep(2)
            
            # Turn on sensor
            self.arduino_connection.write(b'POWER_ON\\n')
            time.sleep(0.5)
            
            self.monitoring_thread = threading.Thread(target=self.live_monitoring_worker)
            self.monitoring_thread.daemon = True
            self.monitoring_thread.start()
            
            # Start animation for live plot
            self.live_animation = animation.FuncAnimation(self.fig, self.update_live_plot, 
                                                        interval=100, blit=False)
            
            logger.info("Live monitoring started")
            
        except Exception as e:
            logger.error(f"Failed to start live monitoring: {e}")
            self.live_monitoring = False
            
    def live_monitoring_worker(self):
        """Worker thread for live sensor monitoring"""
        start_time = time.time()
        
        while self.live_monitoring and self.arduino_connection:
            try:
                self.arduino_connection.write(b'READ\\n')
                response = self.arduino_connection.readline().decode().strip()
                
                if response.startswith('HEIGHT:'):
                    height = float(response.replace('HEIGHT:', ''))
                    current_time = time.time() - start_time
                    
                    self.data_queue.put((current_time, height))
                    
                time.sleep(0.1)  # 10Hz sampling
                
            except Exception as e:
                logger.error(f"Live monitoring error: {e}")
                break
                
    def update_live_plot(self, frame):
        """Update live sensor plot"""
        # Get new data from queue
        new_data = []
        try:
            while True:
                new_data.append(self.data_queue.get_nowait())
        except Empty:
            pass
        
        # Add new data to live data
        for timestamp, height in new_data:
            self.live_data_x.append(timestamp)
            self.live_data_y.append(height)
            
        # Keep only last 100 points
        if len(self.live_data_x) > 100:
            self.live_data_x = self.live_data_x[-100:]
            self.live_data_y = self.live_data_y[-100:]
        
        # Update plot
        self.ax_live.clear()
        if self.live_data_x:
            self.ax_live.plot(self.live_data_x, self.live_data_y, 'b-', alpha=0.7)
            self.ax_live.scatter(self.live_data_x[-1:], self.live_data_y[-1:], 
                               c='red', s=50)
            
        self.ax_live.set_xlabel('Time (s)')
        self.ax_live.set_ylabel('Height (mm)')
        self.ax_live.set_title(f'Live Sensor Reading ({len(self.live_data_x)} points)')
        self.ax_live.grid(True, alpha=0.3)
        
    def stop_live_monitoring(self):
        """Stop live monitoring"""
        self.live_monitoring = False
        
        if hasattr(self, 'live_animation'):
            self.live_animation.event_source.stop()
            
        if self.arduino_connection:
            try:
                self.arduino_connection.write(b'POWER_OFF\\n')
                time.sleep(0.5)
                self.arduino_connection.close()
            except:
                pass
            self.arduino_connection = None
            
        logger.info("Live monitoring stopped")
        
    def test_arduino(self, event):
        """Test Arduino connection"""
        try:
            test_conn = serial.Serial(self.arduino_port, 115200, timeout=2)
            time.sleep(2)
            
            # Test power on
            test_conn.write(b'POWER_ON\\n')
            response = test_conn.readline().decode().strip()
            print(f"Power On Response: {response}")
            
            # Test status
            test_conn.write(b'STATUS\\n')
            response = test_conn.readline().decode().strip()
            print(f"Status Response: {response}")
            
            # Test reading
            test_conn.write(b'READ\\n')
            response = test_conn.readline().decode().strip()
            print(f"Read Response: {response}")
            
            # Power off
            test_conn.write(b'POWER_OFF\\n')
            response = test_conn.readline().decode().strip()
            print(f"Power Off Response: {response}")
            
            test_conn.close()
            logger.info("Arduino test completed successfully")
            
        except Exception as e:
            logger.error(f"Arduino test failed: {e}")
            
    def update_visualization_options(self, label):
        """Update visualization options based on checkbox selection"""
        options = {
            'Grid': 'show_grid',
            'Valid': 'show_valid_points', 
            'Invalid': 'show_invalid_points',
            'Center': 'show_center',
            'Confidence': 'show_confidence_regions'
        }
        
        if label in options:
            current_value = getattr(self, options[label])
            setattr(self, options[label], not current_value)
            self.update_visualization()
            
    def save_data(self, event):
        """Save current measurement data"""
        if not self.current_measurements:
            logger.warning("No data to save")
            return
            
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"die_scan_debug_{timestamp}.json"
        
        save_data = {
            'timestamp': timestamp,
            'measurements': [asdict(p) for p in self.current_measurements],
            'results': asdict(self.current_results) if self.current_results else None
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(save_data, f, indent=2)
            logger.info(f"Data saved to {filename}")
        except Exception as e:
            logger.error(f"Failed to save data: {e}")
            
    def load_data(self, event):
        """Load measurement data from file"""
        # This would typically open a file dialog
        # For now, just log the capability
        logger.info("Load data functionality - implement file dialog")
        
    def clear_data(self, event):
        """Clear all current data"""
        self.current_measurements.clear()
        self.current_results = None
        self.live_data_x.clear()
        self.live_data_y.clear()
        self.update_visualization()
        logger.info("Data cleared")
        
    def show(self):
        """Show the debug interface"""
        plt.show()

def run_debug_interface():
    """Run the debug interface standalone"""
    debugger = DieScannerDebugger()
    
    # Example: Load some test data for demonstration
    test_points = []
    for i in range(81):  # 9x9 grid
        x = (i % 9 - 4) * 10  # -40 to +40 mm
        y = (i // 9 - 4) * 10
        
        # Simulate die measurement (center area out of range)
        if abs(x) < 15 and abs(y) < 15:
            height = -999  # Out of range (die center)
            is_valid = False
        else:
            height = 50 + np.random.normal(0, 2)  # Face measurement
            is_valid = True
            
        test_points.append(MeasurementPoint(x=x, y=y, z=100, height=height, is_valid=is_valid))
    
    debugger.current_measurements = test_points
    
    # Create enhanced test results
    debugger.current_well_result = WellCenterResult(
        valid=True,
        center_x=2.5, 
        center_y=-1.2, 
        well_diameter=24.5,
        confidence=87.5,
        edge_points=[p for p in test_points if p.is_valid and abs(np.sqrt(p.x**2 + p.y**2) - 12) < 3]
    )
    
    # Create zone classification
    debugger.current_zones = ZoneClassification(
        die_face=[p for p in test_points if p.is_valid],
        central_well=[p for p in test_points if not p.is_valid],
        outer_edges=[],
        other_valid=[],
        debris=[],
        die_face_height=49.8
    )
    
    debugger.update_visualization()
    debugger.show()

if __name__ == "__main__":
    run_debug_interface()