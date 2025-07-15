"""
Sensor Test Program - Real-time Analog Sensor Data Visualization
Author: For Jeff's sensor comparison testing

This standalone program interfaces with Arduino to test and compare various sensor types.
Features:
- Real-time data plotting (sensor reading vs time)
- Save data and plots on keypress or button click
- Support for multiple sensor configurations
- Statistical analysis and comparison tools
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, Slider, CheckButtons, TextBox
import numpy as np
import serial
import time
import json
import csv
import logging
from datetime import datetime
from typing import List, Dict, Optional, Tuple
import threading
from queue import Queue, Empty
import tkinter as tk
from tkinter import messagebox, simpledialog
import os

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SensorTester:
    """Real-time sensor testing and comparison tool"""
    
    def __init__(self, arduino_port: str = "COM3", baud_rate: int = 115200):
        self.arduino_port = arduino_port
        self.baud_rate = baud_rate
        self.arduino = None
        
        # Data storage
        self.time_data: List[float] = []
        self.sensor_data: List[float] = []
        self.voltage_data: List[float] = []
        self.raw_adc_data: List[int] = []
        
        # Test session management
        self.test_sessions: Dict[str, Dict] = {}
        self.current_session_name = "Test_1"
        self.session_counter = 1
        
        # Real-time monitoring
        self.monitoring = False
        self.data_queue = Queue()
        self.monitoring_thread = None
        self.start_time = None
        
        # Sensor configuration
        self.sensor_configs = {
            "Keyence_LK-G5000": {"range_mm": (10, 100), "voltage_range": (0.5, 4.5), "color": "blue"},
            "Ultrasonic_Generic": {"range_mm": (20, 400), "voltage_range": (0.2, 4.8), "color": "green"},
            "Laser_SICK": {"range_mm": (5, 60), "voltage_range": (1.0, 4.0), "color": "red"},
            "Custom_Sensor": {"range_mm": (0, 200), "voltage_range": (0, 5.0), "color": "purple"}
        }
        self.current_sensor_type = "Keyence_LK-G5000"
        
        # Display settings
        self.max_display_points = 1000  # Maximum points to display
        self.update_interval = 50       # Animation update interval (ms)
        self.sampling_rate = 20         # Hz
        
        # Statistics
        self.stats_window = 100  # Points for rolling statistics
        
        # Setup GUI
        self.setup_gui()
        self.connect_arduino()
        
    def setup_gui(self):
        """Setup the main GUI interface"""
        # Create main figure
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('Sensor Test Program - Real-time Data Visualization', fontsize=16)
        
        # Create subplot layout
        gs = self.fig.add_gridspec(3, 4, hspace=0.4, wspace=0.3)
        
        # Main real-time plot
        self.ax_main = self.fig.add_subplot(gs[0:2, 0:3])
        self.ax_main.set_title('Real-time Sensor Data')
        self.ax_main.set_xlabel('Time (seconds)')
        self.ax_main.set_ylabel('Distance (mm)')
        self.ax_main.grid(True, alpha=0.3)
        
        # Secondary voltage plot
        self.ax_voltage = self.fig.add_subplot(gs[2, 0:2])
        self.ax_voltage.set_title('Voltage Output')
        self.ax_voltage.set_xlabel('Time (seconds)')
        self.ax_voltage.set_ylabel('Voltage (V)')
        self.ax_voltage.grid(True, alpha=0.3)
        
        # Statistics display
        self.ax_stats = self.fig.add_subplot(gs[0, 3])
        self.ax_stats.set_title('Live Statistics')
        self.ax_stats.axis('off')
        
        # Sensor configuration panel
        self.ax_config = self.fig.add_subplot(gs[1, 3])
        self.ax_config.set_title('Sensor Config')
        self.ax_config.axis('off')
        
        # Control panel
        self.ax_controls = self.fig.add_subplot(gs[2, 2:4])
        self.ax_controls.set_title('Controls')
        self.ax_controls.axis('off')
        
        # Setup control widgets
        self.setup_controls()
        
        # Initialize plot lines
        self.line_main, = self.ax_main.plot([], [], 'b-', linewidth=1.5, label='Distance')
        self.line_voltage, = self.ax_voltage.plot([], [], 'r-', linewidth=1.5, label='Voltage')
        
        # Setup key bindings
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
    def setup_controls(self):
        """Setup control buttons and widgets"""
        # Control buttons
        button_width = 0.08
        button_height = 0.04
        button_spacing = 0.01
        
        # Row 1: Main controls
        y_pos = 0.8
        self.btn_start = Button(plt.axes([0.7, y_pos, button_width, button_height]), 'Start')
        self.btn_stop = Button(plt.axes([0.7 + button_width + button_spacing, y_pos, button_width, button_height]), 'Stop')
        self.btn_save = Button(plt.axes([0.7 + 2*(button_width + button_spacing), y_pos, button_width, button_height]), 'Save')
        
        # Row 2: Session controls
        y_pos = 0.7
        self.btn_new_session = Button(plt.axes([0.7, y_pos, button_width, button_height]), 'New Test')
        self.btn_compare = Button(plt.axes([0.7 + button_width + button_spacing, y_pos, button_width, button_height]), 'Compare')
        self.btn_clear = Button(plt.axes([0.7 + 2*(button_width + button_spacing), y_pos, button_width, button_height]), 'Clear')
        
        # Row 3: Arduino controls
        y_pos = 0.6
        self.btn_connect = Button(plt.axes([0.7, y_pos, button_width, button_height]), 'Connect')
        self.btn_power_on = Button(plt.axes([0.7 + button_width + button_spacing, y_pos, button_width, button_height]), 'Power On')
        self.btn_power_off = Button(plt.axes([0.7 + 2*(button_width + button_spacing), y_pos, button_width, button_height]), 'Power Off')
        
        # Sensor type selector
        y_pos = 0.45
        sensor_types = list(self.sensor_configs.keys())
        self.sensor_selector = CheckButtons(plt.axes([0.7, y_pos, 0.25, 0.1]), sensor_types, [True, False, False, False])
        
        # Sampling rate slider
        y_pos = 0.3
        self.sampling_slider = Slider(plt.axes([0.7, y_pos, 0.2, 0.03]), 'Sample Rate', 1, 50, valinit=20, valfmt='%d Hz')
        
        # Connect button callbacks
        self.btn_start.on_clicked(self.start_monitoring)
        self.btn_stop.on_clicked(self.stop_monitoring)
        self.btn_save.on_clicked(self.save_current_data)
        self.btn_new_session.on_clicked(self.new_test_session)
        self.btn_compare.on_clicked(self.compare_sessions)
        self.btn_clear.on_clicked(self.clear_current_data)
        self.btn_connect.on_clicked(self.reconnect_arduino)
        self.btn_power_on.on_clicked(self.power_on_sensor)
        self.btn_power_off.on_clicked(self.power_off_sensor)
        self.sensor_selector.on_clicked(self.change_sensor_type)
        self.sampling_slider.on_changed(self.update_sampling_rate)
        
    def connect_arduino(self):
        """Connect to Arduino"""
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud_rate, timeout=2)
            time.sleep(2)  # Allow Arduino to reset
            logger.info(f"Connected to Arduino on {self.arduino_port}")
            self.update_status("Arduino Connected")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            self.update_status(f"Connection Failed: {e}")
            return False
            
    def reconnect_arduino(self, event):
        """Reconnect to Arduino"""
        if self.arduino:
            try:
                self.arduino.close()
            except:
                pass
        self.connect_arduino()
        
    def power_on_sensor(self, event):
        """Turn on sensor power"""
        if not self.arduino:
            self.update_status("Arduino not connected")
            return
            
        try:
            self.arduino.write(b'POWER_ON\\n')
            response = self.arduino.readline().decode().strip()
            if "POWER_ON_OK" in response:
                self.update_status("Sensor powered ON")
                logger.info("Sensor powered on successfully")
            else:
                self.update_status(f"Power on failed: {response}")
        except Exception as e:
            logger.error(f"Failed to power on sensor: {e}")
            self.update_status(f"Power on error: {e}")
            
    def power_off_sensor(self, event):
        """Turn off sensor power"""
        if not self.arduino:
            self.update_status("Arduino not connected")
            return
            
        try:
            self.arduino.write(b'POWER_OFF\\n')
            response = self.arduino.readline().decode().strip()
            if "POWER_OFF_OK" in response:
                self.update_status("Sensor powered OFF")
                logger.info("Sensor powered off successfully")
            else:
                self.update_status(f"Power off failed: {response}")
        except Exception as e:
            logger.error(f"Failed to power off sensor: {e}")
            self.update_status(f"Power off error: {e}")
            
    def start_monitoring(self, event):
        """Start real-time monitoring"""
        if self.monitoring:
            return
            
        if not self.arduino:
            self.update_status("Arduino not connected")
            return
            
        # Clear previous data for new session
        self.time_data.clear()
        self.sensor_data.clear()
        self.voltage_data.clear()
        self.raw_adc_data.clear()
        
        self.monitoring = True
        self.start_time = time.time()
        
        # Start monitoring thread
        self.monitoring_thread = threading.Thread(target=self.monitoring_worker)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
        
        # Start animation
        self.animation = animation.FuncAnimation(self.fig, self.update_plots, 
                                               interval=self.update_interval, blit=False)
        
        self.update_status(f"Monitoring started - {self.current_sensor_type}")
        logger.info(f"Started monitoring with {self.current_sensor_type}")
        
    def stop_monitoring(self, event):
        """Stop real-time monitoring"""
        if not self.monitoring:
            return
            
        self.monitoring = False
        
        if hasattr(self, 'animation'):
            self.animation.event_source.stop()
            
        self.update_status("Monitoring stopped")
        logger.info("Monitoring stopped")
        
        # Save session data
        if len(self.time_data) > 0:
            self.save_session_data()
            
    def monitoring_worker(self):
        """Worker thread for sensor data collection"""
        sample_interval = 1.0 / self.sampling_rate
        
        while self.monitoring and self.arduino:
            try:
                # Request measurement
                self.arduino.write(b'READ\\n')
                response = self.arduino.readline().decode().strip()
                
                if response.startswith('HEIGHT:'):
                    # Parse sensor reading
                    height_str = response.replace('HEIGHT:', '')
                    height = float(height_str)
                    
                    # Get additional status info
                    self.arduino.write(b'STATUS\\n')
                    status_response = self.arduino.readline().decode().strip()
                    
                    # Parse status for voltage and raw ADC
                    voltage = 0.0
                    raw_adc = 0
                    if status_response.startswith('STATUS:'):
                        parts = status_response.split(',')
                        for part in parts:
                            if 'VOLTAGE=' in part:
                                voltage = float(part.split('=')[1])
                            elif 'RAW=' in part:
                                raw_adc = int(part.split('=')[1])
                    
                    # Calculate timestamp
                    current_time = time.time() - self.start_time
                    
                    # Queue the data
                    self.data_queue.put((current_time, height, voltage, raw_adc))
                    
                else:
                    logger.warning(f"Unexpected Arduino response: {response}")
                    
                time.sleep(sample_interval)
                
            except Exception as e:
                logger.error(f"Monitoring error: {e}")
                break
                
    def update_plots(self, frame):
        """Update real-time plots"""
        # Get new data from queue
        new_data = []
        try:
            while True:
                new_data.append(self.data_queue.get_nowait())
        except Empty:
            pass
            
        # Add new data to storage
        for timestamp, height, voltage, raw_adc in new_data:
            self.time_data.append(timestamp)
            self.sensor_data.append(height if height >= 0 else np.nan)
            self.voltage_data.append(voltage)
            self.raw_adc_data.append(raw_adc)
            
        # Limit data size for performance
        if len(self.time_data) > self.max_display_points:
            self.time_data = self.time_data[-self.max_display_points:]
            self.sensor_data = self.sensor_data[-self.max_display_points:]
            self.voltage_data = self.voltage_data[-self.max_display_points:]
            self.raw_adc_data = self.raw_adc_data[-self.max_display_points:]
            
        # Update main plot
        if len(self.time_data) > 0:
            self.line_main.set_data(self.time_data, self.sensor_data)
            self.ax_main.relim()
            self.ax_main.autoscale_view()
            
            # Update voltage plot
            self.line_voltage.set_data(self.time_data, self.voltage_data)
            self.ax_voltage.relim()
            self.ax_voltage.autoscale_view()
            
            # Update statistics
            self.update_statistics()
            
        return self.line_main, self.line_voltage
        
    def update_statistics(self):
        """Update live statistics display"""
        self.ax_stats.clear()
        self.ax_stats.axis('off')
        
        if len(self.sensor_data) == 0:
            return
            
        # Calculate statistics for recent data
        recent_data = self.sensor_data[-self.stats_window:] if len(self.sensor_data) > self.stats_window else self.sensor_data
        recent_voltage = self.voltage_data[-self.stats_window:] if len(self.voltage_data) > self.stats_window else self.voltage_data
        
        # Remove NaN values for statistics
        valid_data = [x for x in recent_data if not np.isnan(x)]
        valid_voltage = [x for x in recent_voltage if not np.isnan(x)]
        
        if len(valid_data) > 0:
            stats_text = f"LIVE STATISTICS\\n"
            stats_text += f"=" * 15 + "\\n"
            stats_text += f"Points: {len(self.sensor_data)}\\n"
            stats_text += f"Time: {self.time_data[-1]:.1f}s\\n\\n"
            stats_text += f"DISTANCE (mm):\\n"
            stats_text += f"Current: {valid_data[-1]:.2f}\\n"
            stats_text += f"Mean: {np.mean(valid_data):.2f}\\n"
            stats_text += f"Std: {np.std(valid_data):.2f}\\n"
            stats_text += f"Min: {np.min(valid_data):.2f}\\n"
            stats_text += f"Max: {np.max(valid_data):.2f}\\n\\n"
            
            if len(valid_voltage) > 0:
                stats_text += f"VOLTAGE (V):\\n"
                stats_text += f"Current: {valid_voltage[-1]:.3f}\\n"
                stats_text += f"Mean: {np.mean(valid_voltage):.3f}\\n"
                stats_text += f"Range: {np.max(valid_voltage)-np.min(valid_voltage):.3f}"
        else:
            stats_text = "NO VALID DATA\\n\\nCheck sensor\\nconnection and\\nrange settings"
            
        self.ax_stats.text(0.05, 0.95, stats_text, transform=self.ax_stats.transAxes,
                         verticalalignment='top', fontsize=9,
                         bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
                         
    def update_status(self, message: str):
        """Update status in configuration panel"""
        self.ax_config.clear()
        self.ax_config.axis('off')
        
        config_text = f"SENSOR: {self.current_sensor_type}\\n"
        config_text += f"=" * 20 + "\\n"
        
        sensor_config = self.sensor_configs[self.current_sensor_type]
        config_text += f"Range: {sensor_config['range_mm'][0]}-{sensor_config['range_mm'][1]} mm\\n"
        config_text += f"Voltage: {sensor_config['voltage_range'][0]}-{sensor_config['voltage_range'][1]} V\\n"
        config_text += f"Sample Rate: {self.sampling_rate} Hz\\n\\n"
        config_text += f"STATUS:\\n{message}"
        
        self.ax_config.text(0.05, 0.95, config_text, transform=self.ax_config.transAxes,
                          verticalalignment='top', fontsize=9,
                          bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
                          
    def change_sensor_type(self, label):
        """Change sensor type configuration"""
        self.current_sensor_type = label
        
        # Update plot colors
        sensor_config = self.sensor_configs[label]
        self.line_main.set_color(sensor_config['color'])
        
        self.update_status(f"Sensor type changed to {label}")
        logger.info(f"Sensor type changed to {label}")
        
    def update_sampling_rate(self, val):
        """Update sampling rate"""
        self.sampling_rate = int(val)
        self.update_status(f"Sampling rate: {self.sampling_rate} Hz")
        
    def on_key_press(self, event):
        """Handle keyboard shortcuts"""
        if event.key == 's':
            self.save_current_data(None)
        elif event.key == ' ':  # Spacebar
            if self.monitoring:
                self.stop_monitoring(None)
            else:
                self.start_monitoring(None)
        elif event.key == 'c':
            self.clear_current_data(None)
        elif event.key == 'n':
            self.new_test_session(None)
        elif event.key == 'p':
            if self.arduino:
                self.power_on_sensor(None)
        elif event.key == 'o':
            if self.arduino:
                self.power_off_sensor(None)
                
    def save_current_data(self, event):
        """Save current data and plot"""
        if len(self.time_data) == 0:
            self.update_status("No data to save")
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_filename = f"sensor_test_{self.current_sensor_type}_{timestamp}"
        
        # Save CSV data
        csv_filename = f"{base_filename}.csv"
        try:
            with open(csv_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Time_s', 'Distance_mm', 'Voltage_V', 'Raw_ADC', 'Sensor_Type'])
                
                for i in range(len(self.time_data)):
                    writer.writerow([
                        self.time_data[i],
                        self.sensor_data[i],
                        self.voltage_data[i],
                        self.raw_adc_data[i],
                        self.current_sensor_type
                    ])
                    
            logger.info(f"Data saved to {csv_filename}")
        except Exception as e:
            logger.error(f"Failed to save CSV: {e}")
            
        # Save plot
        plot_filename = f"{base_filename}.png"
        try:
            self.fig.savefig(plot_filename, dpi=300, bbox_inches='tight')
            logger.info(f"Plot saved to {plot_filename}")
        except Exception as e:
            logger.error(f"Failed to save plot: {e}")
            
        # Save session metadata
        json_filename = f"{base_filename}_metadata.json"
        try:
            metadata = {
                'timestamp': timestamp,
                'sensor_type': self.current_sensor_type,
                'sampling_rate': self.sampling_rate,
                'duration': self.time_data[-1] if self.time_data else 0,
                'data_points': len(self.time_data),
                'statistics': {
                    'distance_mean': float(np.nanmean(self.sensor_data)),
                    'distance_std': float(np.nanstd(self.sensor_data)),
                    'distance_min': float(np.nanmin(self.sensor_data)),
                    'distance_max': float(np.nanmax(self.sensor_data)),
                    'voltage_mean': float(np.mean(self.voltage_data)),
                    'voltage_std': float(np.std(self.voltage_data))
                }
            }
            
            with open(json_filename, 'w') as jsonfile:
                json.dump(metadata, jsonfile, indent=2)
                
            logger.info(f"Metadata saved to {json_filename}")
        except Exception as e:
            logger.error(f"Failed to save metadata: {e}")
            
        self.update_status(f"Data saved: {base_filename}")
        
    def save_session_data(self):
        """Save current session to internal storage"""
        if len(self.time_data) == 0:
            return
            
        session_data = {
            'sensor_type': self.current_sensor_type,
            'timestamp': datetime.now().isoformat(),
            'sampling_rate': self.sampling_rate,
            'time_data': self.time_data.copy(),
            'sensor_data': self.sensor_data.copy(),
            'voltage_data': self.voltage_data.copy(),
            'raw_adc_data': self.raw_adc_data.copy()
        }
        
        self.test_sessions[self.current_session_name] = session_data
        logger.info(f"Session {self.current_session_name} saved to memory")
        
    def new_test_session(self, event):
        """Start a new test session"""
        # Save current session if it has data
        if len(self.time_data) > 0:
            self.save_session_data()
            
        # Create new session name
        self.session_counter += 1
        self.current_session_name = f"Test_{self.session_counter}"
        
        # Clear current data
        self.clear_current_data(None)
        
        self.update_status(f"New session: {self.current_session_name}")
        logger.info(f"Started new session: {self.current_session_name}")
        
    def clear_current_data(self, event):
        """Clear current data"""
        self.time_data.clear()
        self.sensor_data.clear()
        self.voltage_data.clear()
        self.raw_adc_data.clear()
        
        # Clear plots
        self.line_main.set_data([], [])
        self.line_voltage.set_data([], [])
        self.ax_main.relim()
        self.ax_voltage.relim()
        
        self.update_status("Data cleared")
        
    def compare_sessions(self, event):
        """Compare multiple test sessions"""
        if len(self.test_sessions) < 2:
            self.update_status("Need at least 2 sessions to compare")
            return
            
        # Create comparison plot
        fig_compare, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        fig_compare.suptitle('Sensor Comparison Analysis')
        
        colors = ['blue', 'red', 'green', 'purple', 'orange', 'brown']
        
        for i, (session_name, session_data) in enumerate(self.test_sessions.items()):
            color = colors[i % len(colors)]
            label = f"{session_name} ({session_data['sensor_type']})"
            
            # Plot distance data
            ax1.plot(session_data['time_data'], session_data['sensor_data'], 
                    color=color, label=label, alpha=0.7)
                    
            # Plot voltage data
            ax2.plot(session_data['time_data'], session_data['voltage_data'], 
                    color=color, label=label, alpha=0.7)
                    
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Distance (mm)')
        ax1.set_title('Distance Measurements Comparison')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Voltage (V)')
        ax2.set_title('Voltage Output Comparison')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save comparison plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        comparison_filename = f"sensor_comparison_{timestamp}.png"
        fig_compare.savefig(comparison_filename, dpi=300, bbox_inches='tight')
        
        plt.show()
        
        self.update_status(f"Comparison saved: {comparison_filename}")
        logger.info(f"Session comparison plot saved: {comparison_filename}")
        
    def shutdown(self):
        """Safely shutdown the program"""
        if self.monitoring:
            self.stop_monitoring(None)
            
        if self.arduino:
            try:
                self.arduino.write(b'POWER_OFF\\n')
                time.sleep(0.5)
                self.arduino.close()
                logger.info("Arduino connection closed")
            except:
                pass
                
        logger.info("Sensor test program shutdown complete")

def main():
    """Main function to run the sensor tester"""
    # Get Arduino port from user if needed
    arduino_port = "COM3"  # Default
    
    try:
        # Create and run the sensor tester
        tester = SensorTester(arduino_port=arduino_port)
        
        # Initial status
        tester.update_status("Ready - Press Start to begin")
        
        # Show instructions
        instructions = """
KEYBOARD SHORTCUTS:
- SPACE: Start/Stop monitoring
- S: Save current data  
- C: Clear data
- N: New test session
- P: Power on sensor
- O: Power off sensor

USAGE:
1. Connect sensor to Arduino
2. Select sensor type
3. Press Start or SPACE
4. Move objects in sensor range
5. Press Save or S to capture data
6. Use New Test for different sensors
7. Use Compare to analyze results
        """
        
        print(instructions)
        logger.info("Sensor test program started")
        
        # Show the GUI
        plt.show()
        
    except KeyboardInterrupt:
        logger.info("Program interrupted by user")
    except Exception as e:
        logger.error(f"Program error: {e}")
    finally:
        if 'tester' in locals():
            tester.shutdown()

if __name__ == "__main__":
    main()