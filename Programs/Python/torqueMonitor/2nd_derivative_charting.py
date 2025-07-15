from typing import List, Optional
from cpppo.server.enip.get_attribute import proxy_simple
import numpy as np
import time
import logging
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
from threading import Thread
import queue

# Configure logging
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class RealTimeDerivativePlotter:
    def __init__(self, max_points=500):
        self.max_points = max_points
        self.data_queue = queue.Queue()
        self.prev_torques = None
        self.prev_derivatives = None
        self.alpha = 0.3  # Filter coefficient for derivatives
        self.filtered_derivatives = np.zeros(6)
        self.filtered_second_derivatives = np.zeros(6)
        
        # Create figure with 12 subplots (6 for 1st derivative, 6 for 2nd derivative)
        self.fig, self.axes = plt.subplots(12, 1, figsize=(12, 18))
        plt.subplots_adjust(hspace=0.4)
        
        # Data storage
        self.time_data = []
        self.derivative_data = [[] for _ in range(6)]
        self.second_derivative_data = [[] for _ in range(6)]
        self.plot_lines = []
        self.second_plot_lines = []
        
        # Configure each joint plot
        joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        colors = ['r', 'g', 'b', 'c', 'm', 'y']
        
        # Create plots for first derivatives
        for i, (ax, name, color) in enumerate(zip(self.axes[:6], joint_names, colors)):
            ax.set_title(f'{name} Torque 1st Derivative (Nm/s)')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('dτ/dt')
            line, = ax.plot([], [], color=color, lw=1.5)
            self.plot_lines.append(line)
            ax.grid(True)
        
        # Create plots for second derivatives
        for i, (ax, name, color) in enumerate(zip(self.axes[6:], joint_names, colors)):
            ax.set_title(f'{name} Torque 2nd Derivative (Nm/s²)')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('d²τ/dt²')
            line, = ax.plot([], [], color=color, lw=1.5)
            self.second_plot_lines.append(line)
            ax.grid(True)
        
        # Thread control
        self.running = False
        self.lock = threading.Lock()
        self.update_thread = None
        self.prev_torques = None
        self.prev_derivatives = None
        
        # Animation
        self.ani = animation.FuncAnimation(
            self.fig, self._update_plot, interval=20, blit=True)

    def _is_running(self):
        with self.lock:
            return self.running

    def _update_plot(self, frame):
        """Process all queued data and update plots"""
        while not self.data_queue.empty():
            try:
                timestamp, derivatives, second_derivatives = self.data_queue.get_nowait()
                
                # Update time data
                self.time_data.append(timestamp)
                if len(self.time_data) > self.max_points:
                    self.time_data.pop(0)
                
                # Update derivative data
                for i in range(6):
                    # First derivative
                    self.derivative_data[i].append(derivatives[i])
                    if len(self.derivative_data[i]) > self.max_points:
                        self.derivative_data[i].pop(0)
                    
                    # Second derivative
                    self.second_derivative_data[i].append(second_derivatives[i])
                    if len(self.second_derivative_data[i]) > self.max_points:
                        self.second_derivative_data[i].pop(0)
                    
                    # Update plot data
                    x_data = self.time_data[-len(self.derivative_data[i]):]
                    
                    # First derivative plot
                    y_data = self.derivative_data[i]
                    self.plot_lines[i].set_data(x_data, y_data)
                    
                    # Second derivative plot
                    y2_data = self.second_derivative_data[i]
                    self.second_plot_lines[i].set_data(x_data, y2_data)
                
                # Adjust x-axis limits
                if len(self.time_data) > 1:
                    xmin, xmax = self.time_data[0], self.time_data[-1]
                    if xmax - xmin > 0.001:
                        for ax in self.axes:
                            ax.set_xlim(xmin, xmax)
                
                # Dynamically adjust y-axis limits for both derivatives
                for i in range(6):
                    # First derivative y-axis
                    if len(self.derivative_data[i]) > 0:
                        y_min = min(self.derivative_data[i])
                        y_max = max(self.derivative_data[i])
                        margin = max(0.1 * (y_max - y_min), 1.0)  # At least 1.0 margin
                        self.axes[i].set_ylim(y_min - margin, y_max + margin)
                    
                    # Second derivative y-axis
                    if len(self.second_derivative_data[i]) > 0:
                        y2_min = min(self.second_derivative_data[i])
                        y2_max = max(self.second_derivative_data[i])
                        margin = max(0.1 * (y2_max - y2_min), 1.0)  # At least 1.0 margin
                        self.axes[i+6].set_ylim(y2_min - margin, y2_max + margin)
                
            except queue.Empty:
                break
        
        return self.plot_lines + self.second_plot_lines

    def add_data(self, timestamp, derivatives, second_derivatives):
        """Thread-safe data addition"""
        if len(derivatives) == 6 and len(second_derivatives) == 6:
            self.data_queue.put((timestamp, list(derivatives), list(second_derivatives)))

    def start_background_update(self, robot, update_rate=100.0):
        """Start high-frequency data collection thread"""
        self.running = True
        self.update_thread = threading.Thread(
            target=self._update_derivative_loop,
            args=(robot, update_rate),
            daemon=True
        )
        self.update_thread.start()
    
    def _low_pass_filter_torques(self, new_torques: np.ndarray) -> np.ndarray:
        """Apply low-pass filter to raw torques before differentiation"""
        if not hasattr(self, 'filtered_torques'):
            self.filtered_torques = new_torques  # Initialize on first run
        self.filtered_torques = (self.alpha * new_torques + 
                            (1 - self.alpha) * self.filtered_torques)
        return self.filtered_torques
        
    def _update_derivative_loop(self, robot, update_rate):
        """Calculate and queue derivatives at high frequency"""
        dt = 1.0/update_rate
        while self._is_running():
            start_time = time.time()
            
            try:
                current_torques = robot._read_torques()
                if current_torques is None:
                    continue
                    
                current_torques = np.array(current_torques)
                filtered_torques = self._low_pass_filter_torques(current_torques)
                
                # Calculate derivatives
                if self.prev_torques is not None:
                    derivatives = (filtered_torques - self.prev_torques)/dt
                    
                    # Calculate second derivatives
                    if self.prev_derivatives is not None:
                        second_derivatives = (derivatives - self.prev_derivatives)/dt
                        self.add_data(time.time(), derivatives, second_derivatives)
                    
                    self.prev_derivatives = derivatives.copy()
                
                self.prev_torques = filtered_torques.copy()
                
            except Exception as e:
                print(f"Derivative calculation error: {str(e)}")
            
            # Maintain precise timing
            elapsed = time.time() - start_time
            time.sleep(max(0, dt - elapsed))

    def stop(self):
        """Clean shutdown"""
        self.running = False
        if self.update_thread is not None:
            self.update_thread.join()
        plt.close('all')

    def show(self):
        """Show the plot window"""
        plt.tight_layout()
        with self.lock:
            self.running = True
        
        self.fig.canvas.draw()
        plt.show(block=False)
        
        try:
            while self._is_running():
                self.fig.canvas.flush_events()
                plt.pause(0.02)
        except KeyboardInterrupt:
            self.stop()

class FanucRobot(proxy_simple):
    """Simplified robot class for torque reading only"""
    PARAMETERS = dict(proxy_simple.PARAMETERS,
        j1t=proxy_simple.parameter('@0x6B/1/50', 'DINT', 'Nm'),
        j2t=proxy_simple.parameter('@0x6B/1/51', 'DINT', 'Nm'),
        j3t=proxy_simple.parameter('@0x6B/1/52', 'DINT', 'Nm'),
        j4t=proxy_simple.parameter('@0x6B/1/53', 'DINT', 'Nm'),
        j5t=proxy_simple.parameter('@0x6B/1/54', 'DINT', 'Nm'),
        j6t=proxy_simple.parameter('@0x6B/1/55', 'DINT', 'Nm')
    )

    def __init__(self, ip_address: str):
        super().__init__(host=ip_address)
        self.ip_address = ip_address

    def _read_parameter(self, param_name: str) -> Optional[float]:
        try:
            param_str = self.parameter_substitution(param_name)
            result, = self.read(param_str, checking=True)
            return float(result[0]) if isinstance(result, list) else float(result)
        except Exception as e:
            logger.error(f"Failed to read {param_name}: {str(e)}")
            return None

    def _read_torques(self) -> Optional[List[float]]:
        torques = []
        for joint in ['j1t', 'j2t', 'j3t', 'j4t', 'j5t', 'j6t']:
            torque = self._read_parameter(joint)
            if torque is None:
                return None
            torques.append(torque)
        return torques

    def run(self, update_rate: float = 100.0):
        """Run derivative plotting"""
        try:
            print("Starting real-time derivative monitoring...")
            plotter = RealTimeDerivativePlotter()
            plotter.start_background_update(self, update_rate)
            plotter.show()
        except Exception as e:
            print(f"Error: {str(e)}")
        finally:
            plotter.stop()

if __name__ == "__main__":
    robot = FanucRobot(ip_address="192.168.0.1")
    # Run at 100Hz for responsive derivative calculation
    robot.run(update_rate=100.0)