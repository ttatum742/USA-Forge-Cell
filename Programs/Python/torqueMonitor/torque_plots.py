from cpppo.server.enip.get_attribute import proxy_simple
import numpy as np
import time
import logging
from typing import Tuple, List, Optional
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
from threading import Thread
import queue
from scipy.signal import butter, lfilter

# Configure logging
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class RealTimePlotter:
    def __init__(self, max_points=1000):
        self.max_points = max_points
        self.data_queue = queue.Queue()
        
        # Create 6 subplots (one per joint)
        self.fig, self.axes = plt.subplots(6, 1, figsize=(10, 12))
        plt.subplots_adjust(hspace=0.5)  # Add spacing between subplots
        
        # Data storage
        self.time_data = []
        self.torque_data = [[] for _ in range(6)]
        self.torque_lines = []
        
        # Configure each joint plot
        joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        colors = ['r', 'g', 'b', 'c', 'm', 'y']
        y_limits = [(-100, 100), (-150, 150), (-100, 100), 
                   (-50, 50), (-30, 30), (-30, 30)]  # Adjust based on your torque ranges
        
        for i, (ax, name, color, ylim) in enumerate(zip(self.axes, joint_names, colors, y_limits)):
            ax.set_title(f'{name} Torque (Nm)')
            ax.set_ylim(ylim)
            line, = ax.plot([], [], color=color)
            self.torque_lines.append(line)
            ax.grid(True)
        
        # Thread control
        self.running = False
        self.lock = threading.Lock()
        self.update_thread = None
        
        # Animation
        self.ani = animation.FuncAnimation(
            self.fig, self._update_plot, interval=50, blit=True, cache_frame_data=False)

    def _is_running(self):
        """Thread-safe running check."""
        with self.lock:
            return self.running

    def _update_plot(self, frame):
        """Process all available data in queue and update plots"""
        while not self.data_queue.empty():
            try:
                timestamp, torques, _ = self.data_queue.get_nowait()  # Ignore forces
                
                # Update time data
                self.time_data.append(timestamp)
                if len(self.time_data) > self.max_points:
                    self.time_data.pop(0)
                
                # Update torque data for each joint
                for i in range(6):
                    self.torque_data[i].append(torques[i])
                    if len(self.torque_data[i]) > self.max_points:
                        self.torque_data[i].pop(0)
                    
                    # Ensure equal array lengths
                    x_data = self.time_data[-len(self.torque_data[i]):]
                    y_data = self.torque_data[i]
                    self.torque_lines[i].set_data(x_data, y_data)
                
                # Adjust x-axis limits for all subplots
                if len(self.time_data) > 1:
                    xmin, xmax = self.time_data[0], self.time_data[-1]
                    if xmax - xmin > 0.001:  # Prevent singular transform
                        for ax in self.axes:
                            ax.set_xlim(xmin, xmax)
                
            except queue.Empty:
                break
        
        return self.torque_lines

    def add_data(self, timestamp, torques, forces=None):
        """Thread-safe data addition (forces parameter kept for compatibility)"""
        if len(torques) == 6:
            self.data_queue.put((timestamp, list(torques), None))

    def start_background_update(self, robot, update_rate=10.0):
        """Start data collection thread"""
        self.running = True
        self.update_thread = threading.Thread(
            target=self._update_real_data_loop,
            args=(robot, update_rate),
            daemon=True
        )
        self.update_thread.start()

    def _update_real_data_loop(self, robot, update_rate):
        """Background thread for data collection"""
        while self._is_running():
            start_time = time.time()
            
            try:
                torques = robot._read_torques()
                print(f"Torques: {torques}")
                if torques is not None:
                    self.add_data(time.time(), torques)
            except Exception as e:
                print(f"Data error: {str(e)}")
            
            # Maintain update rate
            elapsed = time.time() - start_time
            time.sleep(max(0, (1.0/update_rate) - elapsed))

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
        
        # First draw before entering loop
        self.fig.canvas.draw()
        plt.show(block=False)
        
        try:
            while self._is_running():
                # Force redraw and process GUI events
                self.fig.canvas.flush_events()
                plt.pause(0.05)
        except KeyboardInterrupt:
            self.stop()

class FanucRobot(proxy_simple):
    """Fanuc Robot class for communication & torque-to-force conversion"""
    ## Inherits from proxy_simple for communication with the robot controller
    ## and implements methods for torque-to-force conversion using the robot's kinematics.

                # Define parameters to read/write from robot
    PARAMETERS = dict(proxy_simple.PARAMETERS,
        j1t=proxy_simple.parameter('@0x6B/1/50', 'DINT', 'Nm'),
        j2t=proxy_simple.parameter('@0x6B/1/51', 'DINT', 'Nm'),
        j3t=proxy_simple.parameter('@0x6B/1/52', 'DINT', 'Nm'),
        j4t=proxy_simple.parameter('@0x6B/1/53', 'DINT', 'Nm'),
        j5t=proxy_simple.parameter('@0x6B/1/54', 'DINT', 'Nm'),
        j6t=proxy_simple.parameter('@0x6B/1/55', 'DINT', 'Nm'),
        j1a=proxy_simple.parameter('@0x6B/1/60', 'DINT', 'Degrees'),
        j2a=proxy_simple.parameter('@0x6B/1/61', 'DINT', 'Degrees'),
        j3a=proxy_simple.parameter('@0x6B/1/62', 'DINT', 'Degrees'),
        j4a=proxy_simple.parameter('@0x6B/1/63', 'DINT', 'Degrees'),
        j5a=proxy_simple.parameter('@0x6B/1/64', 'DINT', 'Degrees'),
        j6a=proxy_simple.parameter('@0x6B/1/65', 'DINT', 'Degrees'),
        fx=proxy_simple.parameter('@0x6B/1/70', 'DINT', 'N'),
        fy=proxy_simple.parameter('@0x6B/1/71', 'DINT', 'N'),
        fz=proxy_simple.parameter('@0x6B/1/72', 'DINT', 'N'),
    )

    def __init__(self, ip_address: str):
        """Initialize robot connection and params"""
                # Init eth/ip connection
        super().__init__(host=ip_address)
        self.ip_address = ip_address

                # Low-pass filter coefficients
        self.alpha = 0.9  # Smoothing factor (0 < alpha < 1)
        self.filtered_force = np.zeros(3)
        self.filtered_torque = np.zeros(6)

                # Butterworth filter coefficients
        self.filter_order = 3
        self.cutoff_freq = 5.0  # Hz (start with 1/2 of your update rate)
        self.prev_torques = np.zeros(6)  # init previous torques for filtering
        self.torque_filter_states = [{
            'b': None,    # Initialize filter states for each joint
            'a': None,
            #'prev_torques': np.zeros(6)
        } for _ in range(6)]

                # Spike detection parameters
       # self.derivative_threshold = 100  # N/s (adjust based on your needs)
       # self.prev_forces = None
       # self.prev_time = None
       # self.derivative_data = [[] for _ in range(3)]  # Stores dF/dt values

                # Angle filtering parameters
        self.angle_window_size = 3  # Adjust based on update rate
        self.angle_history = [[] for _ in range(6)]  # Store last N angles per joint

                # FANUC M-10iD/16S DH Parameters (in meters, radians)
        self.dh_params = [
                        [0.075,  -1.571,0,      0],  # Joint 1
                        [0.440,  0,     0,      0],  # Joint 2
                        [0.195, -1.571, 0,      0],  # Joint 3
                        [0,      1.571, 0.555, 0],  # Joint 4
                        [0,     -1.571, 0,      0],  # Joint 5
                        [0,      0,     0.090, 0]   # Joint 6
                        ]
        self.torque_window = []  # For moving average of torques
        self.filtered_angles = np.zeros(6)  # For exponential smoothing
        self.angle_alpha = 0.2  # Smoothing factor (0.1-0.3 recommended)


    def _read_parameter(self, param_name: str) -> Optional[float]:
        """Read a single parameter value with proper error handling"""
        try:
            # Get the parameter string
            param_str = self.parameter_substitution(param_name)
            
            # Read the parameter - handle multiple return values
            result, = self.read(param_str, checking=True)
            # Debug logging
            logger.debug(f"Raw read result for {param_name}: {result,}")
            
            # Extract the value from the result
            return float(result[0]) if isinstance(result, list) else float(result)
            
        except Exception as e:
            logger.error(f"Failed to read {param_name}: {str(e)}", exc_info=True)
            return None

    def _read_torques(self) -> Optional[List[float]]:
        """Read all joint torques"""
        torques = []
        for joint in ['j1t', 'j2t', 'j3t', 'j4t', 'j5t', 'j6t']:
            torque = self._read_parameter(joint)
            if torque is None:
                logger.error(f"Failed to read {joint}")
                return None
            torques.append(torque)
            logger.info(f"Read {joint}: {torque:.2f} Nm")
        return torques

    def _read_joint_angles(self) -> Optional[List[float]]:
        """Read all joint angles (in radians)"""
        angles = []
        for joint in ['j1a', 'j2a', 'j3a', 'j4a', 'j5a', 'j6a']:
            angle = self._read_parameter(joint)
            if angle is None:
                logger.error(f"Failed to read {joint}")
                return None
            angles.append(angle)
            logger.info(f"Read {joint}: {np.degrees(angle):.2f}ï¿½")
        return angles
    
    def _write_parameter(self, param_name: str, value: float) -> Optional[float]:
        """Write a single parameter value with proper error handling"""
        try:
            # Get the parameter string
            param = '%s = (DINT) %s' % (param_name, int(value))
            success, = self.write(
                self.parameter_substitution(param), checking=True)
            if success: print(f"Successfully wrote {param_name}: {value:.2f}")
            logger.info(f"Wrote {param_name}: {value:.2f}")
        except Exception as e:
            logging.warning("Failed to write %s: %s", param_name, str(e), exc_info=True)
            self.close_gateway(exc=e)
            raise
    
    def _write_forces(self, forces: np.ndarray) -> bool:
        """Write forces to the robot controller"""
        if forces.shape != (3,):
            raise ValueError("Forces must be a 3-element vector")
        try:
            for i, force in enumerate(forces):
                param_name = ['fx', 'fy', 'fz'][i]
                self._write_parameter(param_name, force)
                logger.info(f"Wrote {param_name}: {force:.2f} N")
            return True
        except Exception as e:
            logger.error(f"Failed to write forces: {str(e)}", exc_info=True)
            return None



    # ======================
    # Kinematics and Control Methods
    # ======================
    def _dh_transform(self, a: float, alpha: float, d: float, theta: float) -> np.ndarray:
        """Compute DH transformation matrix"""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_angles: List[float]) -> Tuple[np.ndarray, np.ndarray]:
        """Compute end-effector pose from joint angles"""
        T = np.eye(4)
        for i in range(len(joint_angles)):
            a, alpha, d, theta = self.dh_params[i]
            theta += joint_angles[i]
            Ti = self._dh_transform(a, alpha, d, theta)
            T = T @ Ti
        return T[:3, 3], T[:3, :3]

    def geometric_jacobian(self, joint_angles: List[float]) -> np.ndarray:
        """Compute the geometric Jacobian matrix"""
        num_joints = len(joint_angles)
        J = np.zeros((6, num_joints))
        T = np.eye(4)
        joint_positions = []
        joint_axes = []
        
        for i in range(num_joints):
            a, alpha, d, theta = self.dh_params[i]
            theta += joint_angles[i]
            Ti = self._dh_transform(a, alpha, d, theta)
            T = T @ Ti
            joint_positions.append(T[:3, 3])
            joint_axes.append(T[:3, 2])
        
        p_ee = T[:3, 3]
        for i in range(num_joints):
            z_i = joint_axes[i]
            p_i = joint_positions[i]
            J[:3, i] = np.cross(z_i, (p_ee - p_i))
            J[3:, i] = z_i
        return J

    def compute_cartesian_force(self) -> Optional[np.ndarray]: # NEED TO MOVE DATA PROCESSING HERE
        """Compute Cartesian force in tool frame"""
        raw_angles = self._read_joint_angles()
        raw_torques = self._read_torques()
        ma_torques = self.filter_torque_mavg(raw_torques)  # Filtered torques for calculation
        filt_torques = self.filter_torque_butter(ma_torques)
        joint_angles = self.filter_angles(raw_angles)  # Filtered angles for calculation
        if joint_angles is None or filt_torques is None:
            return None
            
        J = self.geometric_jacobian(joint_angles)
        F_base = np.linalg.pinv(J.T) @ filt_torques
        _, rotation = self.forward_kinematics(joint_angles)
        return rotation.T @ F_base[:3]

    # ======================
    # Data processing Methods
    # ======================   
    def filter_force_lp(self, force: np.ndarray) -> np.ndarray:
        """Apply low-pass filter to the force vector"""
        self.filtered_force = self.alpha * force + (1 - self.alpha) * self.filtered_force
        return self.filtered_force
    
    def filter_torque_lp(self, force: np.ndarray) -> np.ndarray:            # MOVE CONFIG TO SEPARATE FUNCTION AND RUN ONCE INSTEAD OF EVERY TIME
        """Apply low-pass filter to the incoming joint torques"""
        self.filtered_torque = self.alpha * force + (1 - self.alpha) * self.filtered_torque
        return self.filtered_torque

    def _configure_butter(self, update_rate):
        """Initialize Butterworth filters for each joint"""
        self.filter_order = 2  # Keep this moderate
        self.fs = update_rate  # Sampling rate (Hz)
        
        # Critical: cutoff must be < fs/2 (Nyquist frequency)
        self.cutoff_freq = min(4.0, self.fs * 0.49)  # Never exceed 49% of fs/2
        
        self.b, self.a = butter(
            N=self.filter_order,
            Wn=self.cutoff_freq,
            fs=self.fs,
            btype='lowpass'
        )
        print(f"Initialized {self.filter_order}-order Butterworth @ {self.cutoff_freq:.1f}Hz (fs={self.fs}Hz)")

    def filter_torque_butter(self, raw_torques):
        """Safe torque filtering with minimum data checks"""

        if not hasattr(self, 'prev_torques'):
            self.prev_torques = np.zeros(6)
            return raw_torques  # Return first reading unfiltered

        try:
            filtered = np.zeros(6)
            for i in range(6):
                # Use simple IIR filter when data is scarce
                if not hasattr(self, 'b') or len(self.prev_torques) < 3:
                    # Simple exponential smoothing as fallback
                    alpha = 0.25  # Smoothing factor
                    filtered[i] = alpha * raw_torques[i] + (1-alpha) * self.prev_torques[i]
                else:
                    # Proper Butterworth when we have enough history
                    filtered[i] = lfilter(
                        self.b, self.a,
                        [self.prev_torques[i], raw_torques[i]]
                    )[-1]

            self.prev_torques = filtered.copy()
            return filtered

        except Exception as e:
            print(f"Filter fallback: {str(e)}")
            return raw_torques  # Graceful degradation
        
    def filter_torque_mavg(self, raw_torques):
        self.torque_window.append(raw_torques)
        if len(self.torque_window) > 3:  # 2.6s window at 1.16Hz
            self.torque_window.pop(0)
        ma_torques = np.mean(self.torque_window, axis=0)
        return ma_torques
    
    def _detect_spikes(self, forces):
        """Identify sudden force spikes using simple difference method"""
        spikes = [0, 0, 0]
        
        if len(self.force_history) > 0:
            # Compare with last recorded force
            last_forces = self.force_history[-1]
            for i in range(3):
                if abs(forces[i] - last_forces[i]) > self.spike_threshold:
                    spikes[i] = 1  # Binary spike indicator
        
        # Maintain history size
        self.force_history.append(forces)
        if len(self.force_history) > self.history_size:
            self.force_history.pop(0)
            
        return spikes


    def filter_angles(self, raw_angles):
        """Apply moving average to joint angles"""
        if not hasattr(self, 'angle_history'):
            self.angle_window_size = 5  # 4.3s window at 1.16Hz
            self.angle_history = [[] for _ in range(6)]
            self.filtered_angles = np.zeros(6)
        filtered = np.zeros(6)
        for i in range(6):
            self.angle_history[i].append(raw_angles[i])
            if len(self.angle_history[i]) > self.angle_window_size:
                self.angle_history[i].pop(0)
            med_angles = np.median(self.angle_history[i])
            self.filtered_angles[i] = 0.2*med_angles +0.8*self.filtered_angles[i]  # Exponential smoothing
            filtered[i] = self.filtered_angles[i]
        return filtered

    # ======================
    # Main Control Loop
    # ======================
    def run(self, update_rate: float = 10.0):
        """Main control loop (plotting)"""
        ## This program's main loop is drastically different from the original torque_to_force program.
        ## The original program was designed to prioritize program write speed, while this program is for debugging and visualization.
        try:
            print("Starting real-time monitoring...")
            plotter = RealTimePlotter()

            plotter.start_background_update(self, update_rate)

            print("Data collection active - opening plot window...")
            plotter.show()  # This blocks until window closes

        except Exception as e:
            print(f"Error: {str(e)}")
        finally:
            plotter.stop()
            print("Stopped all threads")


if __name__ == "__main__":
    # Initialize with debug logging
    robot = FanucRobot(ip_address="192.168.0.1")
    
    
    # Start main control loop (10Hz update rate)
    robot.run(update_rate=7.0)