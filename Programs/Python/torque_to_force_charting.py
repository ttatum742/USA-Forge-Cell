from cpppo.server.enip.get_attribute import proxy_simple
import numpy as np
import time
import logging
from typing import Tuple, List, Optional

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D
import threading
from threading import Thread
import queue

# Configure logging
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class RealTimePlotter:
    def __init__(self, max_points=1000):
        self.max_points = max_points
        self.data_queue = queue.Queue()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.running = False
        self.lock = threading.Lock()

        # Configure torque plot
        self.ax1.set_title('Joint Torques (Nm)')
        self.ax1.set_ylim(-1000, 1000)  # Adjust based on your expected range
        self.torque_lines = [
            Line2D([], [], color='r', label='J1'),
            Line2D([], [], color='g', label='J2'),
            Line2D([], [], color='b', label='J3'),
            Line2D([], [], color='c', label='J4'),
            Line2D([], [], color='m', label='J5'),
            Line2D([], [], color='y', label='J6')
        ]
        for line in self.torque_lines:
            self.ax1.add_line(line)
        self.ax1.legend(loc='upper right')
        
        # Configure force plot
        self.ax2.set_title('Cartesian Forces (N)')
        self.ax2.set_ylim(-2000, 2000)  # Adjust based on your expected range
        self.force_lines = [
            Line2D([], [], color='r', label='Fx'),
            Line2D([], [], color='g', label='Fy'),
            Line2D([], [], color='b', label='Fz')
        ]
        for line in self.force_lines:
            self.ax2.add_line(line)
        self.ax2.legend(loc='upper right')
        
        # Data storage
        self.time_data = []
        self.torque_data = [[] for _ in range(6)]
        self.force_data = [[] for _ in range(3)]
        
        # Start animation
        self.ani = animation.FuncAnimation(
            self.fig, self._update_plot, interval=50, blit=True, cache_frame_data=False)
        
    def _update_plot(self, frame):
        """Process all available data in queue and update plots"""
        qsize = self.data_queue.qsize()
        if qsize > 10:  # Only warn if significant backlog
            print(f"Queue backlog: {qsize} items waiting")
        while not self.data_queue.empty():
            try:
                timestamp, torques, forces = self.data_queue.get_nowait()
                
                # Update time data
                self.time_data.append(timestamp)
                if len(self.time_data) > self.max_points:
                    self.time_data.pop(0)
                
                # Update torque data
                for i in range(6):
                    self.torque_data[i].append(torques[i])
                    if len(self.torque_data[i]) > self.max_points:
                        self.torque_data[i].pop(0)
                    self.torque_lines[i].set_data(self.time_data, self.torque_data[i])
                
                # Update force data
                for i in range(3):
                    self.force_data[i].append(forces[i])
                    if len(self.force_data[i]) > self.max_points:
                        self.force_data[i].pop(0)
                    self.force_lines[i].set_data(self.time_data, self.force_data[i])
                
                # Adjust axes
                if self.time_data:
                    self.ax1.set_xlim(self.time_data[0], self.time_data[-1])
                    self.ax2.set_xlim(self.time_data[0], self.time_data[-1])
                
            except queue.Empty:
                break
        
        # Return all artists that need redrawing
        return self.torque_lines + self.force_lines

    def add_data(self, timestamp, torques, forces):
        """Thread-safe data addition"""
        # Ensure data is in correct format
        if len(torques) == 6 and len(forces) == 3:
            self.data_queue.put((timestamp, list(torques), list(forces)))
        else:
            print(f"Invalid data format. Expected 6 torques and 3 forces, got {len(torques)} torques and {len(forces)} forces")

    def _is_running(self):
        """Thread-safe running check."""
        with self.lock:
            return self.running
        
    def show(self):
        """Show and maintain the plot window"""
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

    def start_background_update(self, robot, update_rate=10.0):
        """Start a background thread that reads real data from the robot."""
        self.running = True
        self.update_thread = Thread(
            target=self._update_real_data_loop,
            args=(robot, update_rate),
            daemon=True
        )
        self.update_thread.start()

    def _update_real_data_loop(self, robot, update_rate):
        """Thread-safe data collection with proper NumPy handling"""
        while self._is_running():
            try:
                start_time = time.time()
                
                # 1. Get real data with proper None checks
                torques = robot._read_torques()
                torques = np.asarray(torques, dtype=np.float64)
                torques = robot.filter_torque(torques)  # Uncomment for pre-processing filter
                forces = robot.compute_cartesian_force()
                forces = np.asarray(forces, dtype=np.float64)

                # Proper NumPy array existence check
                if torques is None or isinstance(torques, (list, np.ndarray)) and len(torques) == 0:
                    print("Warning: No torque data received")
                    torques = np.zeros(6)
                
                if forces is None or isinstance(forces, (list, np.ndarray)) and len(forces) == 0:
                    print("Warning: No force data received")
                    forces = np.zeros(3)
                
                # 3. Apply filtering post-processing
                filtered_forces = robot.filter_force(forces)
                filtered_forces = forces  # Uncomment for no filtering output data

                # 4. Add to plot
                self.add_data(
                    timestamp=time.time(),
                    torques=torques.tolist(),  # Convert to list for queue
                    forces=filtered_forces.tolist()
                )
                
                # 5. Maintain update rate
                elapsed = time.time() - start_time
                time.sleep(max(0, (1.0/update_rate) - elapsed))
                
            except Exception as e:
                print(f"Data error: {str(e)}")
                time.sleep(1)  # Prevent tight error loop

    def stop(self):
        """Stop the background thread"""
        self.running = False
        if hasattr(self, 'update_thread'):
            self.update_thread.join()
        plt.close()


class FanucRobot(proxy_simple):
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
        super().__init__(host=ip_address)
        self.ip_address = ip_address
                # Low-pass filter coefficients
        self.alpha = 0.9  # Smoothing factor (0 < alpha < 1)
        self.filtered_force = np.zeros(3)
        self.filtered_torque = np.zeros(6)
####### FANUC M-10iD/16S DH Parameters (in meters)
        self.dh_params = [
                        [0.150, -np.pi/2, 0.630, 0],  # Joint 1
                        [0.850,  0,       0,     0],  # Joint 2
                        [0.145, -np.pi/2, 0,     0],  # Joint 3
                        [0,      np.pi/2, 0.800, 0],  # Joint 4
                        [0,     -np.pi/2, 0,     0],  # Joint 5
                        [0,      0,       0.175, 0]   # Joint 6
                        ]

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

    def compute_cartesian_force(self) -> Optional[np.ndarray]:
        """Compute Cartesian force in tool frame"""
        joint_angles = self._read_joint_angles()
        joint_torques = self._read_torques()
        
        if joint_angles is None or joint_torques is None:
            return None
            
        J = self.geometric_jacobian(joint_angles)
        F_base = np.linalg.pinv(J.T) @ joint_torques
        _, rotation = self.forward_kinematics(joint_angles)
        return rotation.T @ F_base[:3]
    
    def filter_force(self, force: np.ndarray) -> np.ndarray:
        """Apply low-pass filter to the force vector"""
        self.filtered_force = self.alpha * force + (1 - self.alpha) * self.filtered_force
        return self.filtered_force
    
    def filter_torque(self, force: np.ndarray) -> np.ndarray:
        """Apply low-pass filter to the force vector"""
        self.filtered_torque = self.alpha * force + (1 - self.alpha) * self.filtered_torque
        return self.filtered_torque
    # ======================
    # Main Control Loop
    # ======================
    def run(self, update_rate: float = 10.0):
        """Main control loop with plotting"""
        try:
            print("Starting real-time monitoring...")
            plotter = RealTimePlotter()

            # Test mode - uncomment to verify plotting works
            #plotter.start_background_update(None, update_rate)  # Uses test data

            # Production mode - comment above and uncomment below:
            plotter.start_background_update(self, update_rate)  # Uses real robot data

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
    robot.run(update_rate=10.0)