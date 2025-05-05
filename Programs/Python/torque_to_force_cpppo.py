from cpppo.server.enip import client
from cpppo.server.enip.get_attribute import attribute_operations, proxy_simple
import numpy as np
import time
import logging
from typing import Tuple, Dict, List, Optional

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Init variables to store torques and angles
j1_torque = 0.0
j2_torque = 0.0
j3_torque = 0.0
j4_torque = 0.0
j5_torque = 0.0
j6_torque = 0.0

j1_angle = 0.0
j2_angle = 0.0
j3_angle = 0.0
j4_angle = 0.0
j5_angle = 0.0
j6_angle = 0.0

class FanucRobot:
    def __init__(self, ip_address: str, timeout: float = 5.0):
        self.ip_address = ip_address
        self.timeout = timeout
        # self.port = 44818  # Standard EtherNet/IP port
        
        # FANUC CRX-10iA/L DH Parameters
        self.dh_params = [
            [0, np.pi/2, 0.165, 0],       # Joint 1
            [0.260, 0, 0, 0],              # Joint 2
            [0.260, 0, 0, 0],              # Joint 3
            [0, np.pi/2, 0.165, 0],        # Joint 4
            [0, -np.pi/2, 0.140, 0],       # Joint 5
            [0, 0, 0.140, 0]               # Joint 6
        ]

        # Parameter mapping for joint angles and torques
        self.parameters = dict(proxy_simple.PARAMETERS,
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
            j6a=proxy_simple.parameter('@0x6B/1/65', 'DINT', 'Degrees')
        )

        # Torque register mapping (from your working implementation)
        self.torque_registers = {
            'j1t': 'j1_torque',  # Joint 1 torque (R[50])
            'j2t': 'j2_torque',  # Joint 2 torque (R[51])
            'j3t': 'j3_torque',  # Joint 3 torque (R[52])
            'j4t': 'j4_torque',  # Joint 4 torque (R[53])
            'j5t': 'j5_torque',  # Joint 5 torque (R[54])
            'j6t': 'j6_torque'   # Joint 6 torque (R[55])
        }

        # Angle register mapping (from your working implementation)
        self.angle_registers = {
            'j1a': 'j1_angle',  # Joint 1 angle (R[60])
            'j2a': 'j2_angle',  # Joint 2 angle (R[61])
            'j3a': 'j3_angle',  # Joint 3 angle (R[62])
            'j4a': 'j4_angle',  # Joint 4 angle (R[63])
            'j5a': 'j5_angle',  # Joint 5 angle (R[64])
            'j6a': 'j6_angle'   # Joint 6 angle (R[65])
        }

    # ======================
    # 1. EtherNet/IP Communication Methods (Updated)
    # ======================
    def _read_torques(self) -> Optional[List[float]]:
        """Read all joint torques"""
        try:
            with proxy_simple(host=self.ip_address) as proxy:
                torques = []
                for param, var_name in self.torque_registers.items():
                    try:
                        param_string = proxy.parameter_substitution(param)
                        value, = proxy.read(param_string, checking=True)
                        # value, = proxy.read(proxy.parameter_substitution(param), checking=True)
                        globals()[var_name] = float(value[0]) if isinstance(value, list) else float(value)
                        torque = globals()[var_name]  # Access the global variable
                        torques.append(torque)
                        logger.debug(f"Read {param}: {torque:.2f} Nm")
                    except Exception as e:
                        logger.error(f"Failed to read {param}: {str(e)}")
                        return None
                return torques
        except Exception as exc:
            logging.warning("Torque read failed: %s", exc)
            proxy.close_gateway(exc=exc)
            raise
        finally:
            proxy.close_gateway()

    def _read_joint_angles(self) -> Optional[List[float]]:
        """Read joint angles"""
        try:
            with proxy_simple(host=self.ip_address, timeout=self.timeout) as proxy:
                angles = []
                for param, var_name in self.angle_registers.items():
                    try:
                        # Using the exact method from your working implementation
                        value, = proxy.read(proxy.parameter_substitution(param), checking=True)
                        globals()[var_name] = float(value[0]) if isinstance(value, list) else float(value)
                        angle = globals()[var_name]  # Access the global variable
                        angle = np.radians(angle) # Convert to radians
                        angles.append(angle)
                        logger.debug(f"Read {param}: {angle:.2f} radians")
                    except Exception as e:
                        logger.error(f"Failed to read {param}: {str(e)}")
                        return None
                return angles
        except Exception as exc:
            logging.warning("Angle read failed: %s", exc)
            proxy.close_gateway(exc=exc)
            raise
        finally:
            proxy.close_gateway()

    def _write_forces(self, forces: List[float]) -> bool:
        """Write Cartesian forces to robot"""
        # Implement your force writing method here
        return True  # Mock success for demonstration

    # ======================
    # 2. Robot Kinematics (Same as Before)
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

    # ======================
    # 3. Main Operational Methods
    # ======================
    def get_joint_states(self) -> Tuple[Optional[List[float]], Optional[List[float]]]:
        """Read joint angles and torques from robot"""
        # Read torques using verified method
        torques = self._read_torques()
        if torques is None:
            return None, None
            
        # Read joint angles (implement your method here)
        angles = self._read_joint_angles()
        if angles is None:
            return None, None
            
        return angles, torques

    def compute_cartesian_force(self) -> Optional[np.ndarray]:
        """Compute Cartesian force in tool frame"""
        joint_angles, joint_torques = self.get_joint_states()
        if joint_angles is None or joint_torques is None:
            return None
            
        # Convert angles from degrees to radians if needed
        joint_angles_rad = np.radians(joint_angles)
        
        J = self.geometric_jacobian(joint_angles_rad)
        F_base = np.linalg.pinv(J.T) @ joint_torques
        _, rotation = self.forward_kinematics(joint_angles_rad)
        F_tool = rotation.T @ F_base[:3]
        return F_tool

    def send_cartesian_force(self, force: np.ndarray) -> bool:
        """Send Cartesian force to robot"""
        return self._write_forces(force.tolist())

    # ======================
    # 4. Main Control Loop
    # ======================
    def run(self, update_rate: float = 10.0):
        """Main control loop"""
        try:
            logger.info(f"Starting FANUC force control at {update_rate}Hz...")
            while True:
                start_time = time.time()
                
                # 1. Compute Cartesian force
                force = self.compute_cartesian_force()
                if force is None:
                    logger.warning("Failed to compute force - retrying...")
                    time.sleep(1)
                    continue
                
                logger.debug(f"Computed force: {force}")
                
                # 2. Send to robot
                if not self.send_cartesian_force(force):
                    logger.warning("Failed to send force to robot - retrying...")
                
                # 3. Maintain cycle rate
                elapsed = time.time() - start_time
                sleep_time = max(0, (1.0/update_rate) - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            logger.info("\nStopping controller...")
        except Exception as e:
            logger.error(f"Fatal error: {e}")

if __name__ == "__main__":
    # Initialize robot connection
    robot = FanucRobot(ip_address="192.168.0.1", timeout=5.0)
    
    # Start main control loop (10Hz update rate)
    robot.run(update_rate=10.0)