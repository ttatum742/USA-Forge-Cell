from cpppo.server.enip.get_attribute import proxy_simple
import numpy as np
import time
import logging
from typing import Tuple, List, Optional

# Configure logging
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

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
        self.alpha = 0.01  # Smoothing factor (0 < alpha < 1)
        self.filtered_force = np.zeros(3)
        
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
        """Write a single parameter value"""
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
            
        # Apply low-pass filter to joint angles and torques
        joint_angles = self.low_pass_filter(np.array(joint_angles))
        joint_torques = self.low_pass_filter(np.array(joint_torques)) 

        # Compute the Jacobian and pseudo-inverse
        J = self.geometric_jacobian(joint_angles)
        F_base = np.linalg.pinv(J.T) @ joint_torques
        _, rotation = self.forward_kinematics(joint_angles)
        return rotation.T @ F_base[:3]
    
    def low_pass_filter(self, data: np.ndarray) -> np.ndarray:
        """Apply low-pass filter to input data"""
        self.filtered = self.alpha * data + (1 - self.alpha) * self.filtered
        return self.filtered
    
###### MAIN LOOP ######
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
                
                # 2. Apply low-pass filter
                # filtered_force = self.low_pass_filter(force) ########################### Uncomment for filtering output data
                filtered_force = force ########################### Uncomment for no filtering output data
                logger.debug(f"Filtered force: {filtered_force}")
                # 3. Send to robot
                if not self._write_forces(filtered_force):
                    logger.warning("Failed to send force to robot - retrying...")
                
                # 4. Maintain cycle rate
                elapsed = time.time() - start_time
                sleep_time = max(0, (1.0/update_rate) - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            logger.info("\nStopping controller...")
        except Exception as e:
            logger.error(f"Fatal error: {e}")


if __name__ == "__main__":
    # Initialize with debug logging
    robot = FanucRobot(ip_address="192.168.0.1")
    
    
    # Start main control loop (10Hz update rate)
    robot.run(update_rate=10.0)