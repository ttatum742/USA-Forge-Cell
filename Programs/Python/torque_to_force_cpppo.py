from cpppo.server.enip import client
from cpppo.server.enip.get_attribute import attribute_operations
import numpy as np
import time
from typing import Tuple, Dict, List, Optional

class FanucRobot:
    def __init__(self, ip_address: str, timeout: float = 5.0):
        self.ip_address = ip_address
        self.timeout = timeout
        self.port = 44818  # Standard EtherNet/IP port
        
        # FANUC CRX-10iA/L DH Parameters
        self.dh_params = [
            [0, np.pi/2, 0.165, 0],       # Joint 1
            [0.260, 0, 0, 0],              # Joint 2
            [0.260, 0, 0, 0],              # Joint 3
            [0, np.pi/2, 0.165, 0],        # Joint 4
            [0, -np.pi/2, 0.140, 0],       # Joint 5
            [0, 0, 0.140, 0]               # Joint 6
        ]

    # ======================
    # 1. EtherNet/IP Communication Methods
    # ======================
    def _read_tags(self, tags: List[Tuple[str, str]]) -> Optional[Dict[str, float]]:
        """Read multiple tags from the robot"""
        try:
            with client.connector(host=self.ip_address, timeout=self.timeout) as conn:
                operations = attribute_operations([
                    (f'@{tag}', dtype) for tag, dtype in tags
                ])
                
                results = {}
                for idx, (tag, dtype) in enumerate(tags):
                    for _, _, attrs, _ in conn.pipeline(operations=operations[idx:idx+1], depth=1):
                        if attrs:
                            results[tag] = attrs[0].value
                return results
            
        except Exception as e:
            print(f"Error reading tags: {e}")
            return None

    def _write_tags(self, tag_values: Dict[str, float]) -> bool:
        """Write multiple tags to the robot"""
        try:
            with client.connector(host=self.ip_address, timeout=self.timeout) as conn:
                for tag, value in tag_values.items():
                    operations = attribute_operations([
                        (f'@{tag}', 'REAL', value)
                    ])
                    for _, _, attrs, _ in conn.pipeline(operations=operations, depth=1):
                        if not attrs or attrs[0].value != value:
                            return False
                return True
        except Exception as e:
            print(f"Error writing tags: {e}")
            return False

    # ======================
    # 2. Robot Kinematics
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
        # FANUC typically stores joints in R[] registers and torques in other registers
        tags = [(f'64/{i+1}', 'REAL') for i in range(6)]  # Joint angles R[1]-R[6]
        tags += [(f'65/{i+1}', 'REAL') for i in range(6)]  # Joint torques (hypothetical address)
        
        results = self._read_tags(tags)
        if not results:
            return None, None
            
        angles = [results[f'64/{i+1}'] for i in range(6)]
        torques = [results[f'65/{i+1}'] for i in range(6)]
        return angles, torques

    def compute_cartesian_force(self) -> Optional[np.ndarray]:
        """Compute Cartesian force in tool frame"""
        joint_angles, joint_torques = self.get_joint_states()
        if joint_angles is None or joint_torques is None:
            return None
            
        # Convert angles from degrees to radians
        joint_angles_rad = np.radians(joint_angles)
        
        J = self.geometric_jacobian(joint_angles_rad)
        F_base = np.linalg.pinv(J.T) @ joint_torques
        _, rotation = self.forward_kinematics(joint_angles_rad)
        F_tool = rotation.T @ F_base[:3]
        return F_tool

    def send_cartesian_force(self, force: np.ndarray) -> bool:
        """Send Cartesian force to robot"""
        # Adjust to whichever registers you end up storing the forces in
        tag_values = {
            '66/1': force[0],  # Fx
            '66/2': force[1],   # Fy
            '66/3': force[2]    # Fz
        }
        return self._write_tags(tag_values)

    # ======================
    # 4. Main Control Loop
    # ======================
    def run(self, update_rate: float = 10.0):
        """Main control loop"""
        try:
            while True:
                start_time = time.time()
                
                # 1. Compute Cartesian force
                force = self.compute_cartesian_force()
                if force is None:
                    print("Failed to compute force")
                    time.sleep(1)
                    continue
                
                # 2. Send to robot
                if not self.send_cartesian_force(force):
                    print("Failed to send force to robot")
                
                # 3. Maintain cycle rate
                elapsed = time.time() - start_time
                sleep_time = max(0, (1.0/update_rate) - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("Stopping controller...")

if __name__ == "__main__":
    # Initialize robot connection
    robot = FanucRobot(ip_address="192.168.0.1", timeout=5.0)
    
    # Start main control loop (10Hz update rate)
    print("Starting FANUC force control...")
    robot.run(update_rate=10.0)