import numpy as np
from pycomm3 import LogixDriver

# ======================
# 1. Robot Kinematics
# ======================
def dh_transform(a, alpha, d, theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    T = np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])
    return T

def forward_kinematics(dh_params, joint_angles):
    T = np.eye(4)
    for i in range(len(joint_angles)):
        a, alpha, d, theta = dh_params[i]
        theta += joint_angles[i]
        Ti = dh_transform(a, alpha, d, theta)
        T = T @ Ti
    return T[:3, 3], T[:3, :3]

def geometric_jacobian(dh_params, joint_angles):
    num_joints = len(joint_angles)
    J = np.zeros((6, num_joints))
    T = np.eye(4)
    joint_positions = []
    joint_axes = []
    
    for i in range(num_joints):
        a, alpha, d, theta = dh_params[i]
        theta += joint_angles[i]
        Ti = dh_transform(a, alpha, d, theta)
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

def joint_torques_to_cartesian_force(dh_params, joint_angles, joint_torques):
    J = geometric_jacobian(dh_params, joint_angles)
    F_base = np.linalg.pinv(J.T) @ joint_torques
    _, rotation = forward_kinematics(dh_params, joint_angles)
    F_tool = rotation.T @ F_base[:3]
    return F_tool

# ======================
# 2. Ethernet/IP Communication
# ======================
def read_joint_data(plc):
    """Read joint angles (degrees) and torques (Nm) from individual group outputs."""
    try:
        # Read joint angles (degrees)
        joint_angles_deg = [
            plc.read('GO[1]')[1],  # Joint 1
            plc.read('GO[2]')[1],  # Joint 2
            plc.read('GO[3]')[1],  # Joint 3
            plc.read('GO[4]')[1],  # Joint 4
            plc.read('GO[5]')[1],  # Joint 5
            plc.read('GO[6]')[1]   # Joint 6
        ]
        
        # Convert to radians
        joint_angles_rad = np.radians(joint_angles_deg)
        
        # Read joint torques (Nm)
        joint_torques = [
            plc.read('GO[7]')[1],   # Torque 1
            plc.read('GO[8]')[1],   # Torque 2
            plc.read('GO[9]')[1],   # Torque 3
            plc.read('GO[10]')[1],  # Torque 4
            plc.read('GO[11]')[1],  # Torque 5
            plc.read('GO[12]')[1]   # Torque 6
        ]
        
        return joint_angles_rad, joint_torques
    except Exception as e:
        print(f"Error reading from robot: {e}")
        return None, None

def write_force_to_robot(plc, force_vector):
    """Write Cartesian force components to individual group inputs."""
    try:
        plc.write(('GI[1]', force_vector[0]))  # Fx
        plc.write(('GI[2]', force_vector[1]))  # Fy
        plc.write(('GI[3]', force_vector[2]))  # Fz
        print(f"Force written to robot - X: {force_vector[0]:.2f}, Y: {force_vector[1]:.2f}, Z: {force_vector[2]:.2f} N")
    except Exception as e:
        print(f"Error writing to robot: {e}")

# ======================
# 3. Main Program
# ======================
def main():
    # FANUC CRX-10iA/L DH parameters (approximate)
    dh_params = [
        [0, np.pi/2, 0.165, 0],       # Joint 1
        [0.260, 0, 0, 0],              # Joint 2
        [0.260, 0, 0, 0],              # Joint 3
        [0, np.pi/2, 0.165, 0],        # Joint 4
        [0, -np.pi/2, 0.140, 0],       # Joint 5
        [0, 0, 0.140, 0]               # Joint 6
    ]

    # Connect to the robot (replace IP with your robot's IP)
    with LogixDriver('192.168.0.1') as plc:  # Example IP
        print("Connected to FANUC robot")
        while True:
            # 1. Read all joint data
            joint_angles_rad, joint_torques = read_joint_data(plc)
            if joint_angles_rad is None or joint_torques is None:
                continue
            
            # 2. Compute Cartesian force
            force_tool = joint_torques_to_cartesian_force(
                dh_params, joint_angles_rad, joint_torques
            )
            
            # 3. Write force back to robot
            write_force_to_robot(plc, force_tool)

if __name__ == "__main__":
    main()