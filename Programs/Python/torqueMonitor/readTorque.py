###########################################################################
# This script reads the torque values from each joint of a FANUC robot... #
# It uses the cpppo library to communicate with the robot via Ethernet/IP #
# Torques are stored as real values in R[50]-R[55] in the robot's memory. #
###########################################################################

from cpppo.server.enip.get_attribute import proxy_simple
import logging
import time

class fanuc_robot(proxy_simple):
    PARAMETERS = dict(proxy_simple.PARAMETERS,
                     j1t=proxy_simple.parameter('@0x6B/1/50', 'DINT', 'Nm'),
                     j2t=proxy_simple.parameter('@0x6B/1/51', 'DINT', 'Nm'),
                     j3t=proxy_simple.parameter('@0x6B/1/52', 'DINT', 'Nm'),
                     j4t=proxy_simple.parameter('@0x6B/1/53', 'DINT', 'Nm'),
                     j5t=proxy_simple.parameter('@0x6B/1/54', 'DINT', 'Nm'),
                     j6t=proxy_simple.parameter('@0x6B/1/55', 'DINT', 'Nm'),
                     )

via = fanuc_robot(host="192.168.0.1")

# Initialize variables to store torque values
j1_torque = 0.0
j2_torque = 0.0
j3_torque = 0.0
j4_torque = 0.0
j5_torque = 0.0
j6_torque = 0.0

try:
    # Read each parameter individually and store in variables
    param_mapping = {
        'j1t': 'j1_torque',
        'j2t': 'j2_torque',
        'j3t': 'j3_torque',
        'j4t': 'j4_torque',
        'j5t': 'j5_torque',
        'j6t': 'j6_torque'
    }
    
    for param, var_name in param_mapping.items():
        try:
            # Read single parameter
            param_string = via.parameter_substitution(param)
            value, = via.read(param_string, checking=True)
            
            # Store value in the corresponding variable
            globals()[var_name] = float(value[0]) if isinstance(value, list) else float(value)
               
            # print(f"{param}: {globals()[var_name]:.2f} Nm")
            
            # Small delay between reads
            time.sleep(0)
            
        except Exception as exc:
            print(f"Failed to read {param}: {str(exc)}")
            continue
            
    # Now you can use the variables directly:
    print("\nFinal Torque Values:")
    print(f"Joint 1: {j1_torque:.2f} Nm")
    print(f"Joint 2: {j2_torque:.2f} Nm")
    print(f"Joint 3: {j3_torque:.2f} Nm")
    print(f"Joint 4: {j4_torque:.2f} Nm")
    print(f"Joint 5: {j5_torque:.2f} Nm")
    print(f"Joint 6: {j6_torque:.2f} Nm")
            
except Exception as exc:
    logging.warning("Main communication failed: %s", exc)
    via.close_gateway(exc=exc)
    raise
finally:
    via.close_gateway()