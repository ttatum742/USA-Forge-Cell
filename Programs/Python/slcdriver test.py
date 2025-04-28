from pycomm3 import SLCDriver
import time
from pycomm3.exceptions import CommError
import socket

def test_fanuc_connection(ip_address, port=44818, timeout=5):
    """
    Robust connection test for FANUC robots using SLCDriver
    Returns tuple: (success: bool, diagnostics: dict)
    """
    diagnostics = {
        'ip_address': ip_address,
        'port': port,
        'timeout': timeout,
        'steps': {}
    }

    def add_step(name, result, details=None):
        diagnostics['steps'][name] = {
            'success': result,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'details': details
        }
        return result

    # 1. Basic network reachability test
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            s.connect((ip_address, port))
        add_step('network_reachable', True)
    except (socket.timeout, ConnectionRefusedError) as e:
        return (False, add_step('network_reachable', False, f"Network unreachable: {str(e)}"))

    # 2. SLCDriver connection test
    try:
        start_time = time.time()
        with SLCDriver(ip_address, timeout=timeout) as plc:
            conn_time = time.time() - start_time
            
            if not plc.connected:
                return (False, add_step('slc_connection', False, "Driver connected but reported not connected"))

            # 3. Basic read test
            try:
                # Try reading a common FANUC register
                test_read = plc.read_tag('R[1]')
                add_step('read_test', True, {
                    'value': test_read.value,
                    'type': test_read.type,
                    'connection_time_sec': round(conn_time, 3)
                })
                
                # 4. Get controller info
                try:
                    controller_info = {
                        'type': plc.plc_type,
                        'name': plc.name,
                        'path': plc.path
                    }
                    add_step('controller_info', True, controller_info)
                except Exception as e:
                    add_step('controller_info', False, f"Couldn't get controller info: {str(e)}")

                return (True, diagnostics)

            except CommError as e:
                return (False, add_step('read_test', False, f"Read test failed: {str(e)}"))

    except CommError as e:
        return (False, add_step('slc_connection', False, f"SLCDriver connection failed: {str(e)}"))
    except Exception as e:
        return (False, add_step('slc_connection', False, f"Unexpected error: {str(e)}"))


def format_diagnostics(diagnostics):
    """Format the diagnostics output for easy reading"""
    result = []
   # result.append(f"\nConnection Diagnostics for {diagnostics['ip_address']}")
    # result.append("=" * 50)
    
   # for step, data in diagnostics['steps'].items():
   #    status = "✓" if data['success'] else "✗"
   #    result.append(f"{status} {step.replace('_', ' ').title()}")
   #    result.append(f"    Time: {data['timestamp']}")
   #    if data['details']:
   #        if isinstance(data['details'], dict):
   #            for k, v in data['details'].items():
   #                result.append(f"    {k}: {v}")
   #        else:
   #            result.append(f"    Details: {data['details']}")
   #    result.append("-" * 50)
    
   #  return "\n".join(result)


if __name__ == "__main__":
    # Configuration
    FANUC_IP = '192.168.0.1'  # Replace with your robot's IP
    TEST_TIMEOUT = 5  # seconds
    MAX_RETRIES = 3

    print(f"Starting FANUC connection test to {FANUC_IP}...\n")

    for attempt in range(1, MAX_RETRIES + 1):
        print(f"Attempt {attempt} of {MAX_RETRIES}")
        success, diag = test_fanuc_connection(FANUC_IP, timeout=TEST_TIMEOUT)
        
        print(format_diagnostics(diag))
        
        if success:
            print("\nConnection successful!")
            break
            
        if attempt < MAX_RETRIES:
            print(f"\nRetrying in {TEST_TIMEOUT} seconds...")
            time.sleep(TEST_TIMEOUT)
    else:
        print("\nFailed to establish connection after maximum retries")

    # Additional troubleshooting suggestions
    if not success:
        print("\nTroubleshooting Suggestions:")
        print("- Verify the robot's IP address is correct")
        print("- Check physical Ethernet connection and link lights")
        print("- Confirm Ethernet/IP is enabled on the robot controller")
        print("- Try pinging the robot from your computer")
        print("- Check for firewall/antivirus blocking the connection")
        print("- Consult FANUC documentation for specific port requirements")