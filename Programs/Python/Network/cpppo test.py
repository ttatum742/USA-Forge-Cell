from cpppo.server.enip import client
from cpppo.server.enip.get_attribute import attribute_operations
import socket
import time
from typing import Tuple, Dict, Any

class FanucConnectionTester:
    def __init__(self, ip_address: str, timeout: float = 5.0):
        self.ip_address = ip_address
        self.timeout = timeout
        self.port = 44818  # Standard EtherNet/IP port

    def test_connection(self) -> Tuple[bool, Dict[str, Any]]:
        """Test connection to FANUC robot using CPPPO"""
        diagnostics = {
            'ip': self.ip_address,
            'port': self.port,
            'timeout': self.timeout,
            'steps': []
        }

        # 1. Test basic network connectivity
        network_ok, network_diag = self._test_network()
        diagnostics['steps'].append(('network', network_ok, network_diag))
        if not network_ok:
            return False, diagnostics

        # 2. Test EtherNet/IP service
        # enip_ok, enip_diag = self._test_enip()
        # diagnostics['steps'].append(('enip_service', enip_ok, enip_diag))
        # if not enip_ok:
        #    return False, diagnostics

        # 3. Test basic tag reading
        tag_ok, tag_diag = self._test_tag_read()
        diagnostics['steps'].append(('tag_read', tag_ok, tag_diag))

        return tag_ok, diagnostics

    def _test_network(self) -> Tuple[bool, Dict[str, Any]]:
        """Check if we can reach the robot at network level"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(self.timeout)
                sock.connect((self.ip_address, self.port))
            return True, {'message': 'Network reachable'}
        except Exception as e:
            return False, {
                'error': str(e),
                'suggestion': 'Check physical connection and IP configuration'
            }

    def _test_enip(self) -> Tuple[bool, Dict[str, Any]]:
        """Test basic EtherNet/IP service availability"""
        try:
            # Using the proper attribute operations for current CPPPO
            with client.connector(host=self.ip_address, port=self.port, timeout=self.timeout) as conn:
                # Request identity object (Class 1, Instance 1)
                operations = attribute_operations(
                    ['@1/1/1',  # Vendor ID 
                     '@1/1/2',  # Device Type 
                     '@1/1/3',]  # Product Code 
                )
                
                for _, desc, attrs, _ in conn.pipeline(operations=operations, depth=2):
                    if attrs:
                        vendor, device_type, product_code = attrs[0].value, attrs[1].value, attrs[2].value
                        return True, {
                            'vendor': f"0x{vendor:04X}",
                            'device_type': f"0x{device_type:04X}",
                            'product_code': f"0x{product_code:04X}"
                        }
                return False, {
                    'error': 'No response to identity query',
                    'suggestion': 'Verify EtherNet/IP is enabled on FANUC controller'
                }
        except Exception as e:
            return False, {
                'error': str(e),
                'suggestion': 'Check FANUC network configuration'
            }

    def _test_tag_read(self) -> Tuple[bool, Dict[str, Any]]:
        """Attempt to read a FANUC numeric register"""
        try:
            with client.connector(host=self.ip_address, port=self.port, timeout=self.timeout) as conn:
                # Try reading R[1]
                operations = attribute_operations(
                    ['@0x6B/0x01/0x01']  # Register Object (0x6B), Instance 1, Attribute 1 (R[1])
                )
                
                for _, desc, attrs, _ in conn.pipeline(operations=operations, depth=2):
                    if attrs:
                        return True, {
                            'tag': 'R[1]',
                            'value': attrs[0].value,
                            'type': type(attrs[0].value).__name__
                        }
                return False, {
                    'error': 'No response to tag read',
                    'suggestion': 'Check if R[1] exists in FANUC program'
                }
        except Exception as e:
            return False, {
                'error': str(e),
                'suggestion': 'Verify tag addressing format'
            }

def print_diagnostics(diagnostics: Dict[str, Any]):
    """Print formatted diagnostics output"""
    print(f"\nFANUC Connection Diagnostics - {diagnostics['ip']}")
    print("=" * 60)
    
    for step, success, details in diagnostics['steps']:
        status = "✓" if success else "✗"
        print(f"{status} {step.replace('_', ' ').title():<15}", end='')
        
        if success:
            if isinstance(details, dict):
                print(" | " + ", ".join(f"{k}:{v}" for k, v in details.items()))
            else:
                print(f" | {details}")
        else:
            print(f" | ERROR: {details.get('error', 'Unknown')}")
            if 'suggestion' in details:
                print(f"   → Suggestion: {details['suggestion']}")

if __name__ == "__main__":
    # Configuration
    ROBOT_IP = "192.168.0.1"  # Replace with your FANUC's IP
    TEST_TIMEOUT = 5.0
    MAX_RETRIES = 3

    print(f"Testing connection to FANUC at {ROBOT_IP}...")

    tester = FanucConnectionTester(ROBOT_IP, TEST_TIMEOUT)
    
    for attempt in range(1, MAX_RETRIES + 1):
        print(f"\nAttempt {attempt}/{MAX_RETRIES}")
        success, diag = tester.test_connection()
        print_diagnostics(diag)
        
        if success:
            print("\nConnection successful!")
            break
            
        if attempt < MAX_RETRIES:
            print(f"\nRetrying in {TEST_TIMEOUT} seconds...")
            time.sleep(TEST_TIMEOUT)
    else:
        print("\nFailed to establish connection after maximum retries")

    # Additional troubleshooting
    if not success:
        print("\nAdvanced Troubleshooting Steps:")
        print("1. On the FANUC teach pendant:")
        print("   - Press MENU > SETUP > HOST COMM")
        print("   - Ensure EtherNet/IP is enabled")
        print("2. Verify the exact register object class:")
        print("   - FANUC may use different class than 0x6B")
        print("3. Try Wireshark to monitor traffic:")
        print("   - Filter: 'cip && ip.addr == {ROBOT_IP}'")
        print("4. Contact FANUC support with:")
        print("   - The exact error messages")
        print("   - Your controller model/software version")