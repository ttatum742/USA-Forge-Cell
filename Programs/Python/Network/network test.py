from pycomm3 import LogixDriver
import time

def test_plc_connection(ip_address):
    try:
        print(f"Attempting to connect to {ip_address}...")
        
        # Add explicit connection parameters
        with LogixDriver(
            ip_address,
            init_tags=True,       # Initialize tag database
            init_info=True,       # Initialize PLC info
            timeout=5,            # 5 second timeout
            init_program_tags=True
        ) as plc:
            
            if plc.connected:
                print("Connection successful!")
                print(f"PLC Name: {plc.get_plc_name()}")
                print(f"Processor Type: {plc.get_plc_properties().get('vendor')}")
                return True
                
    except Exception as e:
        print(f"Connection failed: {str(e)}")
        return False

# Replace with your FANUC's IP
FANUC_IP = '192.168.0.1'  

# Test with retries
for attempt in range(3):
    print(f"\nAttempt {attempt + 1}/3")
    if test_plc_connection(FANUC_IP):
        break
    time.sleep(2)
else:
    print("Failed to establish connection after 3 attempts")