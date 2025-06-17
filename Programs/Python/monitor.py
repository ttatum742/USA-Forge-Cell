import sys
import time
import serial
import psutil

def is_process_running(process_name):
    """Check if a process with the given name is running."""
    for proc in psutil.process_iter(['name']):
        if proc.info['name'] == process_name:
            return True
    return False

def control_rs232_pin(state):
    """Control the RS232 pin (using RTS or DTR)."""
    try:
        # Configure the serial port
        with serial.Serial('COM1', 9600, timeout=1) as ser:
            # Using RTS pin (can use DTR instead if preferred)
            ser.rts = state
            # Add a small delay to ensure the signal is set
            time.sleep(0.1)
    except serial.SerialException as e:
        print(f"Error accessing serial port: {e}", file=sys.stderr)
        return False
    return True

def main():
    process_name = "torque_derivative.py"
    check_interval = 0.5  # seconds between checks
    
    print(f"Starting monitor for {process_name}...")
    
    try:
        while True:
            # Check if the process is running
            if is_process_running(process_name):
                # Turn on the RS232 pin
                if control_rs232_pin(True):
                    print(f"{process_name} is running - RS232 pin ON")
                else:
                    print("Failed to set RS232 pin")
            else:
                # Turn off the RS232 pin
                if control_rs232_pin(False):
                    print(f"{process_name} not running - RS232 pin OFF")
                else:
                    print("Failed to reset RS232 pin")
            
            # Wait before checking again
            time.sleep(check_interval)
            
    except KeyboardInterrupt:
        # Turn off the pin when program is terminated
        control_rs232_pin(False)
        print("\nProgram terminated. RS232 pin set to OFF.")

if __name__ == "__main__":
    main()