"""
Die Scanner Debug Program - Continuous Edge Detection
This program implements continuous die edge scanning using the Keyence sensor
with real-time robot position tracking and Arduino communication.

Uses keyence_always_on Arduino connection and torque_derivative.py syntax
for Fanuc communications with updated register assignments (R[86-189]).
"""

from cpppo.server.enip.get_attribute import proxy_simple
import serial
import numpy as np
import time
import logging
import threading
# from queue import Queue, Empty  # Removed - no longer using continuous reading
from typing import List, Dict, Optional, Tuple, NamedTuple
from dataclasses import dataclass
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

@dataclass
class ScanPoint:
    """Individual scan measurement point"""
    x: float
    y: float 
    z: float
    height: float
    is_valid: bool
    timestamp: float

@dataclass
class EdgePoint:
    """Die edge detection point"""
    x: float
    y: float
    edge_type: str  # 'drop_off', 'rise', 'stable'
    confidence: float

@dataclass
class DieCenter:
    """Final die center calculation result"""
    center_x: float
    center_y: float
    diameter: float
    avg_height: float
    confidence: float
    edge_points: List[EdgePoint]

class DieScanRobotInterface(proxy_simple):
    """Robot communication interface using torque_derivative.py syntax"""
    
    PARAMETERS = dict(proxy_simple.PARAMETERS,
        # Die Scan Communication Registers (R[86-96])
        ds_command=proxy_simple.parameter('@0x6B/1/86', 'DINT', 'cmd'),
        ds_status=proxy_simple.parameter('@0x6B/1/87', 'DINT', 'status'),
        ds_current_x=proxy_simple.parameter('@0x6C/1/88', 'REAL', 'mm'),
        ds_current_y=proxy_simple.parameter('@0x6C/1/89', 'REAL', 'mm'),
        ds_current_z=proxy_simple.parameter('@0x6C/1/90', 'REAL', 'mm'),
        ds_point_counter=proxy_simple.parameter('@0x6B/1/91', 'DINT', 'count'),
        ds_total_points=proxy_simple.parameter('@0x6B/1/92', 'DINT', 'count'),
        ds_center_x=proxy_simple.parameter('@0x6C/1/93', 'REAL', 'mm'),
        ds_center_y=proxy_simple.parameter('@0x6C/1/94', 'REAL', 'mm'),
        ds_avg_height=proxy_simple.parameter('@0x6C/1/95', 'REAL', 'mm'),
        ds_confidence=proxy_simple.parameter('@0x6B/1/96', 'DINT', 'percent'),
        
        # Die Scan Edge Points (R[161-180])
        ds_edge_x1=proxy_simple.parameter('@0x6C/1/161', 'REAL', 'mm'),
        ds_edge_y1=proxy_simple.parameter('@0x6C/1/162', 'REAL', 'mm'),
        ds_edge_x2=proxy_simple.parameter('@0x6C/1/163', 'REAL', 'mm'),
        ds_edge_y2=proxy_simple.parameter('@0x6C/1/164', 'REAL', 'mm'),
        ds_edge_x3=proxy_simple.parameter('@0x6C/1/165', 'REAL', 'mm'),
        ds_edge_y3=proxy_simple.parameter('@0x6C/1/166', 'REAL', 'mm'),
        ds_edge_x4=proxy_simple.parameter('@0x6C/1/167', 'REAL', 'mm'),
        ds_edge_y4=proxy_simple.parameter('@0x6C/1/168', 'REAL', 'mm'),
        ds_edge_x5=proxy_simple.parameter('@0x6C/1/169', 'REAL', 'mm'),
        ds_edge_y5=proxy_simple.parameter('@0x6C/1/170', 'REAL', 'mm'),
        ds_edge_x6=proxy_simple.parameter('@0x6C/1/171', 'REAL', 'mm'),
        ds_edge_y6=proxy_simple.parameter('@0x6C/1/172', 'REAL', 'mm'),
        ds_edge_x7=proxy_simple.parameter('@0x6C/1/173', 'REAL', 'mm'),
        ds_edge_y7=proxy_simple.parameter('@0x6C/1/174', 'REAL', 'mm'),
        ds_edge_x8=proxy_simple.parameter('@0x6C/1/175', 'REAL', 'mm'),
        ds_edge_y8=proxy_simple.parameter('@0x6C/1/176', 'REAL', 'mm'),
        
        # Next position components (R[182-184]) - robot copies to PR[37]
        ds_next_pos_x=proxy_simple.parameter('@0x6C/1/182', 'REAL', 'mm'),
        ds_next_pos_y=proxy_simple.parameter('@0x6C/1/183', 'REAL', 'mm'),
        ds_next_pos_z=proxy_simple.parameter('@0x6C/1/184', 'REAL', 'mm'),
        
        # Position request (R[185]) - triggers LPOS calculation
        ds_pos_request=proxy_simple.parameter('@0x6B/1/185', 'DINT', 'request'),
        
        # Measurement System (R[140-143])
        ds_measure_result=proxy_simple.parameter('@0x6C/1/140', 'REAL', 'mm'),
        ds_measure_trigger=proxy_simple.parameter('@0x6B/1/141', 'DINT', 'trigger'),
        ds_measure_count=proxy_simple.parameter('@0x6B/1/142', 'DINT', 'count'),
        ds_measure_avg=proxy_simple.parameter('@0x6C/1/143', 'REAL', 'mm'),
        )

    def __init__(self, gateway_ip='192.168.0.1', gateway_port=44818):
        """Initialize robot interface"""
        super().__init__(host=gateway_ip, port=gateway_port)
        self.connected = False
        self.position_lock = threading.Lock()
        
    def connect(self):
        """Connect to robot"""
        try:
            logger.info(f"Connecting to robot at {self.host}:{self.port}")
            self.connected = True
            logger.info("Robot connection established")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to robot: {e}")
            return False
            
    def disconnect(self):
        """Disconnect from robot"""
        try:
            self.connected = False
            logger.info("Robot disconnected")
        except Exception as e:
            logger.error(f"Error disconnecting from robot: {e}")
            
    def _write_parameter(self, parameter_name, value):
        """Write parameter to robot register"""
        try:
            # Format the parameter string like torque_derivative.py
            if isinstance(value, int) or parameter_name.endswith('_status') or parameter_name.endswith('_command'):
                param = '%s = (DINT) %s' % (parameter_name, int(value))
            else:
                param = '%s = (REAL) %s' % (parameter_name, float(value))
            
            # Write using parameter substitution like the working examples
            success, = self.write(
                self.parameter_substitution(param), checking=True)
            
            if success:
                logger.debug(f"Successfully wrote {parameter_name}: {value}")
            return success
        except Exception as e:
            logger.error(f"Failed to write parameter {parameter_name}={value}: {e}")
            raise
            
    def _read_parameter(self, parameter_name):
        """Read parameter from robot register"""
        try:
            # Use parameter substitution like networkRead_cpppo.py
            param_str = self.parameter_substitution(parameter_name)
            result, = self.read(param_str, checking=True)
            # Extract the actual value - handle both list and direct value cases
            return float(result[0]) if isinstance(result, list) else float(result)
        except Exception as e:
            logger.error(f"Failed to read parameter {parameter_name}: {e}")
            raise
            
    def read_robot_position(self):
        """Read current robot position using position request"""
        try:
            with self.position_lock:
                # Request position calculation
                self._write_parameter('ds_pos_request', 1)
                time.sleep(0.01)  # Small delay for calculation
                
                # Read calculated position
                x = self._read_parameter('ds_current_x')
                y = self._read_parameter('ds_current_y')
                z = self._read_parameter('ds_current_z')
                
                return [x, y, z]
        except Exception as e:
            logger.error(f"Failed to read robot position: {e}")
            return [0, 0, 0]
            
    def set_status(self, status_code):
        """Set robot status"""
        try:
            self._write_parameter('ds_status', status_code)
        except Exception as e:
            logger.error(f"Failed to set status {status_code}: {e}")

class KeyenceArduinoInterface:
    """Arduino interface for Keyence sensor using keyence_always_on"""
    
    def __init__(self, port='COM8', baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.connected = False
        # self.height_queue = Queue(maxsize=100)  # Removed - no longer using continuous reading
        # self.read_thread = None  # Removed - no longer using continuous reading
        # self.running = False     # Removed - no longer using continuous reading
        
    def connect(self):
        """Connect to Arduino"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=2)
            time.sleep(2)  # Arduino reset delay
            self.connected = True
            # self.running = True  # Removed - no longer using continuous reading
            
            # Removed continuous reading thread - using synchronous send_command instead
            # self.read_thread = threading.Thread(target=self._continuous_read)
            # self.read_thread.daemon = True
            # self.read_thread.start()
            
            # Clear any initial messages from Arduino
            self.serial_conn.reset_input_buffer()
            time.sleep(0.1)
            
            logger.info(f"Arduino connected on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        try:
            # logger.info("Stopping Arduino reading thread...")  # Removed - no longer using continuous reading
            # self.running = False  # Removed - no longer using continuous reading
            # if self.read_thread and self.read_thread.is_alive():  # Removed - no longer using continuous reading
            #     logger.info("Waiting for Arduino thread to finish...")
            #     self.read_thread.join(timeout=2)
            #     if self.read_thread.is_alive():
            #         logger.warning("Arduino thread did not stop cleanly")
            
            if self.serial_conn and self.serial_conn.is_open:
                logger.info("Closing Arduino serial connection...")
                self.serial_conn.close()
                
            self.connected = False
            logger.info("Arduino disconnected successfully")
        except Exception as e:
            logger.error(f"Error disconnecting Arduino: {e}")
            
    def _continuous_read(self):
        """Continuous reading thread - removed as it conflicts with send_command"""
        # This function was causing Arduino communication issues
        # Arduino communication is now handled synchronously via send_command
        pass
    
    def send_command(self, command: str) -> str:
        """Send command to Arduino and get response"""
        if not self.connected:
            return "ERROR:NOT_CONNECTED"
            
        try:
            self.serial_conn.write(f"{command}\n".encode())
            response = self.serial_conn.readline().decode().strip()
            return response
        except Exception as e:
            logger.error(f"Command '{command}' failed: {e}")
            return f"ERROR:{e}"
    
    def read_height(self) -> float:
        """Read current height measurement"""
        response = self.send_command("READ")
        if response.startswith("HEIGHT:"):
            try:
                return float(response.replace("HEIGHT:", ""))
            except ValueError:
                logger.error(f"Invalid height response: {response}")
                return -999.0
        else:
            logger.error(f"Invalid read response: {response}")
            return -999.0
    
    def calibrate(self, reference_distance: float) -> bool:
        """Perform calibration"""
        # Clear any pending messages
        self.serial_conn.reset_input_buffer()
        time.sleep(0.1)
        
        # Start calibration
        response = self.send_command("CALIBRATE")
        logger.info(f"Calibration command response: {response}")
        
        if "CALIBRATION:STARTING" not in response:
            logger.error(f"Failed to start calibration: {response}")
            return False
            
        time.sleep(0.5)
        
        # Send reference distance
        response = self.send_command(f"CAL_DISTANCE {reference_distance}")
        logger.info(f"Calibration distance command response: {response}")
        
        # Wait for completion by checking status repeatedly
        for i in range(30):  # 30 second timeout
            time.sleep(1)
            # Check calibration status by reading current response
            status_response = self.send_command("STATUS")
            logger.debug(f"Calibration status check {i+1}: {status_response}")
            
            if "CALIBRATION:COMPLETE" in status_response:
                logger.info(f"Calibration completed with reference {reference_distance}mm")
                return True
            elif "CALIBRATION:ERROR" in status_response:
                logger.error(f"Calibration failed: {status_response}")
                return False
                
        logger.error("Calibration timeout")
        return False

class ContinuousDieScanner:
    """Main die scanning coordinator"""
    
    def __init__(self):
        self.robot = DieScanRobotInterface()
        self.arduino = KeyenceArduinoInterface()
        self.scan_points = []
        self.edge_points = []
        self.scanning_active = False
        self.scan_start_time = 0
        
        # Enhanced edge detection parameters
        self.base_height_threshold = 2.0  # mm base threshold for edge detection
        self.adaptive_threshold_factor = 1.5  # Multiplier for local variance
        self.edge_confidence_threshold = 0.7
        self.expected_die_diameter = 114.3  # 4.5 inches in mm
        self.die_tolerance = 10.0  # mm tolerance for die diameter
        
        # Multi-point edge confirmation
        self.edge_confirmation_points = 3  # Number of consecutive points to confirm edge
        self.recent_heights = []  # Rolling buffer for local variance calculation
        self.max_height_buffer = 10  # Maximum points to keep for variance calculation
        self.pending_edges = []  # Edges awaiting confirmation
        
        # Edge quality scoring
        self.min_edge_quality = 0.5  # Minimum quality score to accept edge
        
        # Interior edge filtering parameters - 4.5" diameter die = 114.3mm diameter = 57.15mm radius
        self.expected_die_radius = 57.15  # mm - 4.5" die radius
        self.min_outer_radius = 45.0  # mm - minimum radius for outer edge acceptance (die edge ~57mm)
        self.max_outer_radius = 70.0  # mm - maximum radius for outer edge acceptance 
        self.interior_exclusion_radius = 35.0  # mm - reject edges closer than this to center
        self.min_perimeter_edges = 3  # Minimum outer edges required for center calculation (reduced for testing)
        self.min_angular_coverage = 180.0  # Degrees - minimum perimeter coverage required (reduced for testing)
        
        # Edge classification tracking
        self.outer_edges = []  # Confirmed outer perimeter edges only
        self.interior_edges = []  # Detected interior edges (for debugging)
        self.filtered_edge_count = 0  # Track how many edges were filtered out
        
        # Background position tracking
        self.position_thread = None
        
    def connect_systems(self):
        """Connect to all systems"""
        logger.info("Connecting to systems...")
        
        # Connect Arduino first
        if not self.arduino.connect():
            logger.error("Failed to connect to Arduino")
            return False
            
        # Connect robot
        if not self.robot.connect():
            logger.error("Failed to connect to robot")
            return False
            
        logger.info("All systems connected")
        return True
        
    def disconnect_systems(self):
        """Disconnect all systems"""
        logger.info("Disconnecting systems...")
        try:
            self.arduino.disconnect()
            logger.info("Arduino disconnected")
        except Exception as e:
            logger.error(f"Error disconnecting Arduino: {e}")
        
        try:
            self.robot.disconnect()
            logger.info("Robot disconnected")
        except Exception as e:
            logger.error(f"Error disconnecting robot: {e}")
        
        logger.info("All systems disconnected")
        
    def calibrate_sensor(self):
        """Calibrate sensor at current position"""
        logger.info("Starting sensor calibration...")
        
        # Read current position to verify robot is at calibration position
        current_pos = self.robot.read_robot_position()
        logger.info(f"Current robot position: X={current_pos[0]:.1f}, Y={current_pos[1]:.1f}, Z={current_pos[2]:.1f}")
        
        # Wait for robot to settle
        time.sleep(2.0)
        
        # Zero the sensor with proper reference distance
        if not self.arduino.calibrate(reference_distance=65.0):
            logger.error("Arduino calibration failed")
            return False
        
        time.sleep(2.0)
        
        # Take test readings to verify calibration
        test_readings = []
        for i in range(10):  # More readings for better validation
            height = self.arduino.read_height()
            if height > -999:
                test_readings.append(height)
            time.sleep(0.1)
        
        if len(test_readings) >= 5:
            avg_height = np.mean(test_readings)
            std_height = np.std(test_readings)
            logger.info(f"Calibration complete. Average height: {avg_height:.2f}mm, Std: {std_height:.2f}mm")
            
            # Check if measurements are within acceptable range (should be close to 65mm)
            if abs(avg_height - 65.0) < 0.5:  # Within 0.5mm of reference
                logger.info("Calibration successful - within 0.5mm of reference")
                return True
            else:
                logger.error(f"Calibration failed - average {avg_height:.2f}mm too far from reference 65.0mm")
                return False
        else:
            logger.error("Calibration failed - insufficient valid readings")
            return False
            
    def continuous_scan_worker(self, duration=30.0):
        """Worker function for continuous scanning"""
        logger.info(f"Starting continuous scan for {duration} seconds")
        
        try:
            # Calculate 5-pass scan parameters based on new requirements
            logger.info("Reading robot position for scan parameters")
            cal_pos = self.robot.read_robot_position()
            cal_x, cal_y, cal_z = cal_pos[0], cal_pos[1], cal_pos[2]
            logger.info(f"Calibration position: X={cal_x:.1f}, Y={cal_y:.1f}, Z={cal_z:.1f}")
        except Exception as e:
            logger.error(f"Failed to read robot position: {e}")
            return
        
        # Scanning parameters
        y_scan_distance = 120.0  # 5 inches in mm
        scan_step = 5  # mm - faster initial scanning, refined later
        
        # NEW SCAN PATTERN: After calibration, move +5" in Y, +5" in X
        # This becomes the starting position for Pass 1
        start_offset_x = 125.0  # X from calibration
        start_offset_y = 110.0  # Y from calibration
        
        # X movements between passes
        x_move_small = 6.35   # 0.25" in mm
        x_move_large = 57.15  # 2.25" in mm
        
        # Initial scan start position (after calibration move)
        scan_start_x = cal_x + start_offset_x
        scan_start_y = cal_y + start_offset_y
        scan_start_z = cal_z
        
        logger.info("=== NEW 5-Pass Scanning Pattern ===")
        logger.info(f"  Calibration position: X={cal_x:.1f}, Y={cal_y:.1f}, Z={cal_z:.1f}")
        logger.info(f"  Start position: X={scan_start_x:.1f}, Y={scan_start_y:.1f} (+5\" X, +5\" Y from cal)")
        logger.info(f"  Pass 1: Y={scan_start_y:.1f} to Y={scan_start_y - y_scan_distance:.1f} (-Y), then X-0.25")
        logger.info(f"  Pass 2: Y={scan_start_y - y_scan_distance:.1f} to Y={scan_start_y:.1f} (+Y), then X-0.25")
        logger.info(f"  Pass 3: Y={scan_start_y:.1f} to Y={scan_start_y - y_scan_distance:.1f} (-Y), then X-2.25")
        logger.info(f"  Pass 4: Y={scan_start_y - y_scan_distance:.1f} to Y={scan_start_y:.1f} (+Y), then X-0.25")
        logger.info(f"  Pass 5: Y={scan_start_y:.1f} to Y={scan_start_y - y_scan_distance:.1f} (-Y)")
        
        # First move to scan start position
        try:
            logger.info(f"Moving to scan start position: X={scan_start_x:.1f}, Y={scan_start_y:.1f}, Z={scan_start_z:.1f}")
            self.robot._write_parameter('ds_next_pos_x', scan_start_x)
            self.robot._write_parameter('ds_next_pos_y', scan_start_y)
            self.robot._write_parameter('ds_next_pos_z', scan_start_z)
            self.robot._write_parameter('ds_status', 11)  # Signal robot to move
            
            # Wait for robot to complete initial move
            move_timeout = 10.0
            move_start = time.time()
            while time.time() - move_start < move_timeout:
                status = self.robot._read_parameter('ds_status')
                if status == 10:  # Robot ready again
                    break
                time.sleep(0.1)
            else:
                logger.error("Robot timeout on initial move to scan start position")
                return
                
            logger.info("Robot at scan start position")
        except Exception as e:
            logger.error(f"Failed to move to scan start position: {e}")
            return
            
        # Execute 5-pass scanning pattern
        try:
            logger.info("Starting 5-pass scan execution")
            scan_count = self.perform_new_5_pass_scan(
                scan_start_x, scan_start_y, scan_start_z, 
                y_scan_distance, scan_step, x_move_small, x_move_large)
            
            logger.info(f"Continuous scan complete: {scan_count} points collected, {len(self.edge_points)} edges detected")
        except Exception as e:
            logger.error(f"Failed during 5-pass scan: {e}")
            return
        
        # Continue to center calculation - do not set final status yet
        # Final status (ds_status=2 for success or ds_status=3 for error) will be set after center calculation
        logger.info("Scanning phase complete, proceeding to center calculation")
        
        # Note: Do not set scanning_active = False here, as we still need to calculate center
    
    def perform_new_5_pass_scan(self, start_x, start_y, start_z, y_scan_distance, scan_step, x_move_small, x_move_large):
        """Execute NEW 5-pass scanning pattern per requirements"""
        scan_count = 0
        last_height = None
        
        # Initialize current position
        current_x = start_x
        current_y = start_y
        current_z = start_z
        
        # PASS 1: 5" move in -Y direction
        logger.info("Pass 1: Starting 5 inch scan in -Y direction")
        logger.info(f"Pass 1 start position: X={current_x:.1f}, Y={current_y:.1f}, Z={current_z:.1f}")
        try:
            scan_count, last_height = self._execute_scan_pass(
                current_x, current_y, current_z, y_scan_distance, scan_step, 
                direction="-Y", pass_num=1, scan_count=scan_count, last_height=last_height)
            logger.info(f"Pass 1 completed: {scan_count} points so far")
        except Exception as e:
            logger.error(f"Pass 1 failed: {e}")
            return 0
        
        # Move -0.25" in X for Pass 2
        current_x -= x_move_small
        logger.info(f"Moving -0.25 inch in X to position: X={current_x:.1f}")
        
        # PASS 2: 5" move in +Y direction
        logger.info("Pass 2: Starting 5 inch scan in +Y direction")
        current_y = start_y - y_scan_distance  # Start at the end of previous pass
        scan_count, last_height = self._execute_scan_pass(
            current_x, current_y, current_z, y_scan_distance, scan_step, 
            direction="+Y", pass_num=2, scan_count=scan_count, last_height=last_height)
        
        # Move -0.25" in X for Pass 3
        current_x -= x_move_small
        logger.info(f"Moving -0.25 inch in X to position: X={current_x:.1f}")
        
        # PASS 3: 5" move in -Y direction
        logger.info("Pass 3: Starting 5 inch scan in -Y direction")
        current_y = start_y  # Start at the beginning again
        scan_count, last_height = self._execute_scan_pass(
            current_x, current_y, current_z, y_scan_distance, scan_step, 
            direction="-Y", pass_num=3, scan_count=scan_count, last_height=last_height)
        
        # Move -2.25" in X for Pass 4
        current_x -= x_move_large
        logger.info(f"Moving -2.25 inches in X to position: X={current_x:.1f}")
        
        # PASS 4: 5" move in +Y direction
        logger.info("Pass 4: Starting 5 inch scan in +Y direction")
        current_y = start_y - y_scan_distance  # Start at the end
        scan_count, last_height = self._execute_scan_pass(
            current_x, current_y, current_z, y_scan_distance, scan_step, 
            direction="+Y", pass_num=4, scan_count=scan_count, last_height=last_height)
        
        # Move -0.25" in X for Pass 5
        current_x -= x_move_small
        logger.info(f"Moving -0.25 inch in X to position: X={current_x:.1f}")
        
        # PASS 5: 5" move in -Y direction
        logger.info("Pass 5: Starting 5 inch scan in -Y direction")
        current_y = start_y  # Start at the beginning
        scan_count, last_height = self._execute_scan_pass(
            current_x, current_y, current_z, y_scan_distance, scan_step, 
            direction="-Y", pass_num=5, scan_count=scan_count, last_height=last_height)
        
        # Add cross-direction (X) scanning for edge validation
        logger.info("Starting cross-direction (X) scanning for edge validation")
        x_scan_count = self.perform_cross_direction_scan(start_x, start_y, start_z, y_scan_distance, scan_step)
        
        # Add radial validation scanning from estimated center
        logger.info("Starting radial validation scanning")
        radial_scan_count = self.perform_radial_validation_scan(start_x, start_y, start_z, scan_step)
        
        total_scan_count = scan_count + x_scan_count + radial_scan_count
        
        # Log completion with invalid reading statistics
        self._log_scan_completion_stats(total_scan_count)
        return total_scan_count
    
    def perform_cross_direction_scan(self, original_start_x, original_start_y, original_start_z, y_scan_distance, scan_step):
        """Perform FOCUSED X-direction scanning passes around preliminary center"""
        x_scan_count = 0
        last_height = None
        
        # Get preliminary center from 5-pass results
        prelim_center_x, prelim_center_y = self.get_preliminary_center_from_5_pass()
        
        # ADAPTIVE scanning parameters based on detected edges from 5-pass
        focused_scan_distance = 50.0  # 50mm total scan length
        x_step_size = 0.5  # 0.5mm steps for refined edge scanning
        
        # Calculate optimal scanning position based on detected outer edges
        if len(self.outer_edges) > 0:
            # Use actual detected edge distances to position X-direction scans
            edge_distances = [np.sqrt((edge.x - prelim_center_x)**2 + (edge.y - prelim_center_y)**2) 
                             for edge in self.outer_edges]
            avg_edge_radius = np.mean(edge_distances)
            max_edge_radius = max(edge_distances)
            
            # INWARD STRATEGY: Start just outside average detected edge radius, not max
            # This avoids peripheral equipment while ensuring we cross the actual die edge
            edge_crossing_offset = avg_edge_radius + 10.0  # Start 10mm outside average edge
            logger.info(f"INWARD X-scan positioning: avg edge radius={avg_edge_radius:.1f}mm, max={max_edge_radius:.1f}mm")
            logger.info(f"X-scan will start at {edge_crossing_offset:.1f}mm from center and scan {focused_scan_distance:.1f}mm INWARD toward center")
        else:
            # Fallback to default positioning if no edges detected yet
            edge_crossing_offset = 57.0  # mm - conservative fallback
            logger.warning("No outer edges detected yet, using default X-scan positioning")
        
        # Position passes around preliminary center (not original start)
        y_positions = [
            prelim_center_y - 15.0,  # 15mm below center
            prelim_center_y,         # At center
            prelim_center_y + 15.0,  # 15mm above center
        ]
        
        logger.info(f"Focused X-direction scanning around preliminary center ({prelim_center_x:.1f}, {prelim_center_y:.1f})")
        logger.info(f"Scan parameters: {focused_scan_distance:.1f}mm range, {x_step_size:.1f}mm steps")
        logger.info(f"Edge crossing strategy: start {edge_crossing_offset:.1f}mm from center, scan inward to cross die edge")
        
        for i, y_pos in enumerate(y_positions):
            logger.info(f"X-direction pass {i+1}/3 at Y={y_pos:.1f}")
            
            try:
                # Execute X-direction scan pass using focused parameters
                pass_count, last_height = self._execute_x_scan_pass(
                    prelim_center_x, y_pos, original_start_z,
                    focused_scan_distance, x_step_size, f"X-pass-{i+1}", x_scan_count, last_height, edge_crossing_offset)
                
                x_scan_count += pass_count
                logger.info(f"X-direction pass {i+1} completed: {pass_count} points")
                
            except Exception as e:
                logger.error(f"X-direction pass {i+1} failed: {e}")
        
        logger.info(f"Cross-direction scanning complete: {x_scan_count} additional points")
        return x_scan_count
    
    def _execute_x_scan_pass(self, center_x, y_position, start_z, x_distance, step_size, pass_name, scan_count, last_height, edge_crossing_offset=45.0):
        """Execute a single X-direction scan pass positioned to cross die edges"""
        x_steps = int(x_distance / step_size)
        current_y = y_position
        current_z = start_z
        
        # Position scan to cross die edges: start at center + edge_crossing_offset, scan inward 25mm
        # This ensures we cross the ~57mm radius die edge
        scan_start_x = center_x + edge_crossing_offset  # Start outside the die edge
        logger.info(f"X-scan positioned to cross die edge: start at X={scan_start_x:.1f} (center+{edge_crossing_offset:.1f}), scan {x_distance:.1f}mm inward")
        
        for step in range(x_steps + 1):
            if not self.scanning_active:
                break
                
            try:
                # Calculate current X position - scan inward toward center
                current_x = scan_start_x - (step * step_size)  # Subtract to scan inward
                
                # Move robot to position using existing protocol
                self.robot._write_parameter('ds_next_pos_x', current_x)
                self.robot._write_parameter('ds_next_pos_y', current_y) 
                self.robot._write_parameter('ds_next_pos_z', current_z)
                self.robot._write_parameter('ds_status', 11)  # Signal move request
                
                # Wait for move completion
                move_timeout = 10.0
                move_start = time.time()
                while time.time() - move_start < move_timeout:
                    status = self.robot._read_parameter('ds_status')
                    if status == 10:  # Move complete
                        break
                    time.sleep(0.05)
                else:
                    logger.warning(f"Move timeout in {pass_name} step {step}")
                    continue
                
                # Take measurement
                height = self.arduino.read_height()
                
                if height > -999:
                    scan_point = ScanPoint(
                        x=current_x, y=current_y, z=current_z,
                        height=height,
                        is_valid=True,
                        timestamp=time.time()
                    )
                    self.scan_points.append(scan_point)
                    scan_count += 1
                    
                    # Edge detection
                    last_height = self._detect_edges(current_x, current_y, height, last_height)
                
                # Log progress periodically
                if step % 20 == 0:
                    logger.debug(f"{pass_name} step {step}/{x_steps}: X={current_x:.1f}, height={height:.1f}mm")
                    
            except Exception as e:
                logger.error(f"Error in {pass_name} step {step}: {e}")
                continue
        
        return scan_count, last_height
    
    def perform_radial_validation_scan(self, reference_x, reference_y, reference_z, scan_step):
        """Perform radial spoke scanning in OUTER EDGE area to validate perimeter"""
        radial_scan_count = 0
        last_height = None
        
        # Get current center estimate for radial scanning
        estimated_center_x, estimated_center_y = self._estimate_dynamic_center()
        
        # Define 6 radial directions (every 60 degrees) - FOCUSED on outer edge area
        radial_angles = [0, 60, 120, 180, 240, 300]  # degrees
        
        # INWARD STRATEGY: Start just outside detected edges and scan inward toward center
        if len(self.outer_edges) > 0:
            # Use actual detected edge distances to position radial scans
            edge_distances = [np.sqrt((edge.x - estimated_center_x)**2 + (edge.y - estimated_center_y)**2) 
                             for edge in self.outer_edges]
            avg_edge_radius = np.mean(edge_distances)
            
            # Start just outside average edge radius and scan inward
            radial_start_radius = avg_edge_radius + 10.0  # Start 10mm outside average edge
            radial_scan_length = 25.0  # Scan 25mm inward toward center
            logger.info(f"Radial INWARD scanning: avg edge radius={avg_edge_radius:.1f}mm, starting {radial_start_radius:.1f}mm from center")
        else:
            # Fallback if no edges detected
            radial_start_radius = 60.0  # mm - conservative fallback
            radial_scan_length = 25.0   # mm - scan inward
            logger.warning("No outer edges for radial positioning, using fallback")
            
        radial_step_size = 0.5  # mm - refined steps for edge detection
        
        logger.info(f"Radial INWARD scanning from center: ({estimated_center_x:.1f}, {estimated_center_y:.1f})")
        logger.info(f"Radial scan parameters: START at {radial_start_radius:.1f}mm, scan {radial_scan_length:.1f}mm INWARD toward center")
        
        for i, angle in enumerate(radial_angles):
            logger.info(f"Radial spoke {i+1}/6 at {angle}° in outer edge area")
            
            try:
                # Execute radial spoke scan in outer edge area
                spoke_count, last_height = self._execute_radial_spoke(
                    estimated_center_x, estimated_center_y, reference_z,
                    angle, radial_scan_length, radial_step_size, f"spoke-{angle}°", last_height, radial_start_radius)
                
                radial_scan_count += spoke_count
                logger.info(f"Radial spoke {angle}° completed: {spoke_count} points")
                
            except Exception as e:
                logger.error(f"Radial spoke {angle}° failed: {e}")
        
        logger.info(f"Radial validation scanning complete: {radial_scan_count} additional points")
        return radial_scan_count
    
    def _execute_radial_spoke(self, center_x, center_y, center_z, angle_deg, scan_length, step_size, spoke_name, last_height, start_radius=40.0):
        """Execute a single radial spoke scan INWARD from detected edge area toward center"""
        angle_rad = np.radians(angle_deg)
        radius_steps = int(scan_length / step_size)
        scan_count = 0
        
        end_radius = start_radius - scan_length  # End closer to center
        logger.info(f"Radial spoke {angle_deg}°: scanning INWARD from {start_radius:.1f}mm to {end_radius:.1f}mm radius")
        
        for step in range(radius_steps + 1):
            if not self.scanning_active:
                break
                
            try:
                # Calculate current radius and position - SCAN INWARD toward center
                current_radius = start_radius - (step * step_size)  # Subtract to go inward
                
                # Calculate X,Y position along radial line
                current_x = center_x + current_radius * np.cos(angle_rad)
                current_y = center_y + current_radius * np.sin(angle_rad)
                
                # Move robot to position using existing protocol
                self.robot._write_parameter('ds_next_pos_x', current_x)
                self.robot._write_parameter('ds_next_pos_y', current_y) 
                self.robot._write_parameter('ds_next_pos_z', center_z)
                self.robot._write_parameter('ds_status', 11)  # Signal move request
                
                # Wait for move completion
                move_timeout = 10.0
                move_start = time.time()
                while time.time() - move_start < move_timeout:
                    status = self.robot._read_parameter('ds_status')
                    if status == 10:  # Move complete
                        break
                    time.sleep(0.05)
                else:
                    logger.warning(f"Move timeout in {spoke_name} step {step}")
                    continue
                
                # Take measurement
                height = self.arduino.read_height()
                
                if height > -999:
                    scan_point = ScanPoint(
                        x=current_x, y=current_y, z=center_z,
                        height=height,
                        is_valid=True,
                        timestamp=time.time()
                    )
                    self.scan_points.append(scan_point)
                    scan_count += 1
                    
                    # Edge detection
                    last_height = self._detect_edges(current_x, current_y, height, last_height)
                
                # Log progress periodically
                if step % 10 == 0:
                    logger.debug(f"{spoke_name} step {step}: r={current_radius:.1f}mm, height={height:.1f}mm")
                    
            except Exception as e:
                logger.error(f"Error in {spoke_name} step {step}: {e}")
                continue
        
        return scan_count, last_height

    def _execute_scan_pass(self, start_x, start_y, start_z, y_distance, step_size, direction, pass_num, scan_count, last_height):
        """Execute a single scan pass"""
        y_steps = int(y_distance / step_size)
        current_x = start_x
        current_z = start_z
        
        for step in range(y_steps + 1):
            if not self.scanning_active:
                break
                
            try:
                if direction == "+Y":
                    current_y = start_y + (step * step_size)
                else:  # -Y direction
                    current_y = start_y - (step * step_size)
                
                # Move to next scan point
                self.robot._write_parameter('ds_next_pos_x', current_x)
                self.robot._write_parameter('ds_next_pos_y', current_y) 
                self.robot._write_parameter('ds_next_pos_z', current_z)
                self.robot._write_parameter('ds_status', 11)
                
                # Wait for robot to complete move
                move_timeout = 5.0
                move_start = time.time()
                robot_responded = False
                while time.time() - move_start < move_timeout:
                    status = self.robot._read_parameter('ds_status')
                    if status == 10:
                        robot_responded = True
                        break
                    time.sleep(0.01)
                
                if not robot_responded:
                    logger.error(f"Robot timeout on pass {pass_num} step {step}: no response after {move_timeout}s")
                    self.scanning_active = False
                    break
                
                # Take sensor reading
                height = self.arduino.read_height()
                
                if height > -999:
                    scan_point = ScanPoint(
                        x=current_x, y=current_y, z=current_z,
                        height=height,
                        is_valid=True,
                        timestamp=time.time()
                    )
                    self.scan_points.append(scan_point)
                    scan_count += 1
                    
                    # Edge detection
                    last_height = self._detect_edges(current_x, current_y, height, last_height)
                    
                    # Log progress
                    if step % 10 == 0:
                        logger.info(f"Pass {pass_num} step {step}/{y_steps}: Y={current_y:.1f}, height={height:.1f}mm")
                        
            except Exception as e:
                logger.error(f"Pass {pass_num} scanning error: {e}")
                time.sleep(0.1)
        
        logger.info(f"Pass {pass_num} complete. Points: {scan_count}, Edges: {len(self.edge_points)}")
        return scan_count, last_height
        
    def _move_to_position(self, target_x, target_y, target_z, description):
        """Move robot to specified position"""
        try:
            self.robot._write_parameter('ds_next_pos_x', target_x)
            self.robot._write_parameter('ds_next_pos_y', target_y) 
            self.robot._write_parameter('ds_next_pos_z', target_z)
            self.robot._write_parameter('ds_status', 11)
            logger.info(f"Moving to {description}: ({target_x:.1f}, {target_y:.1f})")
            
            # Wait for move completion
            move_timeout = 10.0
            move_start = time.time()
            while time.time() - move_start < move_timeout:
                status = self.robot._read_parameter('ds_status')
                if status == 10:
                    break
                time.sleep(0.1)
        except Exception as e:
            logger.error(f"Failed to move to {description}: {e}")
            
    def _move_with_measurements(self, target_x, target_y, target_z, scan_count, last_height, scan_step, description):
        """Move to position while taking measurements"""
        # Get current position  
        current_pos = self.robot.read_robot_position()
        start_x, start_y = current_pos[0], current_pos[1]
        
        # Calculate steps for movement
        x_steps = max(1, int(abs(target_x - start_x) / scan_step))
        y_steps = max(1, int(abs(target_y - start_y) / scan_step)) 
        total_steps = max(x_steps, y_steps)
        
        logger.info(f"Moving to {description} with measurements: {total_steps} steps")
        
        try:
            for step in range(total_steps + 1):
                if not self.scanning_active:
                    break
                
                # Calculate intermediate position
                progress = step / max(1, total_steps)
                move_x = start_x + (target_x - start_x) * progress
                move_y = start_y + (target_y - start_y) * progress
                
                # Move and measure
                self.robot._write_parameter('ds_next_pos_x', move_x)
                self.robot._write_parameter('ds_next_pos_y', move_y) 
                self.robot._write_parameter('ds_next_pos_z', target_z)
                self.robot._write_parameter('ds_status', 11)
                
                # Wait for move
                move_timeout = 5.0
                move_start = time.time()
                while time.time() - move_start < move_timeout:
                    status = self.robot._read_parameter('ds_status')
                    if status == 10:
                        break
                    time.sleep(0.01)
                
                # Take measurement
                height = self.arduino.read_height()
                if height > -999:
                    scan_point = ScanPoint(
                        x=move_x, y=move_y, z=target_z,
                        height=height, is_valid=True, timestamp=time.time()
                    )
                    self.scan_points.append(scan_point)
                    scan_count += 1
                    last_height = self._detect_edges(move_x, move_y, height, last_height)
                    
        except Exception as e:
            logger.error(f"Error during move with measurements to {description}: {e}")
            
        return scan_count, last_height
        
    def _detect_edges(self, x, y, height, last_height):
        """Enhanced edge detection with adaptive thresholds and quality scoring"""
        # Validate that current height is valid (not -999)
        if height <= -999:
            logger.debug(f"Skipping edge detection - invalid current height: {height}")
            return None  # Return None to indicate invalid reading
        
        # Update rolling height buffer for variance calculation
        self.recent_heights.append(height)
        if len(self.recent_heights) > self.max_height_buffer:
            self.recent_heights.pop(0)
        
        # Only proceed with edge detection if we have a valid previous height
        if last_height is not None and last_height > -999:
            # Calculate adaptive threshold based on local height variance
            adaptive_threshold = self._calculate_adaptive_threshold()
            
            # Basic height difference
            height_diff = height - last_height
            
            # Gradient-based edge detection (requires sufficient history)
            gradient_strength = self._calculate_gradient_strength()
            
            # Check for significant height change
            if abs(height_diff) > adaptive_threshold:
                # Calculate edge quality score
                edge_quality = self._calculate_edge_quality(x, y, height, last_height, height_diff, gradient_strength)
                
                # Only accept high-quality edges
                if edge_quality >= self.min_edge_quality:
                    # Determine edge type
                    edge_type = 'drop_off' if height_diff < 0 else 'rise'
                    
                    # Create edge point with enhanced confidence calculation
                    confidence = min(edge_quality * (abs(height_diff) / adaptive_threshold), 1.0)
                    
                    edge_point = EdgePoint(
                        x=x, y=y, edge_type=edge_type, confidence=confidence
                    )
                    
                    # Multi-point edge confirmation - classify later in filter_interior_edges()
                    if self._confirm_edge_point(edge_point):
                        # Just add to edge_points list - classification happens later
                        self.edge_points.append(edge_point)
                        logger.info(f"EDGE DETECTED ({edge_type}): ({x:.1f}, {y:.1f}) - {height_diff:.1f}mm, quality={edge_quality:.2f}")
                        
                        # Note: Classification as interior/exterior happens later in filter_interior_edges()
                    else:
                        logger.debug(f"Edge rejected by confirmation: ({x:.1f}, {y:.1f}) - {height_diff:.1f}mm")
                else:
                    logger.debug(f"Edge rejected by quality: ({x:.1f}, {y:.1f}) - quality={edge_quality:.2f} < {self.min_edge_quality}")
            
        elif last_height is not None and last_height <= -999:
            logger.debug(f"Skipping edge detection - invalid previous height: {last_height}")
        
        return height  # Return current height as new last_height

    def _calculate_adaptive_threshold(self):
        """Calculate adaptive threshold based on local height variance"""
        if len(self.recent_heights) < 3:
            return self.base_height_threshold  # Use base threshold if insufficient data
        
        # Calculate local variance
        heights_array = np.array(self.recent_heights)
        local_variance = np.var(heights_array)
        local_std = np.sqrt(local_variance)
        
        # Adaptive threshold: base + factor * standard deviation
        adaptive_threshold = self.base_height_threshold + (self.adaptive_threshold_factor * local_std)
        
        # Ensure threshold doesn't get too small or too large
        min_threshold = self.base_height_threshold * 0.5
        max_threshold = self.base_height_threshold * 3.0
        adaptive_threshold = max(min_threshold, min(max_threshold, adaptive_threshold))
        
        return adaptive_threshold
    
    def _calculate_gradient_strength(self):
        """Calculate gradient strength over recent height measurements"""
        if len(self.recent_heights) < 3:
            return 0.0
        
        # Calculate gradient using finite differences
        heights = np.array(self.recent_heights[-3:])  # Use last 3 points
        gradient = np.gradient(heights)
        
        # Return the strength of the gradient (magnitude)
        return np.mean(np.abs(gradient))
    
    def _calculate_edge_quality(self, x, y, height, last_height, height_diff, gradient_strength):
        """Calculate edge quality score based on multiple factors"""
        quality = 0.0
        
        # Factor 1: Height difference magnitude (normalized)
        height_factor = min(abs(height_diff) / (self.base_height_threshold * 2), 1.0)
        quality += height_factor * 0.4  # 40% weight
        
        # Factor 2: Gradient strength (indicates sharp vs gradual change)
        gradient_factor = min(gradient_strength / 2.0, 1.0)
        quality += gradient_factor * 0.3  # 30% weight
        
        # Factor 3: Consistency with expected die geometry and circular pattern
        estimated_center_x, estimated_center_y = self._estimate_dynamic_center()
        distance_from_center = np.sqrt((x - estimated_center_x)**2 + (y - estimated_center_y)**2)
        expected_radius = self.expected_die_diameter / 2
        
        # Quality decreases if too far from expected radius
        radius_error = abs(distance_from_center - expected_radius)
        radius_factor = max(0, 1.0 - (radius_error / (expected_radius * 0.5)))
        
        # Circular consistency: check if edge direction is consistent with circle
        circular_consistency = self._check_circular_consistency(x, y, estimated_center_x, estimated_center_y, height_diff)
        
        # Combine radius and circular consistency
        geometry_factor = (radius_factor * 0.7) + (circular_consistency * 0.3)
        quality += geometry_factor * 0.2  # 20% weight
        
        # Factor 4: Noise assessment based on recent height stability
        if len(self.recent_heights) >= 5:
            recent_std = np.std(self.recent_heights[-5:])
            noise_factor = max(0, 1.0 - (recent_std / 1.0))  # Lower quality if noisy
            quality += noise_factor * 0.1  # 10% weight
        else:
            quality += 0.05  # Neutral contribution if insufficient data
        
        return min(quality, 1.0)
    
    def _confirm_edge_point(self, edge_point):
        """Multi-point edge confirmation - require consistent edges in nearby locations"""
        # Add current edge to pending confirmation list
        edge_data = {
            'point': edge_point,
            'timestamp': time.time(),
            'confirmed_count': 1
        }
        
        # Check for nearby pending edges of same type
        confirmation_distance = 5.0  # mm - max distance to consider "nearby"
        confirmed = False
        
        for pending in self.pending_edges:
            # Calculate distance between edge points
            distance = np.sqrt(
                (edge_point.x - pending['point'].x)**2 + 
                (edge_point.y - pending['point'].y)**2
            )
            
            # If nearby and same edge type, increment confirmation count
            if (distance <= confirmation_distance and 
                edge_point.edge_type == pending['point'].edge_type):
                pending['confirmed_count'] += 1
                
                # If sufficient confirmations, accept the edge
                if pending['confirmed_count'] >= self.edge_confirmation_points:
                    confirmed = True
                    # Update the confirmed edge location to average of confirmations
                    edge_point.x = (edge_point.x + pending['point'].x) / 2
                    edge_point.y = (edge_point.y + pending['point'].y) / 2
                    edge_point.confidence = max(edge_point.confidence, pending['point'].confidence)
                    
                    # Remove confirmed edge from pending list
                    self.pending_edges.remove(pending)
                    break
        
        # If not confirmed yet, add to pending list
        if not confirmed:
            self.pending_edges.append(edge_data)
            
            # Clean up old pending edges (older than 10 seconds)
            current_time = time.time()
            self.pending_edges = [p for p in self.pending_edges 
                                if current_time - p['timestamp'] < 10.0]
            
            # For initial implementation, also accept high-confidence edges immediately
            if edge_point.confidence > 0.8:
                confirmed = True
                logger.debug(f"High-confidence edge accepted immediately: confidence={edge_point.confidence:.2f}")
        
        return confirmed

    def _estimate_dynamic_center(self):
        """Estimate die center dynamically based on current edge points"""
        if len(self.edge_points) < 3:
            # Use known center as fallback for initial estimation
            return 398.0, 809.0
        
        # Use simple centroid of edge points as rough center estimate
        edge_coords = np.array([[ep.x, ep.y] for ep in self.edge_points])
        center_x = np.mean(edge_coords[:, 0])
        center_y = np.mean(edge_coords[:, 1])
        
        return center_x, center_y
    
    def _check_circular_consistency(self, x, y, center_x, center_y, height_diff):
        """Check if edge direction is consistent with circular die pattern"""
        if len(self.edge_points) < 2:
            return 0.5  # Neutral score if insufficient data
        
        # Calculate vector from center to current point
        center_to_point = np.array([x - center_x, y - center_y])
        
        # For a circular die, edges should generally point radially outward (for drop-offs)
        # or inward (for rises), depending on whether we're scanning from inside or outside
        
        # Calculate expected edge direction based on scanning pattern
        # Since we're scanning with linear passes, the edge direction should be roughly
        # perpendicular to the scanning direction at the edge location
        
        # For circular consistency, edges should form a roughly circular pattern
        # Check if the current edge is consistent with nearby edges
        consistency_score = 0.5  # Start with neutral
        
        if len(self.edge_points) >= 3:
            # Calculate angle from center to current point
            current_angle = np.arctan2(y - center_y, x - center_x)
            
            # Check consistency with nearby edge points
            nearby_edges = 0
            consistent_edges = 0
            
            for edge in self.edge_points[-5:]:  # Check last 5 edges
                edge_angle = np.arctan2(edge.y - center_y, edge.x - center_x)
                angle_diff = abs(current_angle - edge_angle)
                
                # Normalize angle difference to [0, π]
                if angle_diff > np.pi:
                    angle_diff = 2 * np.pi - angle_diff
                
                # If edges are nearby (within 30 degrees), check consistency
                if angle_diff < np.pi / 6:  # 30 degrees
                    nearby_edges += 1
                    # Edges should have similar type for circular consistency
                    if edge.edge_type == ('drop_off' if height_diff < 0 else 'rise'):
                        consistent_edges += 1
            
            if nearby_edges > 0:
                consistency_score = consistent_edges / nearby_edges
        
        return consistency_score

    def _classify_edge_location(self, edge_point):
        """Classify edge as interior, exterior, or uncertain based on geometry"""
        # Get current best estimate of center
        estimated_center_x, estimated_center_y = self._estimate_dynamic_center()
        
        # Calculate distance from estimated center
        distance_from_center = np.sqrt(
            (edge_point.x - estimated_center_x)**2 + 
            (edge_point.y - estimated_center_y)**2
        )
        
        # Apply radius-based classification
        if distance_from_center < self.interior_exclusion_radius:
            return 'interior'  # Definitely inside die
        elif distance_from_center >= self.min_outer_radius and distance_from_center <= self.max_outer_radius:
            return 'exterior'  # Likely outer perimeter
        elif distance_from_center > self.max_outer_radius:
            return 'interior'  # Too far out, probably noise or artifact
        else:
            return 'uncertain'  # In transition zone, needs further analysis
    
    def filter_interior_edges(self):
        """Remove interior edges from edge_points list and populate outer_edges"""
        if not self.edge_points:
            return
        
        # Clear previous classifications
        self.outer_edges.clear()
        self.interior_edges.clear()
        filtered_edges = []
        
        # Get current center estimate
        estimated_center_x, estimated_center_y = self._estimate_dynamic_center()
        
        logger.info(f"Filtering edges using center estimate: ({estimated_center_x:.1f}, {estimated_center_y:.1f})")
        logger.info(f"Radius criteria: min={self.min_outer_radius:.1f}mm, max={self.max_outer_radius:.1f}mm")
        
        # Classify all edges
        for i, edge in enumerate(self.edge_points):
            distance_from_center = np.sqrt(
                (edge.x - estimated_center_x)**2 + 
                (edge.y - estimated_center_y)**2
            )
            
            if (distance_from_center >= self.min_outer_radius and 
                distance_from_center <= self.max_outer_radius):
                self.outer_edges.append(edge)
                filtered_edges.append(edge)
                logger.info(f"Edge {i+1} ACCEPTED as OUTER: ({edge.x:.1f}, {edge.y:.1f}) dist={distance_from_center:.1f}mm")
            else:
                self.interior_edges.append(edge)
                self.filtered_edge_count += 1
                logger.info(f"Edge {i+1} FILTERED as INTERIOR: ({edge.x:.1f}, {edge.y:.1f}) dist={distance_from_center:.1f}mm")
        
        # Update main edge list to only contain outer edges
        self.edge_points = filtered_edges
        
        logger.info(f"Edge filtering complete: {len(self.outer_edges)} outer edges, {len(self.interior_edges)} interior edges filtered")
    
    def validate_perimeter_continuity(self):
        """Check if outer edges form a continuous perimeter with sufficient coverage"""
        if len(self.outer_edges) < self.min_perimeter_edges:
            logger.warning(f"Insufficient outer edges: {len(self.outer_edges)} < {self.min_perimeter_edges}")
            return False
        
        # Calculate angular coverage
        estimated_center_x, estimated_center_y = self._estimate_dynamic_center()
        angles = []
        
        for edge in self.outer_edges:
            angle = np.arctan2(edge.y - estimated_center_y, edge.x - estimated_center_x)
            angles.append(np.degrees(angle) % 360)  # Convert to 0-360 degrees
        
        # Sort angles and check coverage
        angles.sort()
        
        # Calculate maximum gap between consecutive angles
        max_gap = 0
        for i in range(len(angles)):
            gap = angles[(i + 1) % len(angles)] - angles[i]
            if gap < 0:  # Handle wrap-around at 360°
                gap += 360
            max_gap = max(max_gap, gap)
        
        # Calculate total coverage (360° - largest gap)
        coverage = 360 - max_gap
        
        if coverage < self.min_angular_coverage:
            logger.warning(f"Insufficient angular coverage: {coverage:.1f}° < {self.min_angular_coverage}°")
            logger.debug(f"Angular distribution of {len(self.outer_edges)} outer edges:")
            for i, angle in enumerate(angles):
                logger.debug(f"  Edge {i+1}: {angle:.1f}°")
            logger.debug(f"Max gap between edges: {max_gap:.1f}°")
            return False
        
        logger.info(f"Perimeter validation passed: {len(self.outer_edges)} edges, {coverage:.1f}° coverage")
        logger.debug(f"Edge angular distribution: max gap {max_gap:.1f}°, coverage {coverage:.1f}°")
        return True

    def ransac_circle_fit(self, x, y, max_iterations=100, distance_threshold=3.0):
        """RANSAC circle fitting to remove outliers"""
        if len(x) < 3:
            return np.mean(x), np.mean(y), 0, []
        
        best_inliers = []
        best_center_x, best_center_y, best_radius = 0, 0, 0
        
        for _ in range(max_iterations):
            # Randomly sample 3 points
            if len(x) < 3:
                break
            sample_indices = np.random.choice(len(x), 3, replace=False)
            sample_x = x[sample_indices]
            sample_y = y[sample_indices]
            
            # Fit circle to 3 points
            try:
                center_x, center_y, radius = self._fit_circle_3_points(sample_x, sample_y)
                
                # Check if circle parameters are reasonable
                if (radius < self.min_outer_radius * 0.8 or radius > self.max_outer_radius * 1.2):
                    continue
                
                # Find inliers
                distances = np.sqrt((x - center_x)**2 + (y - center_y)**2)
                radius_errors = np.abs(distances - radius)
                inliers = np.where(radius_errors < distance_threshold)[0]
                
                # Keep best fit with most inliers
                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_center_x, best_center_y, best_radius = center_x, center_y, radius
                    
            except:
                continue
        
        # If sufficient inliers found, refit using all inliers
        if len(best_inliers) >= 4:
            inlier_x = x[best_inliers]
            inlier_y = y[best_inliers]
            best_center_x, best_center_y = self.kasa_circle_fit(inlier_x, inlier_y)
        
        return best_center_x, best_center_y, best_radius, best_inliers
    
    def _fit_circle_3_points(self, x, y):
        """Fit circle through 3 points"""
        # Convert to homogeneous coordinates and solve
        A = np.array([
            [x[0], y[0], 1],
            [x[1], y[1], 1],
            [x[2], y[2], 1]
        ])
        B = -(x**2 + y**2)
        
        if np.linalg.det(A) == 0:
            # Points are collinear, return centroid
            return np.mean(x), np.mean(y), np.mean(np.sqrt(x**2 + y**2))
        
        coeffs = np.linalg.solve(A, B)
        center_x = -coeffs[0] / 2
        center_y = -coeffs[1] / 2
        radius = np.sqrt(center_x**2 + center_y**2 - coeffs[2])
        
        return center_x, center_y, radius
    
    def weighted_least_squares_fit(self, x, y):
        """Weighted least squares using edge quality scores"""
        if len(self.outer_edges) != len(x):
            # Fallback to standard least squares if size mismatch
            return self.kasa_circle_fit(x, y)
        
        # Extract weights from edge confidence scores
        weights = np.array([edge.confidence for edge in self.outer_edges])
        weights = weights / np.sum(weights)  # Normalize weights
        
        # Weighted least squares formulation
        W = np.diag(weights)
        A = np.column_stack([2*x, 2*y, np.ones(len(x))])
        B = x**2 + y**2
        
        # Solve weighted least squares: (A^T W A) * params = A^T W B
        try:
            params = np.linalg.solve(A.T @ W @ A, A.T @ W @ B)
            center_x = params[0]
            center_y = params[1]
            return center_x, center_y
        except:
            # Fallback to standard method if weighted fails
            return self.kasa_circle_fit(x, y)
    
    def kasa_circle_fit(self, x, y):
        """Standard Kasa algebraic circle fitting"""
        A = np.column_stack([2*x, 2*y, np.ones(len(x))])
        B = x**2 + y**2
        try:
            params = np.linalg.lstsq(A, B, rcond=None)[0]
            return params[0], params[1]
        except:
            return np.mean(x), np.mean(y)
    
    def taubin_circle_fit(self, x, y):
        """Taubin geometric circle fitting (more accurate than Kasa)"""
        # Center data
        x_mean = np.mean(x)
        y_mean = np.mean(y)
        x_centered = x - x_mean
        y_centered = y - y_mean
        
        # Calculate moments
        Mxx = np.mean(x_centered**2)
        Myy = np.mean(y_centered**2)
        Mxy = np.mean(x_centered * y_centered)
        Mxz = np.mean(x_centered * (x_centered**2 + y_centered**2))
        Myz = np.mean(y_centered * (x_centered**2 + y_centered**2))
        Mzz = np.mean((x_centered**2 + y_centered**2)**2)
        
        # Taubin method coefficients
        try:
            Cov_xy = Mxx * Myy - Mxy**2
            Var_z = Mzz - (Mxx + Myy)**2
            
            if abs(Cov_xy) < 1e-12:
                # Fallback to centroid if degenerate
                return x_mean, y_mean
            
            A2 = 4 * Cov_xy
            A1 = Var_z
            A0 = Mzz * Cov_xy - Mxz**2 * Myy - Myz**2 * Mxx + 2 * Mxz * Myz * Mxy
            
            A = A1 / A2
            B = A0 / A2
            
            # Solve quadratic
            discriminant = A**2 - 4*B
            if discriminant < 0:
                return x_mean, y_mean
            
            lambda_val = (-A + np.sqrt(discriminant)) / 2
            
            center_x = x_mean + (Mxz * (Myy - lambda_val) - Myz * Mxy) / (2 * Cov_xy)
            center_y = y_mean + (Myz * (Mxx - lambda_val) - Mxz * Mxy) / (2 * Cov_xy)
            
            return center_x, center_y
            
        except:
            # Fallback to Kasa method
            return self.kasa_circle_fit(x, y)
    
    def multi_method_consensus(self, method_results):
        """Calculate consensus center from multiple fitting methods"""
        valid_results = []
        total_weight = 0
        
        for center_x, center_y, weight in method_results:
            # Validate result is reasonable
            if (not np.isnan(center_x) and not np.isnan(center_y) and
                abs(center_x) < 2000 and abs(center_y) < 2000):  # Sanity check
                valid_results.append((center_x, center_y, weight))
                total_weight += weight
        
        if not valid_results:
            # Fallback to simple average if all methods failed
            logger.warning("All fitting methods failed, using fallback center")
            return 398.0, 809.0, 10.0  # Known approximate center
        
        # Weighted average of valid results
        weighted_x = sum(x * w for x, y, w in valid_results) / total_weight
        weighted_y = sum(y * w for x, y, w in valid_results) / total_weight
        
        # Calculate confidence based on consistency
        x_coords = [x for x, y, w in valid_results]
        y_coords = [y for x, y, w in valid_results]
        
        x_std = np.std(x_coords) if len(x_coords) > 1 else 0
        y_std = np.std(y_coords) if len(y_coords) > 1 else 0
        
        # Confidence decreases with standard deviation
        consistency_score = max(0, 100 - (x_std + y_std) * 5)
        
        return weighted_x, weighted_y, consistency_score

    def set_error_status(self, error_msg="Unknown error"):
        """Set error status and log failure"""
        try:
            self.robot._write_parameter('ds_status', 3)  # 3 = error
            logger.error(f"Die scanning failed: {error_msg}")
            print(f"❌ SCAN FAILED: {error_msg}")
        except Exception as e:
            logger.error(f"Failed to set error status: {e}")

    def set_success_status(self, center_result):
        """Set success status and output results"""
        try:
            self.robot._write_parameter('ds_status', 2)  # 2 = complete (success)
            logger.info("Die scanning completed successfully")
            print(f"✅ SCAN COMPLETE: Center [{center_result.center_x:.1f}, {center_result.center_y:.1f}]")
        except Exception as e:
            logger.error(f"Failed to set success status: {e}")

    def get_preliminary_center_from_5_pass(self):
        """Extract preliminary center from 5-pass Y-direction scan results"""
        if len(self.outer_edges) < 4:
            # Fallback to known approximate center
            logger.warning("Insufficient outer edges for preliminary center, using fallback")
            logger.debug(f"Only {len(self.outer_edges)} outer edges available, need at least 4")
            return 398.0, 809.0
        
        # Quick centroid calculation from outer edges found in Y-direction scanning
        x_coords = [edge.x for edge in self.outer_edges]
        y_coords = [edge.y for edge in self.outer_edges] 
        
        prelim_x = np.mean(x_coords)
        prelim_y = np.mean(y_coords)
        
        # Log detailed edge information
        logger.info(f"Preliminary center from 5-pass: ({prelim_x:.1f}, {prelim_y:.1f}) based on {len(self.outer_edges)} outer edges")
        logger.debug("Outer edges used for preliminary center:")
        for i, edge in enumerate(self.outer_edges):
            distance = np.sqrt((edge.x - prelim_x)**2 + (edge.y - prelim_y)**2)
            logger.debug(f"  Edge {i+1}: ({edge.x:.1f}, {edge.y:.1f}) dist={distance:.1f}mm type={edge.edge_type}")
        
        return prelim_x, prelim_y

    def filter_edges_by_distance(self, center_x, center_y, max_radius=63.5, edge_list=None):
        """Filter edges to only include those within 2.5" (63.5mm) of center"""
        filtered_edges = []
        excluded_count = 0
        
        # Use provided edge list or default to outer edges
        edges_to_filter = edge_list if edge_list is not None else self.outer_edges
        
        for edge in edges_to_filter:
            distance = np.sqrt((edge.x - center_x)**2 + (edge.y - center_y)**2)
            if distance <= max_radius:
                filtered_edges.append(edge)
            else:
                excluded_count += 1
                logger.debug(f"Excluded peripheral edge at ({edge.x:.1f}, {edge.y:.1f}) - {distance:.1f}mm from center")
        
        logger.info(f"Distance filtering: {len(filtered_edges)} edges kept, {excluded_count} peripheral edges excluded (max radius: {max_radius:.1f}mm)")
        return filtered_edges

    def _log_scan_completion_stats(self, scan_count):
        """Log comprehensive scan completion statistics including invalid readings"""
        # Count valid scan points and those with invalid heights
        valid_scan_points = len([sp for sp in self.scan_points if sp.is_valid and sp.height > -999])
        invalid_height_points = len([sp for sp in self.scan_points if not sp.is_valid or sp.height <= -999])
        total_attempts = scan_count + invalid_height_points  # Approximate total attempts
        
        logger.info(f"=== 5-Pass Scan Completion Statistics ===")
        logger.info(f"Valid scan points collected: {valid_scan_points}")
        logger.info(f"Invalid readings excluded: {invalid_height_points}")
        logger.info(f"Total edge points detected: {len(self.edge_points) + len(self.interior_edges)}")
        logger.info(f"Outer edges (for center calc): {len(self.outer_edges)}")
        logger.info(f"Interior edges filtered: {len(self.interior_edges)}")
        logger.info(f"Data quality: {(valid_scan_points/max(1,total_attempts)*100):.1f}% valid readings")
        
        if invalid_height_points > 0:
            logger.warning(f"Excluded {invalid_height_points} invalid readings (-999 values) from calculations")
        
        if len(self.interior_edges) > 0:
            logger.info(f"Interior edge filtering active: {len(self.interior_edges)} interior edges excluded from center calculation")

    def calculate_final_center(self):
        """Calculate die center from OUTER EDGE points only (filtered)"""
        # Apply final edge filtering to ensure we only use outer edges
        self.filter_interior_edges()
        
        logger.info(f"Starting center calculation with {len(self.outer_edges)} outer edges (filtered from {len(self.edge_points) + len(self.interior_edges)} total)")
        
        # Validate we have sufficient outer edges
        if len(self.outer_edges) < self.min_perimeter_edges:
            logger.error(f"Insufficient outer edges for center calculation: {len(self.outer_edges)} < {self.min_perimeter_edges}")
            logger.info("Outer edges found:")
            for i, ep in enumerate(self.outer_edges):
                logger.info(f"  Outer Edge {i+1}: ({ep.x:.1f}, {ep.y:.1f}) - {ep.edge_type}")
            return None
        
        # Validate perimeter continuity
        if not self.validate_perimeter_continuity():
            logger.error("Perimeter continuity validation failed - insufficient coverage")
            return None
            
        try:
            # Use ONLY outer edges for center calculation
            edge_coords = np.array([[ep.x, ep.y] for ep in self.outer_edges])
            
            # Extract coordinates for fitting
            x = edge_coords[:, 0]
            y = edge_coords[:, 1]
            
            # Method 1: RANSAC Circle Fitting (robust to outliers)
            center_x_ransac, center_y_ransac, radius_ransac, inliers = self.ransac_circle_fit(x, y)
            
            # Method 2: Weighted Least squares (using edge quality)
            center_x_weighted, center_y_weighted = self.weighted_least_squares_fit(x, y)
            
            # Method 3: Standard Least squares circle fitting (Kasa method)
            center_x_ls, center_y_ls = self.kasa_circle_fit(x, y)
            
            # Method 4: Geometric circle fitting (Taubin method - more accurate)
            center_x_taubin, center_y_taubin = self.taubin_circle_fit(x, y)
            
            # Get preliminary center for distance filtering
            prelim_center_x, prelim_center_y = self.get_preliminary_center_from_5_pass()
            
            # Apply distance-based edge filtering using preliminary center
            filtered_outer_edges = self.filter_edges_by_distance(prelim_center_x, prelim_center_y, max_radius=63.5, edge_list=self.outer_edges)
            
            # Re-run fitting methods with distance-filtered edges if we have enough
            if len(filtered_outer_edges) >= self.min_perimeter_edges:
                logger.info(f"Re-calculating with {len(filtered_outer_edges)} distance-filtered edges")
                filtered_coords = np.array([[ep.x, ep.y] for ep in filtered_outer_edges])
                x_filtered = filtered_coords[:, 0]
                y_filtered = filtered_coords[:, 1]
                
                # Re-run methods with filtered data
                center_x_ransac, center_y_ransac, radius_ransac, inliers = self.ransac_circle_fit(x_filtered, y_filtered)
                center_x_weighted, center_y_weighted = self.weighted_least_squares_fit(x_filtered, y_filtered)
                center_x_ls, center_y_ls = self.kasa_circle_fit(x_filtered, y_filtered)
                center_x_taubin, center_y_taubin = self.taubin_circle_fit(x_filtered, y_filtered)
                
                x, y = x_filtered, y_filtered  # Use filtered data for final results
            else:
                logger.warning(f"Distance filtering removed too many edges ({len(filtered_outer_edges)} remaining), using original data")
            
            # Multi-method consensus calculation
            final_center_x, final_center_y, confidence = self.multi_method_consensus([
                (center_x_ransac, center_y_ransac, 0.4),  # RANSAC - highest weight
                (center_x_weighted, center_y_weighted, 0.3),  # Weighted - second highest
                (center_x_taubin, center_y_taubin, 0.2),  # Taubin - geometric accuracy
                (center_x_ls, center_y_ls, 0.1),  # Kasa - backup method
            ])
            
            logger.info(f"=== Multi-Method Circle Fitting Results ===")
            logger.info(f"RANSAC: ({center_x_ransac:.1f}, {center_y_ransac:.1f}) - {len(inliers)}/{len(x)} inliers")
            logger.info(f"Weighted LS: ({center_x_weighted:.1f}, {center_y_weighted:.1f})")
            logger.info(f"Taubin: ({center_x_taubin:.1f}, {center_y_taubin:.1f})")
            logger.info(f"Kasa LS: ({center_x_ls:.1f}, {center_y_ls:.1f})")
            logger.info(f"Final consensus: ({final_center_x:.1f}, {final_center_y:.1f}) confidence={confidence:.1f}%")
            
            # Calculate diameter estimate
            distances = np.sqrt((x - final_center_x)**2 + (y - final_center_y)**2)
            diameter = np.mean(distances) * 2
            
            # Calculate average height - explicitly filter out -999 invalid readings
            valid_heights = [sp.height for sp in self.scan_points if sp.is_valid and sp.height > -999]
            if valid_heights:
                avg_height = np.mean(valid_heights)
                logger.debug(f"Average height calculated from {len(valid_heights)} valid readings (excluded {len(self.scan_points) - len(valid_heights)} invalid readings)")
            else:
                logger.warning("No valid heights found for average calculation")
                avg_height = 0.0
            
            logger.info(f"=== Final Center Calculation Summary ===")
            logger.info(f"Final center: ({final_center_x:.1f}, {final_center_y:.1f})")
            logger.info(f"Diameter: {diameter:.1f}mm, Confidence: {confidence:.1f}%")
            logger.info(f"Based on {len(self.outer_edges)} outer edges, {len(self.interior_edges)} interior edges filtered")
            
            return DieCenter(
                center_x=final_center_x,
                center_y=final_center_y,
                diameter=diameter,
                avg_height=avg_height,
                confidence=confidence,
                edge_points=self.edge_points
            )
            
        except Exception as e:
            logger.error(f"Center calculation failed: {e}")
            return None
            
    def write_results_to_robot(self, die_center):
        """Write comprehensive results to robot registers and console"""
        try:
            # Write primary results to robot
            self.robot._write_parameter('ds_center_x', die_center.center_x)
            self.robot._write_parameter('ds_center_y', die_center.center_y)
            self.robot._write_parameter('ds_avg_height', die_center.avg_height)
            self.robot._write_parameter('ds_confidence', int(die_center.confidence))
            self.robot._write_parameter('ds_total_points', len(self.scan_points))
            
            # Write edge points (up to 8 points, prioritize outer edges)
            edge_registers = [
                ('ds_edge_x1', 'ds_edge_y1'), ('ds_edge_x2', 'ds_edge_y2'),
                ('ds_edge_x3', 'ds_edge_y3'), ('ds_edge_x4', 'ds_edge_y4'),
                ('ds_edge_x5', 'ds_edge_y5'), ('ds_edge_x6', 'ds_edge_y6'),
                ('ds_edge_x7', 'ds_edge_y7'), ('ds_edge_x8', 'ds_edge_y8')
            ]
            
            # Prioritize outer edges for robot register output
            edges_to_write = self.outer_edges[:8] if len(self.outer_edges) >= 8 else self.outer_edges
            
            for i, (x_reg, y_reg) in enumerate(edge_registers):
                if i < len(edges_to_write):
                    self.robot._write_parameter(x_reg, edges_to_write[i].x)
                    self.robot._write_parameter(y_reg, edges_to_write[i].y)
                else:
                    self.robot._write_parameter(x_reg, 0.0)
                    self.robot._write_parameter(y_reg, 0.0)
            
            # Set success status
            self.set_success_status(die_center)
            
            # Comprehensive console output
            self._print_comprehensive_results(die_center)
            
            logger.info("Results written to robot registers and displayed")
            
        except Exception as e:
            logger.error(f"Failed to write results to robot: {e}")
            self.set_error_status(f"Results output failed: {e}")
    
    def _print_comprehensive_results(self, die_center):
        """Print comprehensive scanning results to console"""
        print("\n" + "="*60)
        print("           🎯 DIE SCANNING RESULTS - SUCCESS")
        print("="*60)
        
        # Primary results
        print(f"📍 DIE CENTER: [{die_center.center_x:.1f}, {die_center.center_y:.1f}] mm")
        print(f"📏 DIAMETER: {die_center.diameter:.1f} mm")
        print(f"📐 AVG HEIGHT: {die_center.avg_height:.1f} mm")
        print(f"🎯 CONFIDENCE: {die_center.confidence:.1f}%")
        
        # Data quality metrics
        valid_scan_points = len([sp for sp in self.scan_points if sp.is_valid and sp.height > -999])
        invalid_readings = len(self.scan_points) - valid_scan_points
        data_quality = (valid_scan_points / max(1, len(self.scan_points))) * 100
        
        print(f"\n📊 DATA QUALITY:")
        print(f"   • Valid scan points: {valid_scan_points}")
        print(f"   • Invalid readings: {invalid_readings}")
        print(f"   • Data quality: {data_quality:.1f}%")
        
        # Edge detection summary
        print(f"\n🔍 EDGE DETECTION:")
        print(f"   • Total edges found: {len(self.edge_points) + len(self.interior_edges)}")
        print(f"   • Outer edges (used): {len(self.outer_edges)}")
        print(f"   • Interior edges (filtered): {len(self.interior_edges)}")
        
        # Comprehensive scanning coverage analysis
        coverage_analysis = self._analyze_scanning_coverage()
        print(f"\n📏 SCAN COVERAGE ANALYSIS:")
        print(f"   • X-range: {coverage_analysis['x_range']:.1f} mm")
        print(f"   • Y-range: {coverage_analysis['y_range']:.1f} mm")
        print(f"   • Total area scanned: {coverage_analysis['total_area']:.0f} mm²")
        print(f"   • Point density: {coverage_analysis['point_density']:.1f} points/mm²")
        print(f"   • Edge coverage: {coverage_analysis['edge_coverage']:.1f}% of perimeter")
        print(f"   • Scanning efficiency: {coverage_analysis['efficiency']:.1f}%")
        
        # True center comparison (known target)
        true_center_x, true_center_y = 398.0, 809.0
        error_x = abs(die_center.center_x - true_center_x)
        error_y = abs(die_center.center_y - true_center_y)
        total_error = np.sqrt(error_x**2 + error_y**2)
        
        print(f"\n🎯 ACCURACY (vs true center [398, 809]):")
        print(f"   • X error: {error_x:.1f} mm")
        print(f"   • Y error: {error_y:.1f} mm")
        print(f"   • Total error: {total_error:.1f} mm")
        
        if total_error <= 1.0:
            print(f"   ✅ TARGET ACHIEVED: ±1mm accuracy")
        elif total_error <= 2.0:
            print(f"   ⚠️  CLOSE: Within 2mm accuracy")
        else:
            print(f"   ❌ IMPROVEMENT NEEDED: >2mm error")
        
        # ASCII die surface visualization
        print(f"\n🗺️  DIE SURFACE MAP:")
        self._print_ascii_die_surface(die_center)
        
        print("="*60)
        print(f"✅ SCAN COMPLETE - Results written to robot registers")
        print("="*60 + "\n")
        
    def _print_ascii_die_surface(self, die_center):
        """Print ASCII visualization of die surface with scan points and edges"""
        if not self.scan_points:
            print("   No scan data available for visualization")
            return
        
        # Get valid scan points with heights
        valid_points = [sp for sp in self.scan_points if sp.is_valid and sp.height > -999]
        if not valid_points:
            print("   No valid scan points for visualization")
            return
        
        # Determine visualization bounds
        x_coords = [sp.x for sp in valid_points]
        y_coords = [sp.y for sp in valid_points]
        heights = [sp.height for sp in valid_points]
        
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        h_min, h_max = min(heights), max(heights)
        
        # Create ASCII grid (30x20 characters for reasonable display)
        grid_width, grid_height = 30, 20
        grid = [[' ' for _ in range(grid_width)] for _ in range(grid_height)]
        
        # Height symbols (low to high)
        height_symbols = ['.', '~', '-', '=', '#', '@']
        
        # Map scan points to grid
        for sp in valid_points:
            # Convert to grid coordinates
            grid_x = int((sp.x - x_min) / (x_max - x_min + 0.001) * (grid_width - 1))
            grid_y = int((sp.y - y_min) / (y_max - y_min + 0.001) * (grid_height - 1))
            
            # Ensure within bounds
            grid_x = max(0, min(grid_width - 1, grid_x))
            grid_y = max(0, min(grid_height - 1, grid_y))
            
            # Convert height to symbol
            if h_max > h_min:
                height_ratio = (sp.height - h_min) / (h_max - h_min)
            else:
                height_ratio = 0.5
            symbol_idx = int(height_ratio * (len(height_symbols) - 1))
            symbol_idx = max(0, min(len(height_symbols) - 1, symbol_idx))
            
            grid[grid_y][grid_x] = height_symbols[symbol_idx]
        
        # Mark die center on grid
        center_grid_x = int((die_center.center_x - x_min) / (x_max - x_min + 0.001) * (grid_width - 1))
        center_grid_y = int((die_center.center_y - y_min) / (y_max - y_min + 0.001) * (grid_height - 1))
        center_grid_x = max(0, min(grid_width - 1, center_grid_x))
        center_grid_y = max(0, min(grid_height - 1, center_grid_y))
        grid[center_grid_y][center_grid_x] = '+'
        
        # Mark outer edges on grid
        for edge in self.outer_edges:
            edge_grid_x = int((edge.x - x_min) / (x_max - x_min + 0.001) * (grid_width - 1))
            edge_grid_y = int((edge.y - y_min) / (y_max - y_min + 0.001) * (grid_height - 1))
            edge_grid_x = max(0, min(grid_width - 1, edge_grid_x))
            edge_grid_y = max(0, min(grid_height - 1, edge_grid_y))
            grid[edge_grid_y][edge_grid_x] = 'E'
        
        # Print grid with coordinates
        print(f"   Area: X[{x_min:.0f}-{x_max:.0f}] Y[{y_min:.0f}-{y_max:.0f}] mm")
        print(f"   Heights: {h_min:.1f} to {h_max:.1f} mm")
        print("   Symbols: . ~ - = # @ (low→high)  + = center  E = edges")
        print()
        
        # Print grid with Y coordinates
        for i, row in enumerate(grid):
            y_val = y_max - (i / (grid_height - 1)) * (y_max - y_min)
            print(f"   {y_val:4.0f} │{''.join(row)}│")
        
        # Print X coordinate scale
        print("       └" + "─" * grid_width + "┘")
        x_scale = "        "
        for i in range(0, grid_width, 6):
            x_val = x_min + (i / (grid_width - 1)) * (x_max - x_min)
            x_scale += f"{x_val:4.0f}  "
        print(x_scale)
        print()
    
    def _analyze_scanning_coverage(self):
        """Analyze scanning coverage for quality assessment"""
        coverage_data = {
            'x_range': 0.0,
            'y_range': 0.0,
            'total_area': 0.0,
            'point_density': 0.0,
            'edge_coverage': 0.0,
            'efficiency': 0.0
        }
        
        if not self.scan_points:
            return coverage_data
        
        # Get valid scan points
        valid_points = [sp for sp in self.scan_points if sp.is_valid and sp.height > -999]
        if not valid_points:
            return coverage_data
        
        # Basic coverage metrics
        x_coords = [sp.x for sp in valid_points]
        y_coords = [sp.y for sp in valid_points]
        
        x_range = max(x_coords) - min(x_coords)
        y_range = max(y_coords) - min(y_coords)
        total_area = x_range * y_range
        
        # Point density (points per square mm)
        point_density = len(valid_points) / max(1, total_area)
        
        # Edge coverage analysis (estimate perimeter coverage)
        die_radius = 25.4  # ~1 inch radius for typical die
        expected_perimeter = 2 * np.pi * die_radius
        
        # Calculate coverage around perimeter
        if len(self.outer_edges) > 0:
            # Estimate angular coverage of detected edges
            edge_angles = []
            center_x = np.mean([sp.x for sp in valid_points])
            center_y = np.mean([sp.y for sp in valid_points])
            
            for edge in self.outer_edges:
                angle = np.arctan2(edge.y - center_y, edge.x - center_x)
                edge_angles.append(angle)
            
            # Sort angles and find gaps
            edge_angles.sort()
            if len(edge_angles) > 1:
                # Calculate angular spans covered
                angular_coverage = 0
                for i in range(len(edge_angles)):
                    next_i = (i + 1) % len(edge_angles)
                    gap = abs(edge_angles[next_i] - edge_angles[i])
                    if gap > np.pi:  # Handle wrap-around
                        gap = 2 * np.pi - gap
                    if gap < np.pi / 4:  # Only count small gaps as coverage
                        angular_coverage += gap
                
                edge_coverage = (angular_coverage / (2 * np.pi)) * 100
            else:
                edge_coverage = (len(self.outer_edges) / 8) * 100  # Rough estimate
        else:
            edge_coverage = 0.0
        
        # Scanning efficiency (ratio of useful points to total scans)
        total_scan_attempts = len(self.scan_points)  # Include invalid readings
        efficiency = (len(valid_points) / max(1, total_scan_attempts)) * 100
        
        # Log detailed coverage analysis
        logger.info(f"=== Scanning Coverage Analysis ===")
        logger.info(f"Valid scan points: {len(valid_points)}/{total_scan_attempts}")
        logger.info(f"Scan area: {x_range:.1f} x {y_range:.1f} mm = {total_area:.0f} mm²")
        logger.info(f"Point density: {point_density:.2f} points/mm²")
        logger.info(f"Edge detection: {len(self.outer_edges)} outer edges ({edge_coverage:.1f}% perimeter coverage)")
        logger.info(f"Scanning efficiency: {efficiency:.1f}% (valid readings)")
        
        return {
            'x_range': x_range,
            'y_range': y_range,
            'total_area': total_area,
            'point_density': point_density,
            'edge_coverage': edge_coverage,
            'efficiency': efficiency
        }
            
    def move_to_final_position(self, die_center):
        """Move robot to final position above die center"""
        try:
            self.robot._write_parameter('ds_command', 7)  # Move to final position command
            logger.info(f"Moving to final position above center: ({die_center.center_x:.1f}, {die_center.center_y:.1f})")
            
            # Wait for robot to complete final move
            # The robot should process the ds_status=12 signal and then execute the final move
            timeout = 30.0
            start_time = time.time()
            status_12_seen = False
            
            while time.time() - start_time < timeout:
                status = self.robot._read_parameter('ds_status')
                logger.debug(f"Robot status during final move: {status}")
                
                if status == 12:  # Still in scan complete state
                    status_12_seen = True
                    logger.debug("Robot processing scan completion...")
                elif status == 0 and status_12_seen:  # Complete after processing scan completion
                    logger.info("Robot completed final move")
                    break
                elif status == 0 and not status_12_seen:  # Already processed
                    logger.info("Robot at final position")
                    break
                    
                time.sleep(0.5)
            else:
                logger.warning("Robot final move timeout - robot may not have completed move")
                
        except Exception as e:
            logger.error(f"Failed to move to final position: {e}")
            
    def run_debug_scan(self):
        """Run complete debug scanning sequence"""
        logger.info("=== Die Scanner Debug Sequence Starting ===")
        
        try:
            # Step 1: Connect to systems
            if not self.connect_systems():
                return False
                
            # Step 2: Pick up sensor jig
            logger.info("Step 2: Pick up sensor jig")
            self.robot._write_parameter('ds_command', 1)
            
            # Wait for pickup completion
            timeout = 30.0
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self.robot._read_parameter('ds_status')
                if status == 0:  # Complete
                    break
                time.sleep(0.5)
                
            # Step 3: Move to scan start position
            logger.info("Step 3: Move to scan start position")
            self.robot._write_parameter('ds_command', 2)
            
            # Wait for move completion
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self.robot._read_parameter('ds_status')
                if status == 0:  # Complete
                    break
                time.sleep(0.5)
                
            # Step 4: Calibrate sensor
            if not self.calibrate_sensor():
                return False
                
            # Step 5: Start continuous scanning
            logger.info("Step 5: Start continuous scanning")
            self.robot._write_parameter('ds_command', 5)
            self.scanning_active = True
            
            # Wait for robot to signal ready
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self.robot._read_parameter('ds_status')
                if status == 10:  # Ready for scanning
                    break
                time.sleep(0.1)
                
            # Run scanning worker
            self.continuous_scan_worker(duration=30.0)
            
            # Stop scanning activity 
            self.scanning_active = False
            logger.info("Scanning activity stopped, proceeding to center calculation")
            
            # Step 6: Calculate and write results
            die_center = self.calculate_final_center()
            if die_center:
                self.write_results_to_robot(die_center)
                
                # Step 7: Move to final position
                self.move_to_final_position(die_center)
                
                logger.info("=== Die Scanner Debug Sequence Complete ===")
                return True
            else:
                logger.error("Center calculation failed")
                self.robot.set_status(3)  # Error status
                return False
                
        except Exception as e:
            logger.error(f"Debug scan failed: {e}")
            self.robot.set_status(3)  # Error status
            self.scanning_active = False
            return False
        finally:
            # Ensure scanning is stopped and systems are disconnected
            self.scanning_active = False
            # Don't disconnect systems here to allow successful completion

def main():
    """Main entry point for debug scanning"""
    scanner = ContinuousDieScanner()
    
    try:
        success = scanner.run_debug_scan()
        if success:
            print("Die scanning debug completed successfully")
        else:
            print("Die scanning debug failed")
            
    except KeyboardInterrupt:
        logger.info("User interrupted scan")
        scanner.scanning_active = False
        scanner.disconnect_systems()
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        scanner.scanning_active = False
        scanner.disconnect_systems()
    finally:
        # Always disconnect systems at the end
        scanner.disconnect_systems()

if __name__ == "__main__":
    main()