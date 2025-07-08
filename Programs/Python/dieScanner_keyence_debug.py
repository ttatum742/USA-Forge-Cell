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
from queue import Queue, Empty
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
        ds_current_x=proxy_simple.parameter('@0x6B/1/88', 'REAL', 'mm'),
        ds_current_y=proxy_simple.parameter('@0x6B/1/89', 'REAL', 'mm'),
        ds_current_z=proxy_simple.parameter('@0x6B/1/90', 'REAL', 'mm'),
        ds_point_counter=proxy_simple.parameter('@0x6B/1/91', 'DINT', 'count'),
        ds_total_points=proxy_simple.parameter('@0x6B/1/92', 'DINT', 'count'),
        ds_center_x=proxy_simple.parameter('@0x6B/1/93', 'REAL', 'mm'),
        ds_center_y=proxy_simple.parameter('@0x6B/1/94', 'REAL', 'mm'),
        ds_avg_height=proxy_simple.parameter('@0x6B/1/95', 'REAL', 'mm'),
        ds_confidence=proxy_simple.parameter('@0x6B/1/96', 'DINT', 'percent'),
        
        # Die Scan Edge Points (R[161-180])
        ds_edge_x1=proxy_simple.parameter('@0x6B/1/161', 'REAL', 'mm'),
        ds_edge_y1=proxy_simple.parameter('@0x6B/1/162', 'REAL', 'mm'),
        ds_edge_x2=proxy_simple.parameter('@0x6B/1/163', 'REAL', 'mm'),
        ds_edge_y2=proxy_simple.parameter('@0x6B/1/164', 'REAL', 'mm'),
        ds_edge_x3=proxy_simple.parameter('@0x6B/1/165', 'REAL', 'mm'),
        ds_edge_y3=proxy_simple.parameter('@0x6B/1/166', 'REAL', 'mm'),
        ds_edge_x4=proxy_simple.parameter('@0x6B/1/167', 'REAL', 'mm'),
        ds_edge_y4=proxy_simple.parameter('@0x6B/1/168', 'REAL', 'mm'),
        ds_edge_x5=proxy_simple.parameter('@0x6B/1/169', 'REAL', 'mm'),
        ds_edge_y5=proxy_simple.parameter('@0x6B/1/170', 'REAL', 'mm'),
        ds_edge_x6=proxy_simple.parameter('@0x6B/1/171', 'REAL', 'mm'),
        ds_edge_y6=proxy_simple.parameter('@0x6B/1/172', 'REAL', 'mm'),
        ds_edge_x7=proxy_simple.parameter('@0x6B/1/173', 'REAL', 'mm'),
        ds_edge_y7=proxy_simple.parameter('@0x6B/1/174', 'REAL', 'mm'),
        ds_edge_x8=proxy_simple.parameter('@0x6B/1/175', 'REAL', 'mm'),
        ds_edge_y8=proxy_simple.parameter('@0x6B/1/176', 'REAL', 'mm'),
        
        # Next position components (R[182-184]) - robot copies to PR[37]
        ds_next_pos_x=proxy_simple.parameter('@0x6B/1/182', 'REAL', 'mm'),
        ds_next_pos_y=proxy_simple.parameter('@0x6B/1/183', 'REAL', 'mm'),
        ds_next_pos_z=proxy_simple.parameter('@0x6B/1/184', 'REAL', 'mm'),
        
        # Measurement System (R[140-143])
        ds_measure_result=proxy_simple.parameter('@0x6B/1/140', 'REAL', 'mm'),
        ds_measure_trigger=proxy_simple.parameter('@0x6B/1/141', 'DINT', 'trigger'),
        ds_measure_complete=proxy_simple.parameter('@0x6B/1/142', 'DINT', 'complete'),
        ds_laser_reading=proxy_simple.parameter('@0x6B/1/143', 'REAL', 'mm')
    )

    def __init__(self, ip_address: str = "192.168.0.1"):
        super().__init__(host=ip_address, timeout=5.0)
        self.ip_address = ip_address
        logger.info(f"Robot interface initialized for {ip_address}")

    def read_robot_position(self) -> Tuple[float, float, float]:
        """Read current robot position"""
        try:
            x = self._read_parameter('ds_current_x') or 0.0
            y = self._read_parameter('ds_current_y') or 0.0  
            z = self._read_parameter('ds_current_z') or 0.0
            return x, y, z
        except Exception as e:
            logger.error(f"Failed to read robot position: {e}")
            return 0.0, 0.0, 0.0

    def write_die_center(self, center: DieCenter):
        """Write calculated die center to robot"""
        try:
            self._write_parameter('ds_center_x', center.center_x)
            self._write_parameter('ds_center_y', center.center_y)
            self._write_parameter('ds_avg_height', center.avg_height)
            self._write_parameter('ds_confidence', int(center.confidence))
            
            # Write edge points
            for i, edge_point in enumerate(center.edge_points[:8]):
                self._write_parameter(f'ds_edge_x{i+1}', edge_point.x)
                self._write_parameter(f'ds_edge_y{i+1}', edge_point.y)
                
            logger.info(f"Die center written to robot: ({center.center_x:.1f}, {center.center_y:.1f})")
            
        except Exception as e:
            logger.error(f"Failed to write die center: {e}")

    def set_status(self, status: int):
        """Set die scan status"""
        try:
            self._write_parameter('ds_status', status)
        except Exception as e:
            logger.error(f"Failed to set status: {e}")

    def _read_parameter(self, param_name: str) -> Optional[float]:
        """Read a single parameter value"""
        try:
            param_str = self.parameter_substitution(param_name)
            result, = self.read(param_str, checking=True)
            return float(result[0]) if isinstance(result, list) else float(result)
        except Exception as e:
            logger.error(f"Failed to read {param_name}: {e}")
            return None

    def _write_parameter(self, param_name: str, value: float):
        """Write a single parameter value"""
        try:
            param_str = self.parameter_substitution(param_name)
            self.write(param_str, value)
        except Exception as e:
            logger.error(f"Failed to write {param_name}: {e}")
            raise

class KeyenceAlwaysOnInterface:
    """Interface for keyence_always_on Arduino connection"""
    
    def __init__(self, port: str = "COM8", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.connection = None
        self.is_connected = False
        
    def connect(self) -> bool:
        """Connect to Arduino"""
        try:
            self.connection = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Allow Arduino to initialize
            
            # Test connection
            response = self.send_command("STATUS")
            if "STATUS:" in response:
                self.is_connected = True
                logger.info(f"Connected to Arduino on {self.port}")
                return True
            else:
                logger.error("Arduino not responding properly")
                return False
                
        except Exception as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.connection:
            try:
                self.connection.close()
                self.is_connected = False
                logger.info("Disconnected from Arduino")
            except Exception as e:
                logger.error(f"Error disconnecting: {e}")
    
    def send_command(self, command: str) -> str:
        """Send command to Arduino and get response"""
        if not self.is_connected:
            return "ERROR:NOT_CONNECTED"
            
        try:
            self.connection.write(f"{command}\n".encode())
            response = self.connection.readline().decode().strip()
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
        # Start calibration
        response = self.send_command("CALIBRATE")
        if "CALIBRATION:STARTING" not in response:
            return False
            
        time.sleep(0.5)
        
        # Send reference distance
        response = self.send_command(f"CAL_DISTANCE {reference_distance}")
        
        # Wait for completion
        for _ in range(30):  # 30 second timeout
            time.sleep(1)
            if "CALIBRATION:COMPLETE" in response:
                logger.info(f"Calibration completed with reference {reference_distance}mm")
                return True
            elif "CALIBRATION:ERROR" in response:
                logger.error(f"Calibration failed: {response}")
                return False
                
        logger.error("Calibration timeout")
        return False

class ContinuousDieScanner:
    """Main continuous die scanning class"""
    
    def __init__(self, robot_ip: str = "192.168.0.1", arduino_port: str = "COM8"):
        self.robot = DieScanRobotInterface(robot_ip)
        self.arduino = KeyenceAlwaysOnInterface(arduino_port)
        
        # Scanning parameters
        self.scan_points: List[ScanPoint] = []
        self.edge_points: List[EdgePoint] = []
        self.scanning_active = False
        self.scan_start_time = 0
        
        # Edge detection parameters
        self.height_threshold = 5.0  # mm drop for edge detection
        self.edge_confidence_threshold = 0.7
        self.expected_die_diameter = 107.95  # 4.25 inches in mm
        self.die_tolerance = 10.0  # mm tolerance for die diameter
        
        # Background position tracking
        self.position_thread = None
        self.position_queue = Queue()
        self.tracking_active = False
        
    def initialize(self) -> bool:
        """Initialize all connections"""
        logger.info("Initializing die scanner debug system...")
        
        # Connect to Arduino
        if not self.arduino.connect():
            logger.error("Failed to connect to Arduino")
            return False
            
        # Test robot connection
        try:
            self.robot.set_status(0)  # Set ready status
            logger.info("Robot connection verified")
        except Exception as e:
            logger.error(f"Robot connection failed: {e}")
            return False
            
        return True
    
    def pickup_sensor_jig(self):
        """Step 1: Pick up sensor jig from fixture"""
        logger.info("Step 1: Picking up sensor jig...")
        self.robot.set_status(1)  # Busy status
        
        # Set command to move to sensor jig pickup (handled by diescan_debug.ls)
        try:
            self.robot._write_parameter('ds_command', 1)  # Pickup command
            logger.info("Sensor jig pickup command sent to robot")
        except Exception as e:
            logger.error(f"Failed to send pickup command: {e}")
    
    def move_to_scan_start(self):
        """Step 2: Move to scan start position 65mm above static point"""
        logger.info("Step 2: Moving to scan start position...")
        
        try:
            self.robot._write_parameter('ds_command', 2)  # Move to start command
            logger.info("Move to scan start command sent")
        except Exception as e:
            logger.error(f"Failed to send move command: {e}")
    
    def calibrate_sensor(self) -> bool:
        """Step 3: Calibrate sensor at 65mm"""
        logger.info("Step 3: Calibrating sensor...")
        
        # Perform Arduino calibration
        if not self.arduino.calibrate(65.0):
            logger.error("Sensor calibration failed")
            return False
            
        # Verify we're at 65mm (±0.02mm)
        height = self.arduino.read_height()
        if abs(height - 65.0) > 0.02:
            logger.error(f"Calibration verification failed: {height:.3f}mm != 65.000mm")
            return False
            
        logger.info(f"Sensor calibrated and verified at {height:.3f}mm")
        return True
    
    def start_position_tracking(self):
        """Start background robot position tracking"""
        self.tracking_active = True
        self.position_thread = threading.Thread(target=self._position_tracking_worker)
        self.position_thread.daemon = True
        self.position_thread.start()
        logger.info("Background position tracking started")
    
    def stop_position_tracking(self):
        """Stop background position tracking"""
        self.tracking_active = False
        if self.position_thread:
            self.position_thread.join(timeout=1.0)
        logger.info("Background position tracking stopped")
    
    def _position_tracking_worker(self):
        """Background worker for robot position tracking (4ms intervals)"""
        while self.tracking_active:
            try:
                x, y, z = self.robot.read_robot_position()
                timestamp = time.time()
                
                # Add to queue (keep only recent positions)
                if self.position_queue.qsize() > 1000:  # Limit queue size
                    try:
                        self.position_queue.get_nowait()
                    except Empty:
                        pass
                        
                self.position_queue.put((x, y, z, timestamp))
                
            except Exception as e:
                logger.error(f"Position tracking error: {e}")
                
            time.sleep(0.004)  # 4ms = 250Hz update rate
    
    def start_continuous_scanning(self):
        """Step 5: Start continuous die edge scanning"""
        logger.info("Step 5: Starting continuous die edge scanning...")
        
        self.scanning_active = True
        self.scan_start_time = time.time()
        self.scan_points.clear()
        self.edge_points.clear()
        
        # Start position tracking
        self.start_position_tracking()
        
        # Signal robot to start scanning motion
        try:
            self.robot._write_parameter('ds_command', 5)  # Start scanning command
            logger.info("Continuous scanning started")
        except Exception as e:
            logger.error(f"Failed to start scanning: {e}")
    
    def continuous_scan_worker(self, duration: float = 30.0):
        """Main continuous scanning loop with robot motion coordination"""
        logger.info("Running continuous scan with robot motion coordination...")
        
        scan_count = 0
        last_height = None
        
        # Wait for robot to signal ready for scanning (status = 10)
        logger.info("Waiting for robot to be ready for scanning...")
        timeout = 10.0
        start_wait = time.time()
        while time.time() - start_wait < timeout:
            status = self.robot._read_parameter('ds_status')
            if status == 10:
                break
            time.sleep(0.1)
        else:
            logger.error("Robot did not signal ready for scanning")
            return
        
        # Initialize spiral pattern parameters  
        spiral_radius = 5.0  # mm
        spiral_angle = 0.0   # degrees
        spiral_step = 2.0    # mm per revolution
        max_radius = 60.0    # mm
        angle_increment = 15.0  # degrees per step
        
        # Get starting position (estimated die center)
        start_x, start_y, start_z = self.robot.read_robot_position()
        logger.info(f"Starting spiral scan from ({start_x:.1f}, {start_y:.1f}, {start_z:.1f})")
        
        start_time = time.time()
        
        while self.scanning_active and (time.time() - start_time) < duration:
            try:
                # Calculate next spiral position using Python
                angle_rad = np.radians(spiral_angle)
                offset_x = spiral_radius * np.cos(angle_rad)
                offset_y = spiral_radius * np.sin(angle_rad)
                
                next_x = start_x + offset_x
                next_y = start_y + offset_y
                next_z = start_z  # Keep same Z height
                
                # Write next position to numerical registers for robot to copy to PR[37]
                self.robot._write_parameter('ds_next_pos_x', next_x)
                self.robot._write_parameter('ds_next_pos_y', next_y) 
                self.robot._write_parameter('ds_next_pos_z', next_z)
                
                # Signal robot to move to new position (status = 11)
                self.robot._write_parameter('ds_status', 11)
                
                # Wait for robot to complete move (status = 10)
                move_timeout = 5.0
                move_start = time.time()
                while time.time() - move_start < move_timeout:
                    status = self.robot._read_parameter('ds_status')
                    if status == 10:
                        break
                    time.sleep(0.01)
                else:
                    logger.warning("Robot move timeout")
                
                # Take sensor reading at this position
                height = self.arduino.read_height()
                
                if height > -999:
                    # Create scan point
                    scan_point = ScanPoint(
                        x=next_x, y=next_y, z=next_z,
                        height=height,
                        is_valid=True,
                        timestamp=time.time()
                    )
                    
                    self.scan_points.append(scan_point)
                    scan_count += 1
                    
                    # Edge detection logic
                    if last_height is not None:
                        height_diff = height - last_height
                        
                        # Detect significant drop-offs (die edges)
                        if height_diff < -self.height_threshold:
                            edge_point = EdgePoint(
                                x=next_x, y=next_y,
                                edge_type='drop_off',
                                confidence=min(abs(height_diff) / self.height_threshold, 1.0)
                            )
                            self.edge_points.append(edge_point)
                            logger.info(f"Edge detected at ({next_x:.1f}, {next_y:.1f}): drop {height_diff:.1f}mm")
                    
                    last_height = height
                    
                    # Check if we have enough edge points for center calculation
                    if len(self.edge_points) >= 4:
                        estimated_center = self._estimate_center_from_edges()
                        if estimated_center:
                            # Check if we've found enough edges around the perimeter
                            edge_coverage = self._calculate_edge_coverage(estimated_center)
                            if edge_coverage > 0.6:  # 60% coverage
                                logger.info(f"Sufficient edge coverage ({edge_coverage:.1%}), ending scan")
                                # Signal robot scan complete (status = 12)
                                self.robot._write_parameter('ds_status', 12)
                                break
                
                # Update spiral parameters
                spiral_angle += angle_increment
                if spiral_angle >= 360:
                    spiral_angle = 0
                    spiral_radius += spiral_step
                
                # Check if we've reached maximum radius
                if spiral_radius > max_radius:
                    logger.info("Reached maximum scan radius")
                    break
                
            except Exception as e:
                logger.error(f"Scanning error: {e}")
                time.sleep(0.1)
        
        logger.info(f"Continuous scan completed: {scan_count} points, {len(self.edge_points)} edges")
        self.scanning_active = False
        
        # Signal robot scan complete
        try:
            self.robot._write_parameter('ds_status', 12)
        except Exception as e:
            logger.error(f"Failed to signal scan complete: {e}")
    
    def _get_current_position(self) -> Optional[Tuple[float, float, float, float]]:
        """Get most recent robot position from queue"""
        latest_pos = None
        
        # Drain queue to get most recent position
        try:
            while True:
                latest_pos = self.position_queue.get_nowait()
        except Empty:
            pass
            
        return latest_pos
    
    def _estimate_center_from_edges(self) -> Optional[Tuple[float, float]]:
        """Estimate die center from detected edge points"""
        if len(self.edge_points) < 3:
            return None
            
        # Use least squares circle fitting
        edge_coords = np.array([[ep.x, ep.y] for ep in self.edge_points])
        
        try:
            # Simple center estimation - average of edge points
            center_x = np.mean(edge_coords[:, 0])
            center_y = np.mean(edge_coords[:, 1])
            
            return center_x, center_y
            
        except Exception as e:
            logger.error(f"Center estimation failed: {e}")
            return None
    
    def _calculate_edge_coverage(self, center: Tuple[float, float]) -> float:
        """Calculate how much of the die perimeter we've covered"""
        if len(self.edge_points) < 2:
            return 0.0
            
        center_x, center_y = center
        
        # Calculate angles of edge points relative to center
        angles = []
        for edge_point in self.edge_points:
            dx = edge_point.x - center_x
            dy = edge_point.y - center_y
            angle = np.arctan2(dy, dx)
            angles.append(angle)
        
        angles = np.array(sorted(angles))
        
        # Calculate angular coverage
        if len(angles) < 2:
            return 0.0
            
        # Find largest gap in angles
        angle_diffs = np.diff(angles)
        largest_gap = np.max(angle_diffs)
        
        # Total coverage = 2π - largest_gap
        coverage = (2 * np.pi - largest_gap) / (2 * np.pi)
        
        return max(0.0, coverage)
    
    def calculate_final_center(self) -> Optional[DieCenter]:
        """Step 6: Calculate final die center from all measurements"""
        logger.info("Step 6: Calculating final die center...")
        
        if len(self.edge_points) < 4:
            logger.error("Insufficient edge points for center calculation")
            return None
        
        # Estimate center
        estimated_center = self._estimate_center_from_edges()
        if not estimated_center:
            return None
            
        center_x, center_y = estimated_center
        
        # Calculate average radius/diameter
        radii = []
        for edge_point in self.edge_points:
            dx = edge_point.x - center_x
            dy = edge_point.y - center_y
            radius = np.sqrt(dx*dx + dy*dy)
            radii.append(radius)
        
        avg_radius = np.mean(radii)
        diameter = avg_radius * 2
        
        # Calculate average die face height from scan points
        valid_heights = [sp.height for sp in self.scan_points if sp.is_valid and sp.height > 0]
        avg_height = np.mean(valid_heights) if valid_heights else 0.0
        
        # Calculate confidence based on:
        # 1. Number of edge points
        # 2. Consistency of radius measurements
        # 3. Proximity to expected die diameter
        
        radius_std = np.std(radii)
        diameter_error = abs(diameter - self.expected_die_diameter)
        
        # Confidence factors
        edge_count_factor = min(len(self.edge_points) / 8.0, 1.0)  # Up to 8 edges
        consistency_factor = max(0.0, 1.0 - (radius_std / avg_radius))
        diameter_factor = max(0.0, 1.0 - (diameter_error / self.die_tolerance))
        
        confidence = (edge_count_factor * 0.4 + consistency_factor * 0.4 + diameter_factor * 0.2) * 100
        
        result = DieCenter(
            center_x=center_x,
            center_y=center_y,
            diameter=diameter,
            avg_height=avg_height,
            confidence=confidence,
            edge_points=self.edge_points
        )
        
        logger.info(f"Die center calculated: ({center_x:.1f}, {center_y:.1f}), "
                   f"diameter: {diameter:.1f}mm, confidence: {confidence:.1f}%")
        
        return result
    
    def write_results_to_robot(self, die_center: DieCenter):
        """Step 6: Write calculated results to robot"""
        logger.info("Step 6: Writing results to robot...")
        
        self.robot.write_die_center(die_center)
        self.robot.set_status(2)  # Complete status
        
    def move_to_final_position(self, die_center: DieCenter):
        """Step 7: Move robot 100mm above found center"""
        logger.info("Step 7: Moving to final position...")
        
        try:
            # Set final position coordinates in robot
            self.robot._write_parameter('ds_command', 7)  # Move to final position
            logger.info(f"Moving 100mm above center at ({die_center.center_x:.1f}, {die_center.center_y:.1f})")
        except Exception as e:
            logger.error(f"Failed to move to final position: {e}")
    
    def run_debug_scan(self):
        """Run complete die scanning debug sequence"""
        logger.info("=== Starting Die Scanner Debug Sequence ===")
        
        if not self.initialize():
            logger.error("Initialization failed")
            return False
        
        try:
            # Step 1: Pick up sensor jig
            self.pickup_sensor_jig()
            time.sleep(2)
            
            # Step 2: Move to scan start
            self.move_to_scan_start()
            time.sleep(3)
            
            # Step 3: Calibrate sensor
            if not self.calibrate_sensor():
                logger.error("Calibration failed, aborting")
                return False
            
            # Step 4: Verify at 65mm
            height = self.arduino.read_height()
            logger.info(f"Step 4: Verified at {height:.3f}mm")
            
            # Step 5: Start continuous scanning
            self.start_continuous_scanning()
            
            # Run scanning worker
            self.continuous_scan_worker(duration=30.0)
            
            # Stop position tracking
            self.stop_position_tracking()
            
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
            return False
        finally:
            self.stop_position_tracking()
            self.arduino.disconnect()

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
        scanner.stop_position_tracking()
        scanner.arduino.disconnect()
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        scanner.stop_position_tracking()
        scanner.arduino.disconnect()

if __name__ == "__main__":
    main()