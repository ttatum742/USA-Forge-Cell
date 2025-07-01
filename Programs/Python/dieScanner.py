# Ultrasonic Die Scanner - PC Controller
# One-time die scanning at job start for FANUC robot integration

from cpppo.server.enip.get_attribute import proxy_simple
import numpy as np
import time
import logging
import serial
import matplotlib.pyplot as plt
from typing import List, Optional, Tuple
from scipy import ndimage

# Configure logging
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class UltrasonicDieScanner(proxy_simple):
    """One-time die scanner for job setup using PC control and EtherNet/IP"""
    
    PARAMETERS = dict(proxy_simple.PARAMETERS,
        # Scan control registers
        scan_trigger=proxy_simple.parameter('@0x6B/1/100', 'DINT', 'Bool'),
        scan_complete=proxy_simple.parameter('@0x6B/1/101', 'DINT', 'Bool'),
        scan_progress=proxy_simple.parameter('@0x6B/1/102', 'DINT', 'Percent'),
        
        # Scan configuration
        scan_area_size=proxy_simple.parameter('@0x6B/1/110', 'DINT', 'mm'),
        scan_grid_size=proxy_simple.parameter('@0x6B/1/111', 'DINT', 'points'),
        scan_height=proxy_simple.parameter('@0x6B/1/112', 'DINT', 'mm'),
        
        # Current measurement position
        current_scan_x=proxy_simple.parameter('@0x6B/1/115', 'DINT', 'mm'),
        current_scan_y=proxy_simple.parameter('@0x6B/1/116', 'DINT', 'mm'),
        measurement_ready=proxy_simple.parameter('@0x6B/1/117', 'DINT', 'Bool'),
        
        # Distance measurement result
        measured_distance=proxy_simple.parameter('@0x6B/1/120', 'DINT', 'mm'),
        measurement_valid=proxy_simple.parameter('@0x6B/1/121', 'DINT', 'Bool'),
        
        # Die center results
        die_center_x=proxy_simple.parameter('@0x6B/1/125', 'DINT', 'mm'),
        die_center_y=proxy_simple.parameter('@0x6B/1/126', 'DINT', 'mm'),
        
        # Face measurements
        die_face_center_x=proxy_simple.parameter('@0x6B/1/140', 'DINT', 'mm'),
        die_face_center_y=proxy_simple.parameter('@0x6B/1/141', 'DINT', 'mm'),
        die_face_height=proxy_simple.parameter('@0x6B/1/142', 'DINT', 'mm'),
        die_face_diameter=proxy_simple.parameter('@0x6B/1/143', 'DINT', 'mm'),
        
        # Quality metrics
        face_quality=proxy_simple.parameter('@0x6B/1/145', 'DINT', 'Percent'),
        scan_confidence=proxy_simple.parameter('@0x6B/1/129', 'DINT', 'Percent'),
        
        # Mounting surface reference
        mounting_surface_height=proxy_simple.parameter('@0x6B/1/149', 'DINT', 'mm'),
    )

    def __init__(self, ip_address: str, ultrasonic_port: str = None):
        super().__init__(host=ip_address)
        self.ip_address = ip_address
        
        # Initialize ultrasonic sensor
        self.ultrasonic_port = ultrasonic_port
        self.ultrasonic_sensor = None
        if ultrasonic_port:
            self._init_ultrasonic_sensor()
        
        # Scan data storage
        self.scan_data = []
        self.current_scan_config = {
            'area_size': 60,  # mm
            'grid_size': 13,  # points per axis
            'height': 50,     # mm above surface
        }
        
        logger.info("Die scanner initialized for job setup")
        print("Die scanner initialized for job setup")

    def _init_ultrasonic_sensor(self):
        """Initialize ultrasonic sensor connection"""
        try:
            if self.ultrasonic_port.upper().startswith('COM') or self.ultrasonic_port.startswith('/dev/'):
                self.ultrasonic_sensor = serial.Serial(self.ultrasonic_port, 115200, timeout=2)
                time.sleep(2)  # Allow Arduino to reset
                print(f"Connected to ultrasonic sensor on {self.ultrasonic_port}")
            else:
                print("USB ultrasonic sensor connection not implemented yet")
                
        except Exception as e:
            logger.error(f"Failed to initialize ultrasonic sensor: {e}")
            self.ultrasonic_sensor = None

    def _read_parameter(self, param_name: str) -> Optional[float]:
        """Read a single parameter value"""
        try:
            param_str = self.parameter_substitution(param_name)
            result, = self.read(param_str, checking=True)
            return float(result[0]) if isinstance(result, list) else float(result)
        except Exception as e:
            logger.error(f"Failed to read {param_name}: {str(e)}")
            return None

    def _write_parameter(self, param_name: str, value: float) -> bool:
        """Write a single parameter value"""
        try:
            param = '%s = (DINT) %s' % (param_name, int(value))
            success, = self.write(
                self.parameter_substitution(param), checking=True)
            if success:
                logger.debug(f"Wrote {param_name}: {value}")
            return success
        except Exception as e:
            logger.error(f"Failed to write {param_name}: {e}")
            return False

    def take_ultrasonic_measurement(self) -> Optional[float]:
        """Take a single ultrasonic measurement"""
        if not self.ultrasonic_sensor:
            # Simulate measurement for testing
            return 100.0 + np.random.normal(0, 2)
        
        try:
            if hasattr(self.ultrasonic_sensor, 'write'):
                self.ultrasonic_sensor.write(b'M')
                response = self.ultrasonic_sensor.readline().decode().strip()
                distance = float(response)
                return distance if distance > 0 else None
        except Exception as e:
            logger.error(f"Ultrasonic measurement failed: {e}")
            return None

    def run_die_scan(self):
        """Main die scanning service - waits for job setup triggers"""
        try:
            print("Die scanner ready - waiting for job setup requests...")
            
            while True:
                # Check if robot has triggered a scan
                scan_trigger = self._read_parameter('scan_trigger')
                if scan_trigger and scan_trigger > 0:
                    print("Die scan triggered for job setup")
                    
                    # Read scan configuration from robot
                    self._read_scan_configuration()
                    
                    # Execute the scan
                    success = self._execute_coordinated_scan()
                    
                    if success:
                        print("Job setup scan completed successfully")
                    else:
                        print("Job setup scan failed")
                    
                    # Signal completion to robot
                    self._write_parameter('scan_complete', 1)
                    
                    # Wait for robot to reset trigger
                    while self._read_parameter('scan_trigger') > 0:
                        time.sleep(0.1)
                    
                    # Reset completion flag
                    self._write_parameter('scan_complete', 0)
                    
                    print("Ready for next job setup request")
                
                time.sleep(0.5)  # Check every 500ms for job setup requests
                
        except KeyboardInterrupt:
            print("\nStopping die scanner...")
        except Exception as e:
            logger.error(f"Scanner error: {e}")

    def _read_scan_configuration(self):
        """Read scan parameters from robot"""
        area_size = self._read_parameter('scan_area_size')
        grid_size = self._read_parameter('scan_grid_size')
        height = self._read_parameter('scan_height')
        
        if area_size: self.current_scan_config['area_size'] = area_size
        if grid_size: self.current_scan_config['grid_size'] = int(grid_size)
        if height: self.current_scan_config['height'] = height
        
        print(f"Scan config: {self.current_scan_config['area_size']}mm area, "
              f"{self.current_scan_config['grid_size']}x{self.current_scan_config['grid_size']} grid")

    def _execute_coordinated_scan(self) -> bool:
        """Execute scan in coordination with robot motion"""
        try:
            grid_size = self.current_scan_config['grid_size']
            area_size = self.current_scan_config['area_size']
            step_size = area_size / grid_size
            
            # Initialize scan data array
            self.scan_data = np.zeros((grid_size, grid_size))
            total_points = grid_size * grid_size
            completed_points = 0
            
            print(f"Starting {grid_size}x{grid_size} scan...")
            
            # For each scan point
            for x_index in range(grid_size):
                for y_index in range(grid_size):
                    # Calculate position offset from center
                    x_offset = (x_index - grid_size//2) * step_size
                    y_offset = (y_index - grid_size//2) * step_size
                    
                    # Tell robot where to position
                    self._write_parameter('current_scan_x', x_offset)
                    self._write_parameter('current_scan_y', y_offset)
                    self._write_parameter('measurement_ready', 1)
                    
                    # Wait for robot to position and signal ready
                    timeout = time.time() + 10  # 10 second timeout
                    while self._read_parameter('measurement_ready') > 0:
                        if time.time() > timeout:
                            print("Robot positioning timeout")
                            return False
                        time.sleep(0.05)
                    
                    # Take measurement
                    distance = self.take_ultrasonic_measurement()
                    
                    if distance is not None:
                        self.scan_data[x_index, y_index] = distance
                        # Send measurement back to robot
                        self._write_parameter('measured_distance', distance)
                        self._write_parameter('measurement_valid', 1)
                    else:
                        self._write_parameter('measurement_valid', 0)
                    
                    completed_points += 1
                    progress = int((completed_points / total_points) * 100)
                    self._write_parameter('scan_progress', progress)
                    
                    if completed_points % 20 == 0:
                        print(f"Scan progress: {progress}%")
            
            # Process scan data
            return self._process_scan_results()
            
        except Exception as e:
            logger.error(f"Scan execution failed: {e}")
            return False

    def _process_scan_results(self) -> bool:
        """Process scan data to find die center and face height"""
        try:
            print("Processing scan data...")
            
            # Find baseline using edge points
            baseline = self._calculate_baseline()
            depth_map = baseline - self.scan_data
            
            # Analyze die face geometry
            face_analysis = self._analyze_die_face_geometry(depth_map, baseline)
            
            if not face_analysis['valid']:
                print("Failed to detect die face geometry")
                return False
            
            # Calculate die center and face measurements
            die_center_result = self._calculate_die_center_from_face_analysis(face_analysis)
            
            # Send results to robot
            self._send_die_results(die_center_result, face_analysis)
            
            # Save visualization
            self._save_scan_visualization(die_center_result, face_analysis)
            
            return True
            
        except Exception as e:
            logger.error(f"Scan processing failed: {e}")
            return False

    def _calculate_baseline(self) -> float:
        """Calculate baseline using edge points"""
        grid_size = self.scan_data.shape[0]
        edge_points = []
        
        # Use outer edge points for baseline
        for x in range(grid_size):
            for y in range(grid_size):
                if x < 2 or x >= grid_size-2 or y < 2 or y >= grid_size-2:
                    if self.scan_data[x, y] > 0:
                        edge_points.append(self.scan_data[x, y])
        
        return np.mean(edge_points) if edge_points else 100

    def _analyze_die_face_geometry(self, depth_map: np.ndarray, baseline: float) -> dict:
        """Analyze die geometry to find face height and opening"""
        
        # Create depth masks for different regions
        masks = self._create_depth_masks(depth_map)
        
        # Analyze each depth region
        regions = {
            'mounting_surface': self._analyze_mounting_surface(masks['surface'], depth_map, baseline),
            'die_face': self._analyze_die_face_region(masks['face'], depth_map, baseline),
            'die_opening': self._analyze_opening_region(masks['opening'], depth_map, baseline)
        }
        
        # Validate die structure
        if not all(regions[r]['valid'] for r in ['mounting_surface', 'die_face']):
            return {'valid': False, 'reason': 'Invalid die structure'}
        
        # Calculate face height relative to mounting surface
        face_height = self._calculate_face_height_from_mounting_surface(regions)
        
        return {
            'valid': True,
            'regions': regions,
            'face_height': face_height,
            'face_center': regions['die_face']['center'],
            'mounting_surface_height': regions['mounting_surface']['height_from_baseline']
        }

    def _create_depth_masks(self, depth_map: np.ndarray) -> dict:
        """Create masks for different depth regions"""
        
        mounting_threshold = 3   # mm - mounting surface level
        face_threshold = 12      # mm - die face level
        opening_threshold = 25   # mm - die opening level
        
        masks = {
            'surface': depth_map <= mounting_threshold,
            'face': (depth_map > mounting_threshold) & (depth_map <= face_threshold),
            'opening': depth_map > opening_threshold
        }
        
        # Clean up masks
        for region_name, mask in masks.items():
            masks[region_name] = ndimage.binary_opening(mask, structure=np.ones((3,3)))
            masks[region_name] = ndimage.binary_closing(masks[region_name], structure=np.ones((3,3)))
        
        return masks

    def _analyze_mounting_surface(self, surface_mask: np.ndarray, depth_map: np.ndarray, baseline: float) -> dict:
        """Analyze mounting surface (furthest from sensor)"""
        
        if np.sum(surface_mask) < 10:
            return {'valid': False, 'reason': 'Insufficient mounting surface points'}
        
        surface_points = np.where(surface_mask)
        surface_depths = depth_map[surface_mask]
        
        surface_center_x = np.mean(surface_points[1])
        surface_center_y = np.mean(surface_points[0])
        surface_depth_mean = np.mean(surface_depths)
        
        mounting_surface_height = baseline - surface_depth_mean
        
        return {
            'valid': True,
            'center': (surface_center_x, surface_center_y),
            'height_from_baseline': mounting_surface_height,
            'depth_mean': surface_depth_mean
        }

    def _analyze_die_face_region(self, face_mask: np.ndarray, depth_map: np.ndarray, baseline: float) -> dict:
        """Analyze die face region (pickup surface)"""
        
        if np.sum(face_mask) < 8:
            return {'valid': False, 'reason': 'Insufficient die face points'}
        
        face_points = np.where(face_mask)
        face_depths = depth_map[face_mask]
        
        face_center_x = np.mean(face_points[1])
        face_center_y = np.mean(face_points[0])
        face_depth_mean = np.mean(face_depths)
        
        die_face_height = baseline - face_depth_mean
        
        # Calculate face diameter
        face_area = np.sum(face_mask)
        step_size = self.current_scan_config['area_size'] / self.current_scan_config['grid_size']
        face_diameter = 2 * np.sqrt(face_area * (step_size ** 2) / np.pi)
        
        return {
            'valid': True,
            'center': (face_center_x, face_center_y),
            'area': face_area,
            'diameter': face_diameter,
            'height_from_baseline': die_face_height,
            'depth_mean': face_depth_mean
        }

    def _analyze_opening_region(self, opening_mask: np.ndarray, depth_map: np.ndarray, baseline: float) -> dict:
        """Analyze die opening (deepest cavity)"""
        
        if np.sum(opening_mask) < 4:
            return {'valid': False, 'reason': 'No die opening detected'}
        
        opening_points = np.where(opening_mask)
        opening_center_x = np.mean(opening_points[1])
        opening_center_y = np.mean(opening_points[0])
        
        return {
            'valid': True,
            'center': (opening_center_x, opening_center_y)
        }

    def _calculate_face_height_from_mounting_surface(self, regions: dict) -> float:
        """Calculate die face height above mounting surface"""
        
        mounting_surface_height = regions['mounting_surface']['height_from_baseline']
        die_face_height = regions['die_face']['height_from_baseline']
        
        # Face height = mounting surface distance - die face distance
        face_height = mounting_surface_height - die_face_height
        
        return max(0, face_height)

    def _calculate_die_center_from_face_analysis(self, face_analysis: dict) -> dict:
        """Calculate die center from face analysis"""
        
        face_center = face_analysis['face_center']
        
        return {
            'center_x': face_center[0],
            'center_y': face_center[1],
            'confidence': 85  # Default confidence
        }

    def _send_die_results(self, die_center_result: dict, face_analysis: dict):
        """Send die analysis results to robot"""
        
        # Convert grid coordinates to mm offsets
        grid_size = self.current_scan_config['grid_size']
        step_size = self.current_scan_config['area_size'] / grid_size
        
        # Die center
        center_x_mm = (die_center_result['center_x'] - grid_size//2) * step_size
        center_y_mm = (die_center_result['center_y'] - grid_size//2) * step_size
        
        # Face center
        face_center = face_analysis['face_center']
        face_center_x_mm = (face_center[0] - grid_size//2) * step_size
        face_center_y_mm = (face_center[1] - grid_size//2) * step_size
        
        # Send results to robot
        self._write_parameter('die_center_x', center_x_mm)
        self._write_parameter('die_center_y', center_y_mm)
        self._write_parameter('die_face_center_x', face_center_x_mm)
        self._write_parameter('die_face_center_y', face_center_y_mm)
        self._write_parameter('die_face_height', face_analysis['face_height'])
        self._write_parameter('die_face_diameter', face_analysis['regions']['die_face']['diameter'])
        self._write_parameter('mounting_surface_height', face_analysis['mounting_surface_height'])
        self._write_parameter('scan_confidence', die_center_result['confidence'])
        self._write_parameter('face_quality', 85)  # Default quality
        
        print(f"Job Setup - Die Analysis Results:")
        print(f"  Die Center: ({center_x_mm:.1f}, {center_y_mm:.1f}) mm")
        print(f"  Face Center: ({face_center_x_mm:.1f}, {face_center_y_mm:.1f}) mm")
        print(f"  Face Height: {face_analysis['face_height']:.1f} mm")
        print(f"  Face Diameter: {face_analysis['regions']['die_face']['diameter']:.1f} mm")
        print("These values will be used for entire production job")

    def _save_scan_visualization(self, die_center_result: dict, face_analysis: dict):
        """Save scan visualization for debugging"""
        try:
            plt.figure(figsize=(10, 8))
            plt.imshow(self.scan_data, cmap='viridis', origin='lower')
            plt.colorbar(label='Distance (mm)')
            plt.title('Ultrasonic Die Scan Results')
            
            # Mark detected centers
            grid_size = self.current_scan_config['grid_size']
            
            die_center = die_center_result
            plt.plot(die_center['center_x'], die_center['center_y'], 'r+', markersize=15, markeredgewidth=3, label='Die Center')
            
            face_center = face_analysis['face_center']
            plt.plot(face_center[0], face_center[1], 'b+', markersize=15, markeredgewidth=3, label='Face Center')
            
            plt.xlabel('X Grid Position')
            plt.ylabel('Y Grid Position')
            plt.legend()
            
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            plt.savefig(f'job_setup_scan_{timestamp}.png', dpi=150, bbox_inches='tight')
            plt.close()
            
        except Exception as e:
            logger.warning(f"Could not save scan visualization: {e}")

if __name__ == "__main__":
    # Configuration
    ROBOT_IP = "192.168.0.1"
    ULTRASONIC_PORT = "COM3"  # Adjust for your Arduino port
    
    try:
        # Initialize scanner
        scanner = UltrasonicDieScanner(ip_address=ROBOT_IP, ultrasonic_port=ULTRASONIC_PORT)
        
        # Start die scanning service for job setup
        scanner.run_die_scan()
        
    except Exception as e:
        logger.error(f"Failed to start scanner: {e}")
        print(f"Scanner startup failed: {e}")