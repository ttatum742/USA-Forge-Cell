from cpppo.server.enip.get_attribute import proxy_simple
import numpy as np
import time
import logging
import serial
from typing import Tuple, List, Optional, Dict
from dataclasses import dataclass
import math
from sklearn.cluster import KMeans
import warnings
warnings.filterwarnings('ignore')

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class MeasurementPoint:
    """Data structure for individual measurement points"""
    x: float
    y: float
    z: float  # Robot Z position
    height: float  # Laser measurement (-999 if out of range)
    is_valid: bool

@dataclass
class DieResults:
    """Results of die measurement analysis"""
    center_x: float
    center_y: float
    average_height: float
    confidence: float
    center_area_points: List[MeasurementPoint]
    die_face_points: List[MeasurementPoint]

@dataclass
class WellCenterResult:
    """Results of well center analysis"""
    valid: bool
    center_x: float = 0.0
    center_y: float = 0.0
    well_diameter: float = 0.0
    confidence: float = 0.0
    edge_points: List[MeasurementPoint] = None
    reason: str = ""

@dataclass
class ZoneClassification:
    """Classification of measurement zones"""
    die_face: List[MeasurementPoint]
    central_well: List[MeasurementPoint]
    outer_edges: List[MeasurementPoint]
    other_valid: List[MeasurementPoint]
    debris: List[MeasurementPoint]
    die_face_height: float

class KeyenceDieMeasurement(proxy_simple):
    """Keyence laser die measurement system with FANUC communication"""
    
    PARAMETERS = dict(proxy_simple.PARAMETERS,
        # Communication registers (150-160)
        command=proxy_simple.parameter('@0x6B/1/150', 'DINT', ''),     # 1=start, 2=zone_map, 3=well_detect, 4=validate
        status=proxy_simple.parameter('@0x6B/1/151', 'DINT', ''),      # 0=ready, 1=busy, 10=zone_done, 20=well_done, 2=complete, 3=error
        current_x=proxy_simple.parameter('@0x6B/1/152', 'DINT', 'mm'), # Current X position
        current_y=proxy_simple.parameter('@0x6B/1/153', 'DINT', 'mm'), # Current Y position  
        current_z=proxy_simple.parameter('@0x6B/1/154', 'DINT', 'mm'), # Current Z position
        point_counter=proxy_simple.parameter('@0x6B/1/155', 'DINT', ''), # Measurement point counter
        total_points=proxy_simple.parameter('@0x6B/1/156', 'DINT', ''),  # Total points planned
        
        # Enhanced measurement control
        measure_trigger=proxy_simple.parameter('@0x6B/1/401', 'DINT', ''), # Trigger measurement
        measure_complete=proxy_simple.parameter('@0x6B/1/402', 'DINT', ''), # Measurement done
        laser_reading=proxy_simple.parameter('@0x6B/1/403', 'DINT', 'mm'), # Laser result
        
        # Results registers (157-160)
        center_x=proxy_simple.parameter('@0x6B/1/157', 'DINT', 'mm'),   # Die center X result
        center_y=proxy_simple.parameter('@0x6B/1/158', 'DINT', 'mm'),   # Die center Y result  
        avg_height=proxy_simple.parameter('@0x6B/1/159', 'DINT', 'mm'), # Average die height
        confidence=proxy_simple.parameter('@0x6B/1/160', 'DINT', '%'),  # Confidence level (0-100)
        
        # Note: laser_height now read from Arduino USB instead of robot register
    )

    def __init__(self, ip_address: str, arduino_port: str = "COM3"):
        super().__init__(host=ip_address)
        self.ip_address = ip_address
        self.arduino_port = arduino_port
        self.arduino = None
        
        # Initialize Arduino connection
        self._init_arduino_connection()
        
        # Measurement data storage
        self.measurement_points: List[MeasurementPoint] = []
        self.zone_classification: Optional[ZoneClassification] = None
        self.well_result: Optional[WellCenterResult] = None
        
        # Enhanced die measurement parameters
        self.out_of_range_value = -999
        self.min_confidence_threshold = 70
        self.die_face_height_range = (40, 80)  # Expected die face height range
        self.well_radius_range = (8, 20)       # Expected well radius range
        self.height_tolerance = 3.0             # Height consistency tolerance
        
        logger.info("Enhanced Keyence die measurement system initialized with Arduino communication")

    def _read_parameter(self, param_name: str) -> Optional[float]:
        """Read a single parameter value from robot"""
        try:
            param_str = self.parameter_substitution(param_name)
            result, = self.read(param_str, checking=True)
            return float(result[0]) if isinstance(result, list) else float(result)
        except Exception as e:
            logger.error(f"Failed to read {param_name}: {str(e)}")
            return None

    def _write_parameter(self, param_name: str, value: float) -> bool:
        """Write a single parameter value to robot"""
        try:
            param = f'{param_name} = (DINT) {int(value)}'
            success, = self.write(self.parameter_substitution(param), checking=True)
            return success
        except Exception as e:
            logger.error(f"Failed to write {param_name}: {str(e)}")
            return False

    def _init_arduino_connection(self):
        """Initialize Arduino USB connection"""
        try:
            self.arduino = serial.Serial(self.arduino_port, 115200, timeout=2)
            time.sleep(2)  # Allow Arduino to reset
            
            # Turn on sensor power
            self.arduino.write(b'POWER_ON\n')
            response = self.arduino.readline().decode().strip()
            
            if "POWER_ON_OK" in response:
                logger.info(f"Arduino connected on {self.arduino_port}, sensor powered on")
            else:
                logger.warning(f"Arduino connected but unexpected response: {response}")
                
        except Exception as e:
            logger.error(f"Failed to connect to Arduino on {self.arduino_port}: {e}")
            self.arduino = None

    def _read_laser_measurement(self) -> float:
        """Read current laser height measurement from Arduino"""
        if not self.arduino:
            logger.error("Arduino not connected")
            return self.out_of_range_value
            
        try:
            # Request measurement from Arduino
            self.arduino.write(b'READ\n')
            response = self.arduino.readline().decode().strip()
            
            if response.startswith('HEIGHT:'):
                height_str = response.replace('HEIGHT:', '')
                height = float(height_str)
                return height if height >= 0 else self.out_of_range_value
            else:
                logger.warning(f"Unexpected Arduino response: {response}")
                return self.out_of_range_value
                
        except Exception as e:
            logger.error(f"Failed to read from Arduino: {e}")
            return self.out_of_range_value

    def _wait_for_robot_command(self, expected_command: int, timeout: float = 30.0) -> bool:
        """Wait for robot to send specific command"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            command = self._read_parameter('command')
            if command == expected_command:
                return True
            time.sleep(0.1)
        logger.error(f"Timeout waiting for robot command {expected_command}")
        return False

    def _get_robot_position(self) -> Tuple[float, float, float]:
        """Get current robot position from communication registers"""
        x = self._read_parameter('current_x')
        y = self._read_parameter('current_y') 
        z = self._read_parameter('current_z')
        return x or 0, y or 0, z or 0

    def collect_grid_measurements(self) -> bool:
        """Collect measurements during grid scan phase"""
        logger.info("Starting grid measurement collection...")
        self.measurement_points.clear()
        
        # Wait for robot to start grid scan
        if not self._wait_for_robot_command(2):  # Command 2 = grid scan
            return False
            
        self._write_parameter('status', 1)  # Set status to busy
        
        expected_points = self._read_parameter('total_points') or 81  # 9x9 grid default
        
        while len(self.measurement_points) < expected_points:
            # Check if robot has moved to new position
            current_counter = self._read_parameter('point_counter') or 0
            
            if current_counter > len(self.measurement_points):
                # Robot is at new measurement point
                x, y, z = self._get_robot_position()
                height = self._read_laser_measurement()
                
                point = MeasurementPoint(
                    x=x, y=y, z=z,
                    height=height,
                    is_valid=(height != self.out_of_range_value)
                )
                
                self.measurement_points.append(point)
                logger.debug(f"Point {len(self.measurement_points)}: ({x:.1f}, {y:.1f}) = {height:.1f}mm")
                
            time.sleep(0.05)  # Fast polling during measurement
            
        logger.info(f"Grid measurement complete: {len(self.measurement_points)} points collected")
        self.grid_complete = True
        return True

    def analyze_grid_data(self) -> Optional[Tuple[float, float]]:
        """Analyze grid data to estimate die center location"""
        if not self.grid_complete or not self.measurement_points:
            logger.error("No grid data available for analysis")
            return None
            
        # Separate valid measurements from out-of-range
        valid_points = [p for p in self.measurement_points if p.is_valid]
        invalid_points = [p for p in self.measurement_points if not p.is_valid]
        
        logger.info(f"Valid measurements: {len(valid_points)}, Out-of-range: {len(invalid_points)}")
        
        if len(invalid_points) < 3:
            logger.warning("Very few out-of-range points detected - may not be a typical die")
            
        # Method 1: Use centroid of out-of-range points (these should be in center area)
        if len(invalid_points) >= 3:
            center_x = np.mean([p.x for p in invalid_points])
            center_y = np.mean([p.y for p in invalid_points])
            logger.info(f"Center estimate from out-of-range points: ({center_x:.1f}, {center_y:.1f})")
        else:
            # Method 2: Find lowest height region among valid points
            if len(valid_points) >= 5:
                heights = [p.height for p in valid_points]
                min_height = min(heights)
                low_points = [p for p in valid_points if p.height <= min_height + 2.0]  # Within 2mm of minimum
                
                center_x = np.mean([p.x for p in low_points])
                center_y = np.mean([p.y for p in low_points])
                logger.info(f"Center estimate from low height region: ({center_x:.1f}, {center_y:.1f})")
            else:
                logger.error("Insufficient data for center estimation")
                return None
        
        # Store estimated center and send to robot
        self.estimated_center = (center_x, center_y)
        self._write_parameter('center_x', center_x)
        self._write_parameter('center_y', center_y)
        self._write_parameter('command', 3)  # Signal robot to start confirmation
        
        return self.estimated_center

    def collect_confirmation_measurements(self) -> bool:
        """Collect measurements during confirmation scan"""
        logger.info("Starting confirmation measurement collection...")
        
        if not self.estimated_center:
            logger.error("No estimated center available for confirmation")
            return False
            
        confirmation_points = []
        expected_points = 8  # Radial confirmation points
        
        while len(confirmation_points) < expected_points:
            current_counter = self._read_parameter('point_counter') or 0
            total_measurements = len(self.measurement_points) + len(confirmation_points)
            
            if current_counter > total_measurements:
                x, y, z = self._get_robot_position()
                height = self._read_laser_measurement()
                
                point = MeasurementPoint(
                    x=x, y=y, z=z,
                    height=height,
                    is_valid=(height != self.out_of_range_value)
                )
                
                confirmation_points.append(point)
                logger.debug(f"Confirmation point {len(confirmation_points)}: ({x:.1f}, {y:.1f}) = {height:.1f}mm")
                
            time.sleep(0.05)
            
        # Add confirmation points to main dataset
        self.measurement_points.extend(confirmation_points)
        logger.info(f"Confirmation complete: {len(confirmation_points)} additional points")
        return True

    def calculate_final_results(self) -> Optional[DieResults]:
        """Calculate final die center and confidence metrics"""
        if not self.measurement_points:
            logger.error("No measurement data available")
            return None
            
        # Separate points by type
        valid_points = [p for p in self.measurement_points if p.is_valid]
        invalid_points = [p for p in self.measurement_points if not p.is_valid]
        
        if len(valid_points) < 5:
            logger.error("Insufficient valid measurements for analysis")
            return None
            
        # Calculate die face average height (valid measurements)
        face_heights = [p.height for p in valid_points]
        avg_face_height = np.mean(face_heights)
        
        # Refined center calculation using both grid and confirmation data
        if len(invalid_points) >= 3:
            # Use centroid of out-of-range points
            center_x = np.mean([p.x for p in invalid_points])
            center_y = np.mean([p.y for p in invalid_points])
            
            # Calculate confidence based on consistency of out-of-range points
            distances = [math.sqrt((p.x - center_x)**2 + (p.y - center_y)**2) for p in invalid_points]
            max_distance = max(distances)
            confidence = max(0, 100 - (max_distance * 2))  # Confidence decreases with spread
            
        else:
            # Fallback: use lowest height region
            min_height = min(face_heights)
            low_threshold = min_height + self.die_face_height_threshold
            low_points = [p for p in valid_points if p.height <= low_threshold]
            
            if len(low_points) >= 3:
                center_x = np.mean([p.x for p in low_points])
                center_y = np.mean([p.y for p in low_points])
                confidence = min(80, len(low_points) * 10)  # Lower confidence for this method
            else:
                logger.error("Cannot determine die center with sufficient confidence")
                return None
                
        # Additional confidence factors
        total_measurements = len(self.measurement_points)
        valid_percentage = len(valid_points) / total_measurements * 100
        
        # Adjust confidence based on measurement quality
        if valid_percentage < 30:
            confidence *= 0.7  # Reduce confidence if too few valid measurements
        elif valid_percentage > 80:
            confidence *= 0.8  # Reduce confidence if too many valid measurements (unusual for die)
            
        confidence = min(100, max(0, confidence))
        
        results = DieResults(
            center_x=center_x,
            center_y=center_y,
            average_height=avg_face_height,
            confidence=confidence,
            center_area_points=invalid_points,
            die_face_points=valid_points
        )
        
        logger.info(f"Final results: Center=({center_x:.1f}, {center_y:.1f}), "
                   f"Height={avg_face_height:.1f}mm, Confidence={confidence:.1f}%")
        
        return results

    def send_results_to_robot(self, results: DieResults) -> bool:
        """Send final results back to robot"""
        try:
            self._write_parameter('center_x', results.center_x)
            self._write_parameter('center_y', results.center_y)
            self._write_parameter('avg_height', results.average_height)
            self._write_parameter('confidence', results.confidence)
            
            # Set final status
            if results.confidence >= self.min_confidence_threshold:
                self._write_parameter('status', 2)  # Complete
                logger.info("Results sent to robot - measurement successful")
            else:
                self._write_parameter('status', 3)  # Error - low confidence
                logger.warning(f"Low confidence result: {results.confidence:.1f}%")
                
            return True
            
        except Exception as e:
            logger.error(f"Failed to send results to robot: {str(e)}")
            self._write_parameter('status', 3)  # Error
            return False

    def run_enhanced_measurement_cycle(self) -> Optional[WellCenterResult]:
        """Execute enhanced 3-phase measurement cycle"""
        logger.info("Starting enhanced die measurement cycle...")
        
        try:
            # Phase 1: Zone Mapping
            logger.info("Phase 1: Zone mapping...")
            if not self._wait_for_robot_command(1):  # Command 1 = start
                logger.error("Timeout waiting for start command")
                return None
                
            if not self.execute_zone_mapping():
                logger.error("Zone mapping failed")
                self._write_parameter('status', 3)  # Error
                return None
                
            # Phase 2: Smart Well Detection
            logger.info("Phase 2: Well edge detection...")
            if not self.execute_well_detection():
                logger.error("Well detection failed")
                self._write_parameter('status', 3)  # Error
                return None
                
            # Phase 3: Edge Validation
            logger.info("Phase 3: Edge validation...")
            well_result = self.execute_edge_validation()
            if not well_result or not well_result.valid:
                logger.error("Edge validation failed")
                self._write_parameter('status', 3)  # Error
                return None
                
            # Send final results to robot
            if not self.send_well_results_to_robot(well_result):
                logger.error("Failed to send results to robot")
                return None
                
            logger.info(f"Enhanced measurement cycle completed - Confidence: {well_result.confidence:.1f}%")
            return well_result
            
        except Exception as e:
            logger.error(f"Enhanced measurement cycle failed: {str(e)}")
            self._write_parameter('status', 3)  # Error
            return None

    def execute_zone_mapping(self) -> bool:
        """Execute Phase 1: Zone mapping scan"""
        logger.info("Executing zone mapping scan...")
        
        # Wait for robot to complete zone mapping
        while True:
            status = self._read_parameter('status')
            if status == 10:  # Zone mapping complete
                break
            elif status == 3:  # Error
                return False
            
            # Check for measurement trigger
            trigger = self._read_parameter('measure_trigger')
            if trigger and trigger > 0:
                # Take measurement and send result back
                height = self._read_laser_measurement()
                self._write_parameter('laser_reading', height)
                self._write_parameter('measure_complete', 1)
                
                # Store measurement
                x, y, z = self._get_robot_position()
                point = MeasurementPoint(
                    x=x, y=y, z=z,
                    height=height,
                    is_valid=(height != self.out_of_range_value)
                )
                self.measurement_points.append(point)
                
            time.sleep(0.02)  # Fast polling
            
        # Classify measurements into zones
        self.zone_classification = self._classify_measurement_zones(self.measurement_points)
        logger.info(f"Zone mapping complete: {len(self.measurement_points)} points classified")
        return True
        
    def execute_well_detection(self) -> bool:
        """Execute Phase 2: Smart well edge detection"""
        logger.info("Executing well edge detection...")
        
        # Continue measuring during well detection phase
        well_edge_points = []
        
        while True:
            status = self._read_parameter('status')
            if status == 20:  # Well detection complete
                break
            elif status == 3:  # Error
                return False
            
            # Handle measurement requests
            trigger = self._read_parameter('measure_trigger')
            if trigger and trigger > 0:
                height = self._read_laser_measurement()
                self._write_parameter('laser_reading', height)
                self._write_parameter('measure_complete', 1)
                
                # Store well edge measurement
                x, y, z = self._get_robot_position()
                point = MeasurementPoint(
                    x=x, y=y, z=z,
                    height=height,
                    is_valid=(height != self.out_of_range_value)
                )
                well_edge_points.append(point)
                
            time.sleep(0.02)
            
        # Add well edge points to main dataset
        self.measurement_points.extend(well_edge_points)
        logger.info(f"Well detection complete: {len(well_edge_points)} edge points collected")
        return True
        
    def execute_edge_validation(self) -> Optional[WellCenterResult]:
        """Execute Phase 3: Edge validation and well center calculation"""
        logger.info("Executing edge validation...")
        
        # Use enhanced analysis to find well center
        if self.zone_classification:
            well_result = self._find_well_center_from_zones(self.zone_classification)
        else:
            # Fallback to standard analysis
            zones = self._classify_measurement_zones(self.measurement_points)
            well_result = self._find_well_center_from_zones(zones)
            
        if well_result and well_result.valid:
            self.well_result = well_result
            logger.info(f"Well center found: ({well_result.center_x:.1f}, {well_result.center_y:.1f}) "
                       f"Diameter: {well_result.well_diameter:.1f}mm Confidence: {well_result.confidence:.1f}%")
        else:
            logger.error(f"Well center validation failed: {well_result.reason if well_result else 'Unknown error'}")
            
        return well_result

    def run_continuous(self):
        """Run continuous enhanced measurement cycles"""
        logger.info("Starting enhanced continuous die measurement system...")
        
        try:
            while True:
                # Set ready status and wait for start command
                self._write_parameter('status', 0)  # Ready
                self._write_parameter('command', 0)  # Reset command
                
                # Wait for start command from robot
                logger.info("Ready for next enhanced measurement cycle...")
                if self._wait_for_robot_command(1, timeout=300):  # 5 minute timeout
                    results = self.run_enhanced_measurement_cycle()
                    
                    if results:
                        logger.info(f"Enhanced cycle complete - Confidence: {results.confidence:.1f}%")
                    else:
                        logger.error("Enhanced measurement cycle failed")
                        
                else:
                    logger.info("No start command received, continuing to wait...")
                    
                time.sleep(1)  # Brief pause between cycles
                
        except KeyboardInterrupt:
            logger.info("Shutting down enhanced die measurement system...")
            self._shutdown_arduino()
        except Exception as e:
            logger.error(f"System error: {str(e)}")
            self._shutdown_arduino()

    def _shutdown_arduino(self):
        """Safely shutdown Arduino connection"""
        if self.arduino:
            try:
                # Turn off sensor power
                self.arduino.write(b'POWER_OFF\n')
                time.sleep(0.5)
                self.arduino.close()
                logger.info("Arduino connection closed, sensor powered off")
            except Exception as e:
                logger.error(f"Error during Arduino shutdown: {e}")


    def _classify_measurement_zones(self, measurements: List[MeasurementPoint]) -> ZoneClassification:
        """Classify measurements into different zones"""
        
        zones = {
            'die_face': [],      # Valid measurements at die face height
            'central_well': [],  # Invalid measurements near center (target well)
            'outer_edges': [],   # Invalid measurements far from center (die edges)
            'other_valid': [],   # Valid measurements at other heights
            'debris': []         # Outlier measurements
        }
        
        # Calculate center estimate for distance-based classification
        all_points = [p for p in measurements if p.is_valid]
        if len(all_points) < 3:
            return ZoneClassification(
                die_face=[], central_well=[], outer_edges=[], 
                other_valid=[], debris=[], die_face_height=50.0
            )
            
        center_x = np.mean([p.x for p in all_points])
        center_y = np.mean([p.y for p in all_points])
        
        # Estimate die face height from valid measurements
        valid_heights = [p.height for p in all_points]
        die_face_height = self._find_die_face_height(valid_heights) if valid_heights else 50.0
            
        # Classify each measurement
        for point in measurements:
            distance_from_center = np.sqrt((point.x - center_x)**2 + (point.y - center_y)**2)
            
            if not point.is_valid:  # Unmeasurable
                if distance_from_center < 20:  # Close to center
                    zones['central_well'].append(point)
                else:  # Far from center
                    zones['outer_edges'].append(point)
                    
            else:  # Valid measurement
                if abs(point.height - die_face_height) < self.height_tolerance:
                    zones['die_face'].append(point)
                elif (self.die_face_height_range[0] <= point.height <= 
                      self.die_face_height_range[1]):
                    zones['other_valid'].append(point)
                else:
                    zones['debris'].append(point)
                    
        return ZoneClassification(
            die_face=zones['die_face'],
            central_well=zones['central_well'],
            outer_edges=zones['outer_edges'],
            other_valid=zones['other_valid'],
            debris=zones['debris'],
            die_face_height=die_face_height
        )
    
    def _find_well_center_from_zones(self, zones: ZoneClassification) -> WellCenterResult:
        """Find well center using zone-classified measurements"""
        
        die_face_points = zones.die_face
        central_well_points = zones.central_well
        
        if len(die_face_points) < 6:
            return WellCenterResult(valid=False, reason="Insufficient die face measurements")
            
        # Estimate well center from invalid points near center
        if len(central_well_points) >= 3:
            well_center_x = np.mean([p.x for p in central_well_points])
            well_center_y = np.mean([p.y for p in central_well_points])
        else:
            # Fallback to geometric center of die face
            well_center_x = np.mean([p.x for p in die_face_points])
            well_center_y = np.mean([p.y for p in die_face_points])
            
        # Find die face points that are near the well edge
        well_edge_candidates = []
        for point in die_face_points:
            dist_to_well = np.sqrt((point.x - well_center_x)**2 + 
                                 (point.y - well_center_y)**2)
            
            if self.well_radius_range[0] <= dist_to_well <= self.well_radius_range[1]:
                well_edge_candidates.append(point)
                
        if len(well_edge_candidates) >= 6:
            # Refine well center using edge points
            refined_center_x = np.mean([p.x for p in well_edge_candidates])
            refined_center_y = np.mean([p.y for p in well_edge_candidates])
            
            # Calculate well diameter
            distances = [np.sqrt((p.x - refined_center_x)**2 + 
                               (p.y - refined_center_y)**2) 
                        for p in well_edge_candidates]
            well_diameter = np.mean(distances) * 2
            
            # Calculate confidence
            radius_consistency = 1.0 - (np.std(distances) / np.mean(distances))
            confidence = min(100, radius_consistency * 100)
            
            return WellCenterResult(
                valid=True,
                center_x=refined_center_x,
                center_y=refined_center_y,
                well_diameter=well_diameter,
                confidence=confidence,
                edge_points=well_edge_candidates
            )
            
        return WellCenterResult(valid=False, reason="Could not identify well edge points")
    
    def _find_die_face_height(self, heights: List[float]) -> float:
        """Find the most likely die face height using clustering"""
        if len(heights) < 3:
            return np.mean(heights)
            
        try:
            # Use 2-means clustering to separate face from other heights
            heights_array = np.array(heights).reshape(-1, 1)
            n_clusters = min(2, len(set(heights)))
            if n_clusters < 2:
                return np.mean(heights)
                
            kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init=10)
            clusters = kmeans.fit_predict(heights_array)
            
            # Return the cluster center with the most points (likely die face)
            cluster_0_count = np.sum(clusters == 0)
            cluster_1_count = np.sum(clusters == 1) if len(set(clusters)) > 1 else 0
            
            if cluster_0_count >= cluster_1_count:
                return kmeans.cluster_centers_[0][0]
            else:
                return kmeans.cluster_centers_[1][0]
        except:
            # Fallback to median
            return np.median(heights)
            
    def send_well_results_to_robot(self, results: WellCenterResult) -> bool:
        """Send well center results back to robot"""
        try:
            self._write_parameter('center_x', results.center_x)
            self._write_parameter('center_y', results.center_y)
            self._write_parameter('avg_height', results.well_diameter)  # Use diameter in avg_height register
            self._write_parameter('confidence', results.confidence)
            
            # Set final status
            if results.confidence >= self.min_confidence_threshold:
                self._write_parameter('status', 2)  # Complete
                logger.info("Well center results sent to robot - measurement successful")
            else:
                self._write_parameter('status', 3)  # Error - low confidence
                logger.warning(f"Low confidence result: {results.confidence:.1f}%")
                
            return True
            
        except Exception as e:
            logger.error(f"Failed to send results to robot: {str(e)}")
            self._write_parameter('status', 3)  # Error
            return False


if __name__ == "__main__":
    # Initialize enhanced die measurement system
    system = KeyenceDieMeasurement(
        ip_address="192.168.0.1",  # Robot IP
        arduino_port="COM3"  # Arduino USB port
    )
    
    # Run continuous enhanced measurement cycles
    system.run_continuous()