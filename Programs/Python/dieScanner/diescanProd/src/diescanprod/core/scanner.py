"""Main die scanner production class."""

import time
import logging
from typing import List, Tuple, Optional

from ..models.data_models import ScanPoint, EdgePoint, DieCenter, ScanConfiguration
from ..services.robot_service import RobotService
from ..services.sensor_service import SensorService
from ..config.settings import Settings
from ..exceptions.custom_exceptions import (
    DieScannerError, SafetyError, CommunicationError, ScanningError
)
from .edge_detection import EdgeDetector
from .center_calculation import CenterCalculator
from .advanced_scanning import AdvancedScanningMethods
from ..utils.logging_utils import (
    setup_logger, log_scan_start, log_scan_success, 
    log_scan_failure, log_safety_violation, suppress_debug_output
)
from ..utils.validation import validate_scan_data, validate_edge_points


class DieScannerProd:
    """Production die scanner with safety-first design."""
    
    def __init__(self, settings: Optional[Settings] = None):
        """
        Initialize production die scanner.
        
        Args:
            settings: Scanner configuration (uses defaults if None)
        """
        self.settings = settings or Settings()
        
        # Validate settings for production safety
        self.settings.validate_settings()
        
        # Setup production logging
        suppress_debug_output()
        self.logger = setup_logger(__name__, self.settings.log_level)
        
        # Initialize services
        self.robot = RobotService(self.settings)
        self.sensor = SensorService(self.settings)
        
        # Initialize core components
        self.edge_detector = EdgeDetector(
            self.settings.safety_params,
            self.settings.base_height_threshold
        )
        self.center_calculator = CenterCalculator(self.settings.safety_params)
        self.advanced_scanner = AdvancedScanningMethods(
            self.robot, self.sensor, self.settings.safety_params
        )
        
        # Scanning state
        self.scan_points: List[ScanPoint] = []
        self.edge_points: List[EdgePoint] = []
        self.outer_edges: List[EdgePoint] = []
        self.interior_edges: List[EdgePoint] = []
        self.scanning_active = False
    
    def scan(self) -> DieCenter:
        """
        Execute complete die scanning sequence.
        
        Returns:
            DieCenter with validated results
            
        Raises:
            SafetyError: If scan results are unsafe for operation
            ScanningError: If scanning operation fails
        """
        try:
            log_scan_start(self.logger)
            
            # Step 1: Connect to systems
            self._connect_systems()
            
            # Step 2: Calibrate sensor
            self._calibrate_sensor()
            
            # Step 3: Execute scanning sequence
            self._execute_scan_sequence()
            
            # Step 4: Calculate center with safety validation
            die_center = self._calculate_center()
            
            # Step 5: Write results to robot
            self._write_results_to_robot(die_center)
            
            log_scan_success(self.logger, die_center.center_x, die_center.center_y, die_center.confidence)
            
            return die_center
            
        except SafetyError as e:
            log_safety_violation(self.logger, str(e))
            self._set_robot_error_status()
            raise
        except Exception as e:
            error_msg = f"Scan operation failed: {e}"
            log_scan_failure(self.logger, error_msg)
            self._set_robot_error_status()
            raise ScanningError(error_msg) from e
        finally:
            self._disconnect_systems()
    
    def _connect_systems(self) -> None:
        """Connect to robot and sensor systems."""
        try:
            self.logger.info("Connecting to systems...")
            
            # Connect robot first
            self.robot.connect()
            
            # Connect sensor
            self.sensor.connect()
            
            self.logger.info("All systems connected")
            
        except Exception as e:
            raise CommunicationError(f"System connection failed: {e}")
    
    def _disconnect_systems(self) -> None:
        """Disconnect from all systems."""
        try:
            if hasattr(self.sensor, 'connected') and self.sensor.connected:
                self.sensor.disconnect()
            
            if hasattr(self.robot, 'connected') and self.robot.connected:
                self.robot.disconnect()
                
            self.logger.info("Systems disconnected")
            
        except Exception as e:
            self.logger.error(f"Error during disconnect: {e}")
    
    def _calibrate_sensor(self) -> None:
        """Calibrate sensor at current position."""
        try:
            self.logger.info("Starting sensor calibration...")
            
            # Read current position
            current_pos = self.robot.read_position()
            self.logger.info(f"Calibration position: X={current_pos[0]:.1f}, Y={current_pos[1]:.1f}, Z={current_pos[2]:.1f}")
            
            # Wait for robot to settle
            time.sleep(2.0)
            
            # Perform calibration
            self.sensor.calibrate()
            
            self.logger.info("Sensor calibration completed")
            
        except Exception as e:
            raise DieScannerError(f"Sensor calibration failed: {e}")
    
    def _execute_scan_sequence(self) -> None:
        """Execute the complete scanning sequence."""
        try:
            self.scanning_active = True
            self.scan_points.clear()
            self.edge_points.clear()
            
            # Signal robot to start scanning
            self.robot.write_command(5)  # Start scan command
            
            # Wait for robot ready
            if not self.robot.wait_for_status(10, timeout=30.0):
                raise ScanningError("Robot not ready for scanning")
            
            # Execute 5-pass scanning pattern
            self._execute_5_pass_scan()
            
            # Execute cross-direction scanning
            self._execute_cross_direction_scan()
            
            # Validate collected data
            self._validate_scan_data()
            
            self.logger.info(f"=== Main scanning complete: {len(self.scan_points)} points, {len(self.edge_points)} edges ===")
            
            # Advanced Edge Refinement Phases (Production Safety Critical)
            self.logger.info("=== Starting Advanced Edge Refinement ===")
            
            # Phase 2: Mini-spiral edge refinement
            initial_edges = len(self.edge_points)
            self.logger.info(f"Phase 2: Starting mini-spiral refinement around {initial_edges} detected edges")
            self.edge_points, self.scan_points = self.advanced_scanner.perform_spiral_edge_refinement(
                self.edge_points, self.scan_points
            )
            refined_edges = len(self.edge_points) - initial_edges
            self.logger.info(f"Phase 2 complete: +{refined_edges} refined edges")
            
            # Phase 3: Coverage gap analysis and filling
            self.logger.info("Phase 3: Starting coverage gap analysis")
            self.edge_points, self.scan_points = self.advanced_scanner.perform_coverage_gap_analysis(
                self.edge_points, self.scan_points
            )
            self.logger.info("Phase 3 complete: Coverage gaps filled")
            
            # Phase 4: Quality validation and outlier removal
            self.logger.info("Phase 4: Starting quality validation and outlier removal")
            original_edge_count = len(self.edge_points)
            self.edge_points = self.advanced_scanner.remove_outlier_edges(self.edge_points)
            outliers_removed = original_edge_count - len(self.edge_points)
            self.logger.info(f"Phase 4 complete: {outliers_removed} outlier edges removed")
            
            # Phase 5: Enhanced edge classification with iterative center refinement
            self.logger.info("Phase 5: Starting enhanced edge classification")
            outer_edges, interior_edges = self.advanced_scanner.perform_enhanced_edge_classification(self.edge_points)
            self.logger.info(f"Phase 5 complete: {len(outer_edges)} outer edges, {len(interior_edges)} interior edges classified")
            
            # Store classified edges for center calculation
            self.outer_edges = outer_edges
            self.interior_edges = interior_edges
            
            total_edges = len(self.edge_points)
            self.logger.info(f"=== Edge Refinement Complete: {initial_edges} → {total_edges} edges ===")
            self.logger.info(f"Classification: {len(outer_edges)} outer, {len(interior_edges)} interior")
            
            # Final coverage analysis
            coverage_metrics = self.advanced_scanner.analyze_scanning_coverage(self.scan_points, self.edge_points)
            self.logger.info(f"Final coverage: {coverage_metrics['edge_coverage']:.1f}° angular coverage")
            
            self.scanning_active = False
            self.logger.info("All scanning phases complete, proceeding to center calculation")
            
        except Exception as e:
            self.scanning_active = False
            raise ScanningError(f"Scan sequence failed: {e}")
    
    def _execute_5_pass_scan(self) -> None:
        """Execute 5-pass Y-direction scanning."""
        self.logger.info("Starting 5-pass Y-direction scanning")
        
        # Get scan configuration
        config = self.settings.scan_config
        
        # Read current position for scan start calculation
        cal_pos = self.robot.read_position()
        
        # Calculate scan start position
        start_x = cal_pos[0] + config.start_offset_x
        start_y = cal_pos[1] + config.start_offset_y
        start_z = cal_pos[2]
        
        # Move to scan start position
        self.robot.send_move_command(start_x, start_y, start_z)
        if not self.robot.wait_for_status(10, timeout=15.0):
            raise ScanningError("Robot failed to reach scan start position")
        
        # Execute 5 passes with Y-direction scanning
        current_x = start_x
        current_y = start_y
        
        for pass_num in range(1, 6):
            self.logger.info(f"Executing pass {pass_num}")
            
            # Determine scan direction and distance
            if pass_num % 2 == 1:  # Odd passes: -Y direction
                scan_direction = -1
                scan_start_y = current_y
            else:  # Even passes: +Y direction  
                scan_direction = 1
                scan_start_y = current_y - config.y_scan_distance
            
            # Execute single pass
            self._execute_single_pass(
                current_x, scan_start_y, start_z,
                config.y_scan_distance, config.scan_step, scan_direction
            )
            
            # Move X position for next pass
            if pass_num < 5:
                if pass_num == 3:  # After pass 3, large X move
                    current_x -= config.x_move_large
                else:  # Small X moves
                    current_x -= config.x_move_small
    
    def _execute_single_pass(self, start_x: float, start_y: float, start_z: float,
                           scan_distance: float, step_size: float, direction: int) -> None:
        """Execute a single scanning pass."""
        steps = int(scan_distance / step_size)
        last_height = None
        
        for step in range(steps):
            current_y = start_y + (step * step_size * direction)
            
            # Move to scan position
            self.robot.send_move_command(start_x, current_y, start_z)
            
            # Take measurement
            height = self.sensor.read_height()
            
            # Create scan point
            scan_point = ScanPoint(
                x=start_x,
                y=current_y,
                z=start_z,
                height=height,
                is_valid=height > -999
            )
            
            self.scan_points.append(scan_point)
            
            # Detect edges
            if scan_point.is_valid:
                edge = self.edge_detector.detect_edge(scan_point, last_height)
                if edge is not None:
                    self.edge_points.append(edge)
                
                last_height = height
    
    def _execute_cross_direction_scan(self) -> None:
        """Execute cross-direction X-scanning for validation."""
        self.logger.info("Starting cross-direction X-scanning")
        
        # Get preliminary center for focused scanning
        if len(self.edge_points) >= 3:
            prelim_center_x, prelim_center_y = self.center_calculator.estimate_preliminary_center(self.edge_points)
        else:
            # Use scan area center as fallback for positioning only
            scan_points_valid = [sp for sp in self.scan_points if sp.is_valid]
            if scan_points_valid:
                prelim_center_x = sum(sp.x for sp in scan_points_valid) / len(scan_points_valid)
                prelim_center_y = sum(sp.y for sp in scan_points_valid) / len(scan_points_valid)
            else:
                raise ScanningError("No valid scan points for cross-direction positioning")
        
        # Execute focused X-direction passes around preliminary center
        config = self.settings.scan_config
        scan_distance = 25.4  # 1 inch scan distance
        
        # 3 passes on each side of center
        for offset in [-12.7, 0, 12.7]:  # -0.5", 0", +0.5" from center
            scan_y = prelim_center_y + offset
            self._execute_single_pass(
                prelim_center_x - scan_distance/2, scan_y, self.scan_points[0].z,
                scan_distance, config.refinement_step, 1
            )
    
    def _validate_scan_data(self) -> None:
        """Validate collected scan data meets safety requirements."""
        try:
            # Validate scan points
            validate_scan_data(self.scan_points)
            
            # Validate edge points
            validate_edge_points(self.edge_points, self.settings.safety_params)
            
            self.logger.info("Scan data validation passed")
            
        except Exception as e:
            raise SafetyError(f"Scan data validation failed: {e}")
    
    def _calculate_center(self) -> DieCenter:
        """Calculate die center with safety validation using classified edges."""
        try:
            self.logger.info("Starting center calculation...")
            
            # Use pre-classified outer edges from enhanced classification
            if len(self.outer_edges) < self.settings.safety_params.min_perimeter_edges:
                raise SafetyError(
                    f"Insufficient outer edges: {len(self.outer_edges)} < {self.settings.safety_params.min_perimeter_edges}"
                )
            
            # Calculate center using classified outer edges only
            die_center = self.center_calculator.calculate_center(self.outer_edges, self.scan_points)
            
            self.logger.info(f"Center calculated: ({die_center.center_x:.1f}, {die_center.center_y:.1f})")
            self.logger.info(f"Used {len(self.outer_edges)} outer edges, filtered {len(self.interior_edges)} interior edges")
            
            return die_center
            
        except SafetyError:
            raise  # Re-raise safety errors
        except Exception as e:
            raise SafetyError(f"Center calculation failed: {e}")
    
    def _write_results_to_robot(self, die_center: DieCenter) -> None:
        """Write results to robot registers."""
        try:
            self.robot.write_scan_results(die_center)
            self.robot.set_success_status()
            
            self.logger.info("Results written to robot")
            
        except Exception as e:
            raise CommunicationError(f"Failed to write results to robot: {e}")
    
    def _set_robot_error_status(self) -> None:
        """Set error status on robot."""
        try:
            if hasattr(self.robot, 'connected') and self.robot.connected:
                self.robot.set_error_status()
        except Exception:
            pass  # Don't raise exceptions during error handling
    
    def test_communications(self) -> bool:
        """
        Test robot and sensor communications.
        
        Returns:
            True if both communications working
        """
        try:
            # Test robot communication
            self.robot.connect()
            robot_ok = True
            self.logger.info("Robot communication: OK")
        except Exception as e:
            robot_ok = False
            self.logger.error(f"Robot communication: FAILED - {e}")
        
        try:
            # Test sensor communication
            self.sensor.connect()
            sensor_ok = self.sensor.test_communication()
            if sensor_ok:
                self.logger.info("Sensor communication: OK")
            else:
                self.logger.error("Sensor communication: FAILED")
        except Exception as e:
            sensor_ok = False
            self.logger.error(f"Sensor communication: FAILED - {e}")
        
        # Disconnect after testing
        self._disconnect_systems()
        
        return robot_ok and sensor_ok