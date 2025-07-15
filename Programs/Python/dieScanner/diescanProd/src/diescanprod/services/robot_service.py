"""Robot communication service for die scanner production package."""

import time
import logging
from typing import List, Tuple, Optional
from cpppo.server.enip.get_attribute import proxy_simple

from ..models.data_models import DieCenter
from ..exceptions.custom_exceptions import CommunicationError, ValidationError
from ..config.settings import Settings


class RobotService(proxy_simple):
    """Production robot communication service using CPPPO."""
    
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
        
        # Next position components (R[182-184])
        ds_next_pos_x=proxy_simple.parameter('@0x6C/1/182', 'REAL', 'mm'),
        ds_next_pos_y=proxy_simple.parameter('@0x6C/1/183', 'REAL', 'mm'),
        ds_next_pos_z=proxy_simple.parameter('@0x6C/1/184', 'REAL', 'mm'),
        
        # Position request (R[185])
        ds_pos_request=proxy_simple.parameter('@0x6B/1/185', 'DINT', 'request'),
    )

    def __init__(self, settings: Settings):
        """Initialize robot service with production settings."""
        super().__init__(host=settings.robot_ip, port=settings.robot_port)
        self.settings = settings
        self.logger = logging.getLogger(__name__)
        self.connected = False
        self._connection_retries = 0

    def connect(self) -> None:
        """
        Connect to robot with retry logic.
        
        Raises CommunicationError if connection fails.
        """
        max_retries = self.settings.communication_retry_count
        
        for attempt in range(max_retries + 1):
            try:
                self.logger.info(f"Connecting to robot at {self.host}:{self.port} (attempt {attempt + 1})")
                
                # Test connection by reading status register
                self.read_status()
                
                self.connected = True
                self.logger.info("Robot connection established")
                return
                
            except Exception as e:
                self._connection_retries = attempt + 1
                if attempt < max_retries:
                    self.logger.warning(f"Connection attempt {attempt + 1} failed: {e}")
                    time.sleep(1.0)
                else:
                    raise CommunicationError(f"Failed to connect to robot after {max_retries + 1} attempts: {e}")

    def disconnect(self) -> None:
        """Disconnect from robot."""
        try:
            self.connected = False
            self.logger.info("Robot disconnected")
        except Exception as e:
            self.logger.error(f"Error during robot disconnect: {e}")

    def _write_parameter(self, parameter_name: str, value: float) -> None:
        """
        Write parameter to robot register with error handling.
        
        Raises CommunicationError if write fails.
        """
        if not self.connected:
            raise CommunicationError("Robot not connected")
        
        try:
            if isinstance(value, int) or 'status' in parameter_name or 'command' in parameter_name:
                param = f'{parameter_name} = (DINT) {int(value)}'
            else:
                param = f'{parameter_name} = (REAL) {float(value)}'
            
            success, = self.write(
                self.parameter_substitution(param), checking=True)
            
            if not success:
                raise CommunicationError(f"Failed to write {parameter_name}={value}")
                
        except Exception as e:
            raise CommunicationError(f"Robot write error {parameter_name}={value}: {e}")

    def _read_parameter(self, parameter_name: str) -> float:
        """
        Read parameter from robot register with error handling.
        
        Raises CommunicationError if read fails.
        """
        if not self.connected:
            raise CommunicationError("Robot not connected")
        
        try:
            param_str = self.parameter_substitution(parameter_name)
            result, = self.read(param_str, checking=True)
            return float(result[0]) if isinstance(result, list) else float(result)
        except Exception as e:
            raise CommunicationError(f"Robot read error {parameter_name}: {e}")

    def read_position(self) -> Tuple[float, float, float]:
        """
        Read current robot position.
        
        Returns:
            Tuple of (x, y, z) coordinates
        """
        try:
            # Request position calculation
            self._write_parameter('ds_pos_request', 1)
            time.sleep(0.01)  # Allow calculation time
            
            x = self._read_parameter('ds_current_x')
            y = self._read_parameter('ds_current_y')
            z = self._read_parameter('ds_current_z')
            
            return (x, y, z)
        except Exception as e:
            raise CommunicationError(f"Failed to read robot position: {e}")

    def read_status(self) -> int:
        """Read robot status register."""
        return int(self._read_parameter('ds_status'))

    def write_command(self, command_code: int) -> None:
        """Write command to robot."""
        self._write_parameter('ds_command', command_code)

    def wait_for_status(self, expected_status: int, timeout: float = 30.0) -> bool:
        """
        Wait for robot to reach expected status.
        
        Returns:
            True if status reached, False if timeout
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                current_status = self.read_status()
                if current_status == expected_status:
                    return True
                time.sleep(0.1)
            except CommunicationError:
                # Continue waiting during temporary communication issues
                time.sleep(0.5)
        
        return False

    def write_scan_results(self, die_center: DieCenter) -> None:
        """
        Write scan results to robot registers.
        
        Args:
            die_center: Die center calculation results
        """
        try:
            # Write primary results
            self._write_parameter('ds_center_x', die_center.center_x)
            self._write_parameter('ds_center_y', die_center.center_y)
            self._write_parameter('ds_avg_height', die_center.avg_height)
            self._write_parameter('ds_confidence', int(die_center.confidence))
            
            # Write edge points (up to 8 points)
            edge_registers = [
                ('ds_edge_x1', 'ds_edge_y1'), ('ds_edge_x2', 'ds_edge_y2'),
                ('ds_edge_x3', 'ds_edge_y3'), ('ds_edge_x4', 'ds_edge_y4'),
                ('ds_edge_x5', 'ds_edge_y5'), ('ds_edge_x6', 'ds_edge_y6'),
                ('ds_edge_x7', 'ds_edge_y7'), ('ds_edge_x8', 'ds_edge_y8')
            ]
            
            edges_to_write = die_center.edge_points[:8]
            
            for i, (x_reg, y_reg) in enumerate(edge_registers):
                if i < len(edges_to_write):
                    self._write_parameter(x_reg, edges_to_write[i].x)
                    self._write_parameter(y_reg, edges_to_write[i].y)
                else:
                    self._write_parameter(x_reg, 0.0)
                    self._write_parameter(y_reg, 0.0)
            
            self.logger.info("Scan results written to robot registers")
            
        except Exception as e:
            raise CommunicationError(f"Failed to write scan results: {e}")

    def set_success_status(self) -> None:
        """Set successful completion status."""
        self._write_parameter('ds_status', 2)  # 2 = success

    def set_error_status(self) -> None:
        """Set error status."""
        self._write_parameter('ds_status', 3)  # 3 = error

    def send_move_command(self, x: float, y: float, z: float) -> None:
        """Send move command to robot."""
        self._write_parameter('ds_next_pos_x', x)
        self._write_parameter('ds_next_pos_y', y)
        self._write_parameter('ds_next_pos_z', z)
        self._write_parameter('ds_status', 11)  # 11 = move request