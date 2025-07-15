"""Sensor communication service for die scanner production package."""

import serial
import time
import logging
from typing import Optional

from ..exceptions.custom_exceptions import CommunicationError, CalibrationError
from ..config.settings import Settings
from ..utils.validation import validate_calibration_result


class SensorService:
    """Production sensor communication service for Keyence/Arduino interface."""
    
    def __init__(self, settings: Settings):
        """Initialize sensor service with production settings."""
        self.settings = settings
        self.logger = logging.getLogger(__name__)
        self.serial_conn: Optional[serial.Serial] = None
        self.connected = False
        self._connection_retries = 0

    def connect(self) -> None:
        """
        Connect to Arduino/sensor with retry logic.
        
        Raises CommunicationError if connection fails.
        """
        max_retries = self.settings.communication_retry_count
        
        for attempt in range(max_retries + 1):
            try:
                self.logger.info(f"Connecting to sensor at {self.settings.arduino_port} (attempt {attempt + 1})")
                
                self.serial_conn = serial.Serial(
                    self.settings.arduino_port,
                    self.settings.arduino_baud,
                    timeout=self.settings.arduino_timeout
                )
                
                time.sleep(2)  # Arduino reset delay
                
                # Clear buffer and test communication
                self.serial_conn.reset_input_buffer()
                time.sleep(0.1)
                
                # Test communication
                test_response = self._send_command("READ")
                if test_response.startswith("HEIGHT:") or "ERROR" in test_response:
                    self.connected = True
                    self.logger.info("Sensor connection established")
                    return
                else:
                    raise CommunicationError(f"Invalid test response: {test_response}")
                
            except Exception as e:
                self._connection_retries = attempt + 1
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
                
                if attempt < max_retries:
                    self.logger.warning(f"Sensor connection attempt {attempt + 1} failed: {e}")
                    time.sleep(1.0)
                else:
                    raise CommunicationError(f"Failed to connect to sensor after {max_retries + 1} attempts: {e}")

    def disconnect(self) -> None:
        """Disconnect from sensor."""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.connected = False
            self.logger.info("Sensor disconnected")
        except Exception as e:
            self.logger.error(f"Error during sensor disconnect: {e}")

    def _send_command(self, command: str) -> str:
        """
        Send command to Arduino and get response.
        
        Raises CommunicationError if communication fails.
        """
        if not self.connected or not self.serial_conn:
            raise CommunicationError("Sensor not connected")
        
        try:
            # Send command
            self.serial_conn.write(f"{command}\n".encode())
            
            # Read response with timeout
            response = self.serial_conn.readline().decode().strip()
            
            if not response:
                raise CommunicationError(f"No response to command '{command}'")
            
            return response
            
        except Exception as e:
            raise CommunicationError(f"Sensor command '{command}' failed: {e}")

    def read_height(self) -> float:
        """
        Read current height measurement.
        
        Returns:
            Height measurement in mm
            
        Raises CommunicationError if read fails.
        """
        response = self._send_command("READ")
        
        if response.startswith("HEIGHT:"):
            try:
                height_str = response.replace("HEIGHT:", "")
                height = float(height_str)
                return height
            except ValueError:
                raise CommunicationError(f"Invalid height response: {response}")
        else:
            raise CommunicationError(f"Invalid read response: {response}")

    def calibrate(self, reference_distance: Optional[float] = None) -> None:
        """
        Perform sensor calibration.
        
        Args:
            reference_distance: Reference distance in mm (uses config default if None)
            
        Raises CalibrationError if calibration fails.
        """
        if reference_distance is None:
            reference_distance = self.settings.calibration_reference
        
        try:
            self.logger.info(f"Starting sensor calibration with reference {reference_distance}mm")
            
            # Clear any pending messages
            self.serial_conn.reset_input_buffer()
            time.sleep(0.1)
            
            # Start calibration
            response = self._send_command("CALIBRATE")
            self.logger.debug(f"Calibration start response: {response}")
            
            if "CALIBRATION:STARTING" not in response:
                raise CalibrationError(f"Failed to start calibration: {response}")
            
            time.sleep(0.5)
            
            # Send reference distance
            response = self._send_command(f"CAL_DISTANCE {reference_distance}")
            self.logger.debug(f"Reference distance response: {response}")
            
            # Wait for calibration completion
            for i in range(30):  # 30 second timeout
                time.sleep(1.0)
                
                try:
                    status_response = self._send_command("CAL_STATUS")
                    
                    if "CALIBRATION:COMPLETE" in status_response:
                        self.logger.info("Calibration completed successfully")
                        break
                    elif "CALIBRATION:FAILED" in status_response:
                        raise CalibrationError("Calibration failed during execution")
                    elif "CALIBRATION:RUNNING" in status_response:
                        continue  # Still running
                    else:
                        self.logger.debug(f"Calibration status: {status_response}")
                        
                except CommunicationError:
                    # Continue waiting during temporary communication issues
                    continue
            else:
                raise CalibrationError("Calibration timeout after 30 seconds")
            
            # Validate calibration result
            time.sleep(1.0)
            validation_readings = []
            
            for _ in range(5):
                try:
                    height = self.read_height()
                    if height > -999:  # Valid reading
                        validation_readings.append(height)
                    time.sleep(0.2)
                except CommunicationError:
                    continue
            
            if len(validation_readings) < 3:
                raise CalibrationError("Insufficient validation readings after calibration")
            
            avg_height = sum(validation_readings) / len(validation_readings)
            
            # Validate against reference with tolerance
            validate_calibration_result(
                avg_height, 
                reference_distance, 
                self.settings.calibration_tolerance
            )
            
            self.logger.info(f"Calibration validated: {avg_height:.2f}mm (reference: {reference_distance:.2f}mm)")
            
        except CalibrationError:
            raise  # Re-raise calibration errors
        except Exception as e:
            raise CalibrationError(f"Calibration failed: {e}")

    def read_multiple_heights(self, count: int, delay: float = 0.1) -> list[float]:
        """
        Read multiple height measurements.
        
        Args:
            count: Number of readings to take
            delay: Delay between readings in seconds
            
        Returns:
            List of height measurements
        """
        heights = []
        failed_readings = 0
        max_failures = count // 2  # Allow up to 50% failure rate
        
        for i in range(count):
            try:
                height = self.read_height()
                if height > -999:  # Valid reading
                    heights.append(height)
                else:
                    failed_readings += 1
                    
                if delay > 0:
                    time.sleep(delay)
                    
            except CommunicationError:
                failed_readings += 1
                if failed_readings > max_failures:
                    raise CommunicationError(f"Too many failed readings: {failed_readings}/{i+1}")
                continue
        
        if len(heights) < count // 2:
            raise CommunicationError(f"Insufficient valid readings: {len(heights)}/{count}")
        
        return heights

    def test_communication(self) -> bool:
        """
        Test sensor communication.
        
        Returns:
            True if communication is working
        """
        try:
            response = self._send_command("READ")
            return response.startswith("HEIGHT:") or "ERROR" in response
        except CommunicationError:
            return False