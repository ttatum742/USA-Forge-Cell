"""Configuration management for die scanner production package."""

import os
from typing import Optional
from ..models.data_models import ScanConfiguration, SafetyParameters


class Settings:
    """Production settings for die scanner package."""
    
    def __init__(self):
        # Robot communication settings
        self.robot_ip: str = os.getenv("ROBOT_IP", "192.168.0.1")
        self.robot_port: int = int(os.getenv("ROBOT_PORT", "44818"))
        self.robot_timeout: float = float(os.getenv("ROBOT_TIMEOUT", "5.0"))
        
        # Arduino/sensor settings
        self.arduino_port: str = os.getenv("ARDUINO_PORT", "COM8")
        self.arduino_baud: int = int(os.getenv("ARDUINO_BAUD", "115200"))
        self.arduino_timeout: float = float(os.getenv("ARDUINO_TIMEOUT", "2.0"))
        
        # Calibration settings
        self.calibration_reference: float = float(os.getenv("CALIBRATION_REFERENCE", "65.0"))
        self.calibration_tolerance: float = float(os.getenv("CALIBRATION_TOLERANCE", "0.5"))
        
        # Edge detection settings
        self.base_height_threshold: float = float(os.getenv("BASE_HEIGHT_THRESHOLD", "2.0"))
        self.adaptive_threshold_factor: float = float(os.getenv("ADAPTIVE_THRESHOLD_FACTOR", "1.5"))
        self.edge_confirmation_points: int = int(os.getenv("EDGE_CONFIRMATION_POINTS", "3"))
        
        # Logging settings
        self.log_level: str = os.getenv("LOG_LEVEL", "INFO")
        self.enable_debug_output: bool = os.getenv("ENABLE_DEBUG_OUTPUT", "false").lower() == "true"
        
        # Performance settings
        self.max_scan_duration: float = float(os.getenv("MAX_SCAN_DURATION", "45.0"))
        self.communication_retry_count: int = int(os.getenv("COMMUNICATION_RETRY_COUNT", "3"))
        self.measurement_rate_hz: float = float(os.getenv("MEASUREMENT_RATE_HZ", "100.0"))
        
        # Initialize sub-configurations
        self.scan_config = ScanConfiguration()
        self.safety_params = SafetyParameters()
    
    def validate_settings(self) -> bool:
        """Validate all settings for production safety."""
        if not self.robot_ip:
            raise ValueError("Robot IP address must be specified")
        
        if self.robot_timeout <= 0:
            raise ValueError("Robot timeout must be positive")
            
        if not self.arduino_port:
            raise ValueError("Arduino port must be specified")
            
        if self.calibration_reference <= 0:
            raise ValueError("Calibration reference must be positive")
            
        if self.base_height_threshold <= 0:
            raise ValueError("Base height threshold must be positive")
            
        if self.safety_params.min_perimeter_edges < 5:
            raise ValueError("Minimum perimeter edges must be at least 5 for safety")
            
        if self.safety_params.min_angular_coverage < 270.0:
            raise ValueError("Minimum angular coverage must be at least 270° for safety")
            
        if self.safety_params.min_confidence_score < 80.0:
            raise ValueError("Minimum confidence score must be at least 80% for safety")
            
        return True
    
    def get_robot_address(self) -> tuple:
        """Get robot connection address tuple."""
        return (self.robot_ip, self.robot_port)
    
    def is_production_mode(self) -> bool:
        """Check if running in production mode (higher safety requirements)."""
        return os.getenv("PRODUCTION_MODE", "true").lower() == "true"