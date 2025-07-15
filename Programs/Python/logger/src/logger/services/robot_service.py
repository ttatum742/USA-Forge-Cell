"""Robot communication service for logger package."""

import logging
from typing import Dict, List
from cpppo.server.enip.get_attribute import proxy_simple

from ..models.data_models import LoggerConfig, LogEntry
from ..exceptions.custom_exceptions import CommunicationError


class RobotService(proxy_simple):
    """Robot communication service for logging."""
    
    def __init__(self, config: LoggerConfig):
        """Initialize robot service."""
        super().__init__(host=config.robot_ip, port=config.robot_port)
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.connected = False
    
    def connect(self) -> None:
        """Connect to robot."""
        try:
            # Test connection by reading a register
            test_register = list(self.config.registers_to_log.keys())[0]
            self.read_register(test_register)
            
            self.connected = True
            self.logger.info(f"Connected to robot at {self.host}:{self.port}")
        except Exception as e:
            raise CommunicationError(f"Failed to connect to robot: {e}")
    
    def disconnect(self) -> None:
        """Disconnect from robot."""
        self.connected = False
        self.logger.info("Robot disconnected")
    
    def read_register(self, register: str) -> float:
        """
        Read single register value.
        
        Args:
            register: Register name (e.g., "R[50]")
            
        Returns:
            Register value as float
        """
        try:
            # Convert register format for CPPPO
            # R[50] -> @0x6B/1/50 for DINT registers
            # Need to determine if DINT or REAL based on register number
            reg_num = int(register.split('[')[1].split(']')[0])
            
            if reg_num >= 60 and reg_num <= 95:  # Real registers
                param_str = f"@0x6C/1/{reg_num}"
            else:  # DINT registers
                param_str = f"@0x6B/1/{reg_num}"
            
            result, = self.read(param_str, checking=True)
            return float(result[0]) if isinstance(result, list) else float(result)
            
        except Exception as e:
            raise CommunicationError(f"Failed to read {register}: {e}")
    
    def read_all_configured_registers(self) -> List[LogEntry]:
        """
        Read all configured registers.
        
        Returns:
            List of LogEntry objects
        """
        if not self.connected:
            raise CommunicationError("Robot not connected")
        
        entries = []
        
        for register, description in self.config.registers_to_log.items():
            try:
                value = self.read_register(register)
                
                entry = LogEntry(
                    timestamp=None,  # Will use current time
                    register=register,
                    value=value,
                    description=description
                )
                entries.append(entry)
                
            except CommunicationError as e:
                self.logger.error(f"Failed to read {register}: {e}")
                # Continue with other registers
                continue
        
        return entries
    
    def check_job_completion_status(self) -> str:
        """
        Check robot job status for completion/abort detection.
        
        Returns:
            Status string: "running", "completed", "aborted", "error"
        """
        try:
            # Example: Check system variables or specific registers
            # This would need to be customized based on robot program logic
            
            # For now, return basic status
            # In practice, you'd check specific robot status registers
            return "running"
            
        except Exception as e:
            self.logger.error(f"Failed to check job status: {e}")
            return "error"