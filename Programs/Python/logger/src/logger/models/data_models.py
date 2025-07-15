"""Data models for robot logger package."""

from dataclasses import dataclass
from typing import Dict, Any, Optional
from datetime import datetime


@dataclass
class LogEntry:
    """Individual log entry for robot data."""
    timestamp: datetime
    register: str
    value: float
    description: str = ""
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()


@dataclass 
class JobData:
    """Complete job data record."""
    job_id: str
    job_type: str  # "completed", "aborted", "error"
    start_time: datetime
    end_time: datetime
    registers: Dict[str, float]
    metadata: Dict[str, Any]
    
    def __post_init__(self):
        if self.end_time is None:
            self.end_time = datetime.now()
    
    @property
    def duration_seconds(self) -> float:
        """Calculate job duration in seconds."""
        return (self.end_time - self.start_time).total_seconds()


@dataclass
class LoggerConfig:
    """Logger configuration parameters."""
    robot_ip: str = "192.168.0.1"
    robot_port: int = 44818
    log_directory: str = "logs"
    csv_filename_template: str = "robot_log_{date}.csv"
    registers_to_log: Dict[str, str] = None  # register -> description mapping
    
    def __post_init__(self):
        if self.registers_to_log is None:
            # Default registers to log from forging cell
            self.registers_to_log = {
                "R[22]": "Furnace 1 state",
                "R[23]": "Furnace 2 state", 
                "R[24]": "Priority furnace",
                "R[25]": "Available furnace",
                "R[50]": "J1 torque",
                "R[51]": "J2 torque",
                "R[52]": "J3 torque",
                "R[53]": "J4 torque", 
                "R[54]": "J5 torque",
                "R[55]": "J6 torque",
                "R[80]": "J1 torque derivative",
                "R[81]": "J2 torque derivative",
                "R[82]": "J3 torque derivative",
                "R[83]": "J4 torque derivative",
                "R[84]": "J5 torque derivative",
                "R[85]": "J6 torque derivative"
            }