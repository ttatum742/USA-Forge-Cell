"""Main robot logger implementation."""

import csv
import os
import time
import logging
from datetime import datetime
from typing import List, Optional

from ..models.data_models import LogEntry, JobData, LoggerConfig
from ..services.robot_service import RobotService
from ..exceptions.custom_exceptions import LoggerError


class RobotLogger:
    """Main robot logger for job completion/abort logging."""
    
    def __init__(self, config: Optional[LoggerConfig] = None):
        """Initialize robot logger."""
        self.config = config or LoggerConfig()
        self.robot = RobotService(self.config)
        self.logger = logging.getLogger(__name__)
        
        # Ensure log directory exists
        os.makedirs(self.config.log_directory, exist_ok=True)
        
        self.running = False
        self.current_job: Optional[JobData] = None
    
    def start_monitoring(self) -> None:
        """Start continuous job monitoring."""
        try:
            self.robot.connect()
            self.running = True
            
            self.logger.info("Robot logger monitoring started")
            
            while self.running:
                self._check_job_status()
                time.sleep(1.0)  # Check every second
                
        except KeyboardInterrupt:
            self.logger.info("Logger monitoring stopped by user")
        except Exception as e:
            self.logger.error(f"Logger monitoring error: {e}")
            raise LoggerError(f"Monitoring failed: {e}")
        finally:
            self.stop_monitoring()
    
    def stop_monitoring(self) -> None:
        """Stop job monitoring."""
        self.running = False
        
        if self.robot.connected:
            self.robot.disconnect()
        
        self.logger.info("Robot logger monitoring stopped")
    
    def _check_job_status(self) -> None:
        """Check current job status and log on completion/abort."""
        try:
            status = self.robot.check_job_completion_status()
            
            if status in ["completed", "aborted", "error"]:
                if self.current_job is None or self.current_job.job_type == "running":
                    # Job just finished - log the data
                    self._log_job_completion(status)
            elif status == "running":
                if self.current_job is None:
                    # New job started
                    self._start_new_job()
        
        except Exception as e:
            self.logger.error(f"Error checking job status: {e}")
    
    def _start_new_job(self) -> None:
        """Start tracking a new job."""
        job_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        self.current_job = JobData(
            job_id=job_id,
            job_type="running",
            start_time=datetime.now(),
            end_time=None,
            registers={},
            metadata={}
        )
        
        self.logger.info(f"Started tracking job {job_id}")
    
    def _log_job_completion(self, status: str) -> None:
        """Log job completion data."""
        try:
            # Read all configured registers
            register_entries = self.robot.read_all_configured_registers()
            
            # Create job data
            if self.current_job is None:
                # Create job for unexpected completion
                self.current_job = JobData(
                    job_id=datetime.now().strftime("%Y%m%d_%H%M%S"),
                    job_type=status,
                    start_time=datetime.now(),  # Unknown start time
                    end_time=datetime.now(),
                    registers={},
                    metadata={}
                )
            
            # Update job data
            self.current_job.job_type = status
            self.current_job.end_time = datetime.now()
            
            # Convert register entries to dictionary
            for entry in register_entries:
                self.current_job.registers[entry.register] = entry.value
            
            # Add metadata
            self.current_job.metadata = {
                "duration_seconds": self.current_job.duration_seconds,
                "register_count": len(register_entries),
                "status": status
            }
            
            # Write to CSV
            self._write_to_csv(self.current_job, register_entries)
            
            self.logger.info(f"Logged {status} job {self.current_job.job_id}")
            
            # Reset current job
            self.current_job = None
            
        except Exception as e:
            self.logger.error(f"Failed to log job completion: {e}")
    
    def _write_to_csv(self, job_data: JobData, register_entries: List[LogEntry]) -> None:
        """Write job data to CSV file."""
        try:
            # Generate filename with date
            date_str = job_data.end_time.strftime("%Y-%m-%d")
            filename = self.config.csv_filename_template.format(date=date_str)
            filepath = os.path.join(self.config.log_directory, filename)
            
            # Check if file exists to determine if we need headers
            file_exists = os.path.exists(filepath)
            
            with open(filepath, 'a', newline='') as csvfile:
                fieldnames = [
                    'job_id', 'job_type', 'start_time', 'end_time', 'duration_seconds'
                ] + list(self.config.registers_to_log.keys())
                
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                # Write header if new file
                if not file_exists:
                    writer.writeheader()
                
                # Prepare row data
                row_data = {
                    'job_id': job_data.job_id,
                    'job_type': job_data.job_type,
                    'start_time': job_data.start_time.isoformat(),
                    'end_time': job_data.end_time.isoformat(),
                    'duration_seconds': job_data.duration_seconds
                }
                
                # Add register values
                for register in self.config.registers_to_log.keys():
                    row_data[register] = job_data.registers.get(register, 0.0)
                
                writer.writerow(row_data)
            
            self.logger.debug(f"Data written to {filepath}")
            
        except Exception as e:
            raise LoggerError(f"Failed to write CSV: {e}")
    
    def log_single_snapshot(self) -> str:
        """
        Take a single snapshot of all registers and save to CSV.
        
        Returns:
            Path to created CSV file
        """
        try:
            if not self.robot.connected:
                self.robot.connect()
            
            # Read all registers
            register_entries = self.robot.read_all_configured_registers()
            
            # Create snapshot job data
            snapshot_job = JobData(
                job_id=f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
                job_type="snapshot",
                start_time=datetime.now(),
                end_time=datetime.now(),
                registers={entry.register: entry.value for entry in register_entries},
                metadata={"type": "manual_snapshot"}
            )
            
            # Write to CSV
            self._write_to_csv(snapshot_job, register_entries)
            
            # Generate filename for return
            date_str = snapshot_job.end_time.strftime("%Y-%m-%d")
            filename = self.config.csv_filename_template.format(date=date_str)
            filepath = os.path.join(self.config.log_directory, filename)
            
            self.logger.info(f"Snapshot logged: {snapshot_job.job_id}")
            
            return filepath
            
        except Exception as e:
            raise LoggerError(f"Failed to log snapshot: {e}")