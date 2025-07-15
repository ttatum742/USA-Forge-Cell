# Robot Logger Package - Deployment Guide

## Overview

The Robot Logger package provides automated logging of robot register data for job completion and abort events in the FANUC forging cell.

## Installation

### Method 1: Development Installation

```bash
# Navigate to logger package directory
cd "Programs/Python/logger"

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/WSL
# OR
venv\Scripts\activate     # Windows

# Install in development mode
pip install -e .

# Verify installation
python -c "from logger import RobotLogger; print('Logger installed successfully')"
```

### Method 2: Production Installation

```bash
# Install from source
pip install .

# Or create wheel and install
python -m build
pip install dist/logger-1.0.0-py3-none-any.whl
```

## Configuration

### Environment Variables

```bash
# Robot Configuration
export ROBOT_IP=192.168.0.1
export ROBOT_PORT=44818

# Logging Configuration
export LOG_DIRECTORY=logs
export LOG_LEVEL=INFO
```

### Custom Configuration

```python
from logger.models.data_models import LoggerConfig

# Custom configuration
config = LoggerConfig(
    robot_ip="192.168.0.1",
    robot_port=44818,
    log_directory="production_logs",
    csv_filename_template="robot_log_{date}.csv",
    registers_to_log={
        "R[22]": "Furnace 1 state",
        "R[23]": "Furnace 2 state",
        "R[50]": "J1 torque",
        "R[51]": "J2 torque",
        # Add more registers as needed
    }
)
```

## Usage

### Continuous Monitoring

```python
from logger import RobotLogger

# Initialize logger
logger = RobotLogger()

# Start continuous monitoring (blocks until stopped)
logger.start_monitoring()
```

### Manual Snapshot

```python
from logger import RobotLogger

# Take single snapshot
logger = RobotLogger()
csv_file = logger.log_single_snapshot()
print(f"Snapshot saved to: {csv_file}")
```

### Background Service

Create `robot_logger_service.py`:

```python
#!/usr/bin/env python3
"""Robot logger background service."""

import signal
import sys
from logger import RobotLogger

def signal_handler(sig, frame):
    print('Stopping robot logger...')
    sys.exit(0)

def main():
    # Setup signal handling
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Start logger
    logger = RobotLogger()
    
    try:
        print("Starting robot logger monitoring...")
        logger.start_monitoring()
    except Exception as e:
        print(f"Logger error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
```

## Deployment Options

### Option 1: Manual Execution

```bash
# Run logger manually
python robot_logger_service.py
```

### Option 2: Windows Service

Create `logger_service.bat`:

```batch
@echo off
cd /d "C:\production\logger"
call venv\Scripts\activate.bat

python robot_logger_service.py

pause
```

### Option 3: Linux systemd Service

Create `/etc/systemd/system/robot-logger.service`:

```ini
[Unit]
Description=Robot Logger Service
After=network.target

[Service]
Type=simple
User=robotuser
WorkingDirectory=/home/robotuser/logger
Environment=PATH=/home/robotuser/logger/venv/bin
ExecStart=/home/robotuser/logger/venv/bin/python robot_logger_service.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl enable robot-logger
sudo systemctl start robot-logger
sudo systemctl status robot-logger
```

## Package Structure

The logger package follows the standard Python package structure:

```
logger/
├── src/logger/
│   ├── __init__.py
│   ├── core/
│   │   ├── __init__.py
│   │   └── logger.py          # Main RobotLogger class
│   ├── models/
│   │   ├── __init__.py
│   │   └── data_models.py     # LogEntry, JobData, LoggerConfig
│   ├── services/
│   │   ├── __init__.py
│   │   └── robot_service.py   # Robot communication
│   └── exceptions/
│       ├── __init__.py
│       └── custom_exceptions.py
├── tests/
├── requirements.txt
└── pyproject.toml
```

## Customization

### Adding New Registers

```python
# Extend default registers
config = LoggerConfig()
config.registers_to_log.update({
    "R[100]": "Custom register 1",
    "R[101]": "Custom register 2",
})
```

### Custom Job Detection Logic

Extend the `RobotService.check_job_completion_status()` method:

```python
def check_job_completion_status(self) -> str:
    # Custom logic to detect job completion
    # Example: Check specific system variables
    try:
        # Read job status register
        job_status = self.read_register("R[99]")
        
        if job_status == 1:
            return "completed"
        elif job_status == 2:
            return "aborted"
        elif job_status == 3:
            return "error"
        else:
            return "running"
            
    except Exception:
        return "error"
```

## Monitoring and Maintenance

### Log File Management

```bash
# View recent logs
ls -la logs/

# Check today's log
tail -f logs/robot_log_$(date +%Y-%m-%d).csv

# Archive old logs
find logs/ -name "*.csv" -mtime +30 -exec mv {} archive/ \;
```

### Health Checking

```python
# Health check script
from logger import RobotLogger

def health_check():
    try:
        logger = RobotLogger()
        logger.robot.connect()
        
        # Test register read
        entries = logger.robot.read_all_configured_registers()
        
        if entries:
            print("✅ Logger health check passed")
            return True
        else:
            print("❌ No register data received")
            return False
            
    except Exception as e:
        print(f"❌ Health check failed: {e}")
        return False

if __name__ == "__main__":
    health_check()
```

## Troubleshooting

### Common Issues

#### Robot Connection Errors

```bash
# Test robot connectivity
ping 192.168.0.1

# Test register access
python -c "
from logger.services.robot_service import RobotService
from logger.models.data_models import LoggerConfig
service = RobotService(LoggerConfig())
service.connect()
print('Connection successful')
"
```

#### Permission Errors

```bash
# Ensure log directory is writable
mkdir -p logs
chmod 755 logs

# Check file permissions
ls -la logs/
```

#### Missing Registers

```bash
# Test individual register reads
python -c "
from logger.services.robot_service import RobotService
from logger.models.data_models import LoggerConfig
service = RobotService(LoggerConfig())
service.connect()
value = service.read_register('R[22]')
print(f'R[22] = {value}')
"
```

## Integration with Other Systems

### Integration with DieScanner

The logger can be triggered by die scanner events:

```python
# In die scanner completion
from logger import RobotLogger

def on_scan_complete():
    # Log current state after scan
    logger = RobotLogger()
    logger.log_single_snapshot()
```

### Database Integration

Extend the logger to write to database:

```python
import sqlite3
from logger.core.logger import RobotLogger as BaseLogger

class DatabaseLogger(BaseLogger):
    def __init__(self, db_path="robot_data.db"):
        super().__init__()
        self.db_path = db_path
        self._init_database()
    
    def _init_database(self):
        conn = sqlite3.connect(self.db_path)
        conn.execute("""
            CREATE TABLE IF NOT EXISTS job_logs (
                id INTEGER PRIMARY KEY,
                job_id TEXT,
                job_type TEXT,
                start_time TEXT,
                end_time TEXT,
                register_data TEXT
            )
        """)
        conn.close()
    
    def _write_to_database(self, job_data):
        # Custom database writing logic
        pass
```

This logger package provides a flexible foundation for robot data logging that can be extended and customized based on specific requirements.