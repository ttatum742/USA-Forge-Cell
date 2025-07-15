# DieScanner Production Package - Deployment Guide

## ⚠️ SAFETY CRITICAL DEPLOYMENT ⚠️

This package is designed for **2000°F forging operations** where incorrect calculations could result in **operator fatalities**. Follow this deployment guide exactly.

## Pre-Deployment Requirements

### System Requirements
- **Python 3.8+** (tested with 3.8, 3.9, 3.10, 3.11)
- **Windows 10/11** or **Linux** (WSL2 supported)
- **FANUC Robot** with Ethernet/IP capability
- **Arduino with Keyence sensor** interface
- **Network connectivity** to robot (192.168.0.1 default)
- **Serial port access** for Arduino communication

### Hardware Validation
1. **Robot Communication**: Verify Ethernet/IP access to registers R[86-189]
2. **Sensor Interface**: Confirm Arduino responds on configured COM port
3. **Network Configuration**: Ensure robot IP accessibility
4. **Safety Systems**: Verify all emergency stops and safety interlocks

## Installation Methods

### Method 1: Development Installation (Recommended for Testing)

```bash
# Navigate to package directory
cd "Programs/Python/dieScanner/diescanProd"

# Create virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # Linux/WSL
# OR
venv\Scripts\activate     # Windows

# Install in development mode with all dependencies
pip install -e ".[dev]"

# Verify installation
diescan-prod --version
```

### Method 2: Production Installation

```bash
# Install from wheel (after building)
pip install dist/diescanprod-1.0.0-py3-none-any.whl

# Or install from source
pip install .

# Verify installation
diescan-prod --version
```

### Method 3: Isolated Production Environment

```bash
# Create isolated environment for production
python -m venv production_env
source production_env/bin/activate  # Linux/WSL
# OR
production_env\Scripts\activate     # Windows

# Install only production dependencies
pip install -r requirements.txt
pip install .

# Lock dependencies for reproducible deployments
pip freeze > deployed_requirements.txt
```

## Configuration Setup

### Environment Variables

Create a `.env` file or set system environment variables:

```bash
# Robot Configuration
export ROBOT_IP=192.168.0.1
export ROBOT_PORT=44818
export ROBOT_TIMEOUT=5.0

# Arduino/Sensor Configuration  
export ARDUINO_PORT=COM8          # Windows
# export ARDUINO_PORT=/dev/ttyUSB0  # Linux
export ARDUINO_BAUD=115200
export ARDUINO_TIMEOUT=2.0

# Calibration Settings
export CALIBRATION_REFERENCE=65.0
export CALIBRATION_TOLERANCE=0.5

# Safety Settings
export PRODUCTION_MODE=true       # CRITICAL: Must be true for production
export LOG_LEVEL=INFO

# Performance Settings
export MAX_SCAN_DURATION=45.0
export COMMUNICATION_RETRY_COUNT=3
```

### Windows Environment Variables (Alternative)

```cmd
# Set persistent environment variables (run as Administrator)
setx ROBOT_IP "192.168.0.1" /M
setx ARDUINO_PORT "COM8" /M
setx PRODUCTION_MODE "true" /M
setx LOG_LEVEL "INFO" /M
```

### Configuration Validation

```bash
# Validate configuration before deployment
diescan-prod validate-config

# Test communications
diescan-prod test-comms

# Show current configuration
diescan-prod config
```

## Pre-Production Testing

### 1. Communication Testing

```bash
# Test robot communication
diescan-prod test-comms

# Expected output:
# ✅ Robot communication: OK
# ✅ Sensor communication: OK
# ✅ All communications OK
```

### 2. Calibration Testing

```bash
# Test sensor calibration
diescan-prod calibrate --output-file calibration_test.txt

# Expected output:
# ✅ Sensor calibration completed successfully
# 📏 Average Height: 65.02mm
# 📊 Standard Deviation: 0.05mm
# 🏆 EXCELLENT calibration stability
```

### 3. Configuration Testing

```bash
# Verify production safety parameters
diescan-prod config

# Verify these values for production:
# Min Perimeter Edges: 10
# Min Angular Coverage: 300.0°
# Min Confidence Score: 90.0%
# Production Mode: True
```

### 4. Test Scan (TESTING MODE ONLY)

```bash
# IMPORTANT: Only run with testing parameters initially
export PRODUCTION_MODE=false

# Run test scan
diescan-prod scan --testing

# Expected output:
# ✅ SCAN SUCCESSFUL
# 📍 Die Center: [398.1, 809.2] mm  
# 📏 Diameter: 114.1 mm
# 🎯 Confidence: 92.3%
```

## Production Deployment

### 1. Production Environment Setup

```bash
# Create production directory
mkdir /production/diescan
cd /production/diescan

# Copy package files
cp -r /path/to/diescanProd/* .

# Install in production mode
python -m venv prod_env
source prod_env/bin/activate
pip install -r requirements.txt
pip install .
```

### 2. Production Configuration

```bash
# Set production environment variables
export PRODUCTION_MODE=true
export LOG_LEVEL=INFO
export ROBOT_IP=192.168.0.1
export ARDUINO_PORT=COM8

# Validate production configuration
diescan-prod validate-config

# Expected output:
# ✅ Configuration validation passed
# 🛡️  All safety parameters meet requirements
# 🏭 PRODUCTION MODE - Full safety validation active
```

### 3. Service Integration Options

#### Option A: Manual Execution

```bash
# Run single scan
diescan-prod scan

# Run with specific configuration
diescan-prod scan --config-file production_config.json
```

#### Option B: Batch Script Integration

Create `run_diescan.bat` (Windows):

```batch
@echo off
cd /d "C:\production\diescan"
call prod_env\Scripts\activate.bat

REM Set production environment
set PRODUCTION_MODE=true
set LOG_LEVEL=INFO

REM Run die scanner
diescan-prod scan

REM Check exit code
if %ERRORLEVEL% NEQ 0 (
    echo SCAN FAILED - Check logs
    exit /b 1
)

echo SCAN COMPLETED SUCCESSFULLY
exit /b 0
```

#### Option C: Background Service (Advanced)

Create Windows Service or Linux systemd service for continuous monitoring.

### 4. Robot Program Integration

Update robot program to call die scanner:

```fanuc
! Robot LS program integration
! Set scanner ready status
R[87:ds_status] = 10

! Wait for scanner completion  
WAIT_FOR R[87:ds_status] <> 10 TIMEOUT=60

! Check result
IF R[87:ds_status] = 2 THEN
  ! Success - use calculated center
  PR[37,1] = R[93:ds_center_x]
  PR[37,2] = R[94:ds_center_y]
ELSE
  ! Failure - STOP operation
  ABORT
ENDIF
```

## Safety Validation Checklist

### Pre-Deployment Safety Checks

- [ ] **Communication verified**: Robot and sensor responding
- [ ] **Calibration stable**: Standard deviation < 0.1mm
- [ ] **Production mode enabled**: `PRODUCTION_MODE=true`
- [ ] **Safety parameters validated**: Min 10 edges, 300° coverage, 90% confidence
- [ ] **Emergency stops tested**: All safety systems functional
- [ ] **Operator training completed**: Staff understand error messages

### Production Safety Validation

- [ ] **No fallback values**: System fails completely on insufficient data
- [ ] **Angular coverage**: Minimum 300° perimeter coverage required
- [ ] **Edge validation**: Minimum 10 high-quality edge points required
- [ ] **Confidence scoring**: Minimum 90% confidence required
- [ ] **Center validation**: Maximum 3mm deviation from expected position
- [ ] **Failure modes**: System stops on any validation failure

## Monitoring and Maintenance

### Log Monitoring

```bash
# Monitor real-time logs
tail -f /var/log/diescan.log

# Check for safety violations
grep "SAFETY VIOLATION" /var/log/diescan.log

# Check scan success rate
grep "SCAN SUCCESS" /var/log/diescan.log | wc -l
```

### Performance Monitoring

```bash
# Check scan performance
diescan-prod scan --log-level DEBUG

# Monitor confidence scores
grep "Confidence:" /var/log/diescan.log | tail -20
```

### Maintenance Schedule

- **Daily**: Verify communication tests pass
- **Weekly**: Run calibration validation
- **Monthly**: Review scan confidence trends
- **Quarterly**: Full safety parameter validation

## Troubleshooting

### Common Issues

#### Communication Errors

```bash
# Robot connection issues
ERROR: Failed to connect to robot: [Errno 10061]

# Solutions:
# 1. Verify robot IP address
# 2. Check network connectivity: ping 192.168.0.1
# 3. Verify robot Ethernet/IP configuration
# 4. Check firewall settings
```

#### Sensor Issues

```bash
# Arduino communication errors  
ERROR: Failed to connect to sensor: [Errno 2]

# Solutions:
# 1. Verify COM port: Check Device Manager
# 2. Check Arduino power and connections
# 3. Verify baud rate (115200)
# 4. Test with serial terminal
```

#### Safety Violations

```bash
# Insufficient edge detection
SAFETY VIOLATION: Insufficient edge points: 7 < 10

# Solutions:
# 1. Check sensor calibration
# 2. Verify die position
# 3. Adjust scan parameters
# 4. DO NOT lower safety thresholds
```

### Emergency Procedures

#### If Scan Fails in Production

1. **STOP robot operation immediately**
2. **Do NOT use fallback values**
3. **Check error messages in logs**
4. **Verify sensor calibration**
5. **Re-run communication tests**
6. **Contact engineering if safety violations occur**

#### If Safety Violation Occurs

1. **ABORT current operation**
2. **Do NOT override safety systems**
3. **Document the violation**
4. **Investigate root cause**
5. **Engineering review required before restart**

## Support and Maintenance

### Package Updates

```bash
# Update package (test environment first)
git pull origin main
pip install -e ".[dev]"

# Run full test suite
pytest tests/

# Validate configuration
diescan-prod validate-config
```

### Emergency Contacts

- **Engineering Team**: [Contact Information]
- **Safety Officer**: [Contact Information]  
- **Robot Maintenance**: [Contact Information]

## Deployment Verification

After deployment, verify the following:

```bash
# 1. Package installation
diescan-prod --version

# 2. Configuration validation
diescan-prod validate-config

# 3. Communication testing
diescan-prod test-comms

# 4. Production mode verification
diescan-prod config | grep "Production Mode: True"

# 5. Safety parameter verification
diescan-prod config | grep -E "(Min Perimeter Edges: 10|Min Angular Coverage: 300|Min Confidence Score: 90)"
```

**Deployment is complete when ALL verification steps pass.**

---

## 🚨 CRITICAL SAFETY REMINDER 🚨

This system controls 2000°F forging operations. **NEVER** bypass safety validations or use fallback values. The package is designed to fail completely rather than provide potentially incorrect data that could cause operator injury or death.

**When in doubt, STOP operations and contact engineering.**