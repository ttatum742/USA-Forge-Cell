# DieScanner Production Package

Production-ready die scanning system for FANUC robot forging cell operations.

## ⚠️ SAFETY CRITICAL SYSTEM ⚠️

This system is designed for 2000°F forging operations where incorrect calculations could result in **operator fatalities**. The package implements stringent safety validation with NO fallback values.

## Features

- Production-grade edge detection and center calculation
- Multi-method center calculation with consensus algorithm
- Comprehensive safety validation (NO fallbacks)
- FANUC robot communication via CPPPO Ethernet/IP
- Arduino/Keyence sensor interface
- Fast initialization for production use
- Operator-friendly logging interface

## Safety Requirements

- **Minimum 10 perimeter edges** required for production
- **Minimum 300° angular coverage** required for production  
- **Minimum 90% confidence score** required for production
- **Maximum 3mm center deviation** allowed for production
- **Complete failure on insufficient data** - NO hardcoded fallbacks

## Installation

```bash
# Install in production environment
pip install -e .

# Install with development dependencies
pip install -e ".[dev]"
```

## Usage

### Command Line Interface

```bash
# Run die scanner
diescan-prod scan

# Test communications
diescan-prod test-comms

# Show current configuration
diescan-prod config
```

### Python API

```python
from diescanprod import DieScannerProd
from diescanprod.config import Settings

# Initialize with production settings
settings = Settings()
scanner = DieScannerProd(settings)

# Run scan (will raise SafetyError if unsafe)
try:
    result = scanner.scan()
    print(f"Center: ({result.center_x:.1f}, {result.center_y:.1f})")
except SafetyError as e:
    print(f"SCAN FAILED - UNSAFE: {e}")
    # DO NOT proceed with robot operation
```

## Configuration

Set environment variables:

```bash
export ROBOT_IP=192.168.0.1
export ARDUINO_PORT=COM8
export PRODUCTION_MODE=true
export LOG_LEVEL=INFO
```

## Development

```bash
# Run tests
pytest

# Run linting
black src/ tests/
flake8 src/ tests/

# Type checking
mypy src/
```

## Architecture

- `core/`: Edge detection and center calculation algorithms
- `services/`: Robot and sensor communication
- `models/`: Data models with validation
- `config/`: Settings and safety parameters
- `utils/`: Geometric calculations and validation
- `exceptions/`: Custom exception hierarchy

## Safety Implementation

This package implements a "fail-safe" approach:

1. **No fallback values** - System fails completely on insufficient data
2. **Multiple validation layers** - Edge detection, center calculation, final validation
3. **Consensus algorithms** - Multiple methods must agree on results
4. **Operator alerts** - Clear error messages for unsafe conditions
5. **Production thresholds** - Higher requirements than debug version

## Communication Protocol

- **Robot**: FANUC R-30iB via Ethernet/IP (CPPPO)
- **Sensor**: Keyence via Arduino serial interface
- **Registers**: R[86-96] for commands/status, R[161-180] for results

## Support

This is a safety-critical system. Any modifications require engineering review and validation testing.