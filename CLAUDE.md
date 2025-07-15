# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a FANUC robot force control system for a dual-furnace forging cell. The system combines FANUC LS robot programs with Python applications that perform real-time torque monitoring and derivative calculation to enable force-sensitive operations without dedicated force sensors.

## Environment Setup

### Python Environment
```bash
# Create conda environment from yaml file
conda env create -f usa-forge-cell-env.yaml
conda activate usa-forge-cell

# Or use batch file for quick Python setup (Windows)
cd Programs/batch
run_derivative_calc.bat
```

### Key Dependencies
- **cpppo**: Ethernet/IP communication with FANUC robot
- **numpy**: Numerical computations for torque derivatives
- **matplotlib**: Real-time plotting for debugging/calibration
- **scipy**: Signal processing filters

## System Architecture

### Communication Protocol
- **Robot IP**: 192.168.0.1 (hardcoded in Python applications)
- **Protocol**: Ethernet/IP via CPPPO library
- **Update Rate**: 100-500Hz for real-time torque monitoring
- **Register Mapping**:
  - R[50-55]: Joint torques (read from robot)
  - R[60-65]: Joint angles
  - R[70-72]: Cartesian forces (calculated)
  - R[80-91]: Torque derivatives (written to robot)
  - All registers and position register assignments shall be maintained and recorded in the relevant csv file, found in the registers_io folder

### Robot Program Structure
- **MAIN.LS**: Master program orchestrating dual-furnace workflow
- **PUSHTOBACKSTOP.LS**: Core force-sensitive operation using skip conditions on R[80] (J1 torque derivative)
- **Support programs**: Handle material movement, furnace coordination, safety checks

### Python Application Architecture
- **torque_derivative.py**: Production system for real-time derivative calculation
- **torque_to_force*.py**: Convert joint torques to Cartesian forces using robot kinematics
- ***_charting.py**: Development tools with live plotting for calibration

## Common Development Tasks

### Running the Force Control System
```bash
# Production derivative calculator
cd Programs/Python
python torque_derivative.py

# Development with real-time plotting
python torque_to_force_charting.py

# Using batch automation (Windows)
cd Programs/batch
run_derivative_calc.bat
```

### Testing Communication
```bash
# Test robot connection
python network test.py

# Test register read/write
python networkRead_cpppo.py
python networkWrite_cpppo.py
```

### Torque Analysis and Calibration
```bash
# Generate derivative plots for tuning
python derivative_charting.py
python 2nd_derivative_charting.py

# Force analysis
python torque_to_force.py
```

## Critical System Parameters

### Skip Conditions (PUSHTOBACKSTOP.LS)
- Primary: `R[80:j1dt]<=(-800)` - J1 torque derivative threshold
- Secondary: `R[91:j6d2t]>=200` - J6 second derivative (commented out)

### Torque Derivative Filtering
- Low-pass filter coefficient: `alpha = 0.3`
- Default update rate: 500Hz
- Torques filtered BEFORE differentiation to reduce noise

### Furnace State Management
- R[22]: Furnace 1 state (0=empty, 1=loading, 2=heating, 3=ready)
- R[23]: Furnace 2 state
- R[24]: Priority furnace (ready first)
- R[25]: Available furnace (can accept part)

## File Organization

### Robot Programs (`Programs/Fanuc/`)
- Production programs in root directory
- `roboguide06_16/`: Historical/development versions

### Python Applications (`Programs/Python/`)
- Production: `torque_derivative.py`
- Development: Files with `_charting` suffix include real-time plotting
- Network utilities: `network*.py` files for communication testing

### Automation (`Programs/batch/`)
- `run_derivative_calc.bat`: Automated Python environment setup and execution
- `run_torque_plotter.bat`: Batch plotting utilities

## Robot Safety Considerations

When modifying skip conditions or torque thresholds:
1. Test in simulation first (roboguide programs available)
2. Use development charting programs to visualize torque patterns
3. Implement gradual threshold changes
4. Always verify safety checks in SAFETY_CHECK.LS are functioning

## Network Configuration

Robot controller must be configured for:
- Ethernet/IP communication enabled
- IP address: 192.168.0.1
- Registers R[50-91] configured for external read/write access
- System variables $TORQUE[1-6] mapped to R[50-55]