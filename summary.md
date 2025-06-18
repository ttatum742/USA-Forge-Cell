# FANUC Robot Force Control Project

## Project Overview
This project implements advanced force control and monitoring capabilities for a FANUC industrial robot used in a dual-furnace forging cell. The system combines robot programs written in FANUC's LS language with Python-based monitoring and control applications that communicate via Ethernet/IP.

## Key Components

### 1. **Forging Cell Automation**
The robot handles a complete forging workflow:
- Picks parts from an input array
- Places parts in one of two available furnaces
- Pushes parts to a backstop using force-sensitive control
- Retrieves heated parts and places them in a forge press
- Outputs finished parts

### 2. **Force-Sensitive Push Operation**
The core innovation is the `PUSHTOBACKSTOP` program that uses real-time torque monitoring:
- Monitors joint torques via registers R[50-55]
- Python calculates torque derivatives (1st and 2nd order) in real-time
- Writes derivatives to R[80-91] for the robot to use as skip conditions
- Enables precise force control without dedicated force sensors

### 3. **Python Integration**
Multiple Python programs provide advanced capabilities:
- **torque_derivative.py**: High-speed (100-500Hz) derivative calculation
- **torque_to_force.py**: Converts joint torques to Cartesian forces using robot kinematics
- **Real-time plotting**: Various visualization tools for debugging and calibration
- **Network utilities**: CPPPO-based Ethernet/IP communication with the robot

### 4. **Dual-Furnace Optimization**
The system intelligently manages two furnaces to maximize throughput:
- Tracks furnace states (empty, loading, heating, ready)
- Prioritizes ready furnaces to minimize heat loss
- Loads the next part while another is heating
- Handles asynchronous furnace operations

## Technical Architecture

### Communication
- **Protocol**: Ethernet/IP using CPPPO library
- **Registers**: 
  - R[1-4]: Part parameters from PLC
  - R[50-55]: Joint torques
  - R[60-65]: Joint angles
  - R[70-72]: Cartesian forces
  - R[80-91]: Torque derivatives
- **Update Rate**: 10-500Hz depending on application

### Robot Kinematics
- DH parameters for FANUC M-10iD/16S robot
- Forward kinematics for position calculation
- Jacobian-based force transformation
- Tool frame force computation

### Signal Processing
- Low-pass filtering for noise reduction
- Moving average for torque smoothing
- Butterworth filters for derivative calculation
- Spike detection for collision avoidance

## Key Programs

### FANUC LS Programs
- **MAIN.LS**: Orchestrates the entire forging process
- **PUSHTOBACKSTOP.LS**: Force-sensitive pushing with skip conditions
- **PICK_*/PLACE_*.LS**: Material handling routines
- **TORQUE_MONITOR.LS**: Updates torque registers from system variables
- **CALC_POS.LS**: Calculates pick positions based on array geometry

### Python Applications
- **torque_derivative.py**: Production derivative calculator
- **torque_to_force_charting.py**: Development tool with real-time plots
- **2nd_derivative_charting.py**: Advanced derivative analysis
- **networkWrite_cpppo.py**: Register writing utilities

## Future Development
- Logging systems for process data collection
- Enhanced collision detection algorithms
- Multi-robot coordination
- Integration with factory MES systems
- Machine learning for optimal force profiles

## Setup Requirements
- FANUC robot with Ethernet/IP enabled
- Python 3.x with numpy, cpppo, matplotlib
- Robot registers configured as per registers_inputs_outputs.xlsx
- Proper network configuration (robot at 192.168.0.1)

This project demonstrates sophisticated integration between traditional industrial robotics and modern Python-based control systems, enabling capabilities typically requiring expensive force/torque sensors through clever software engineering.