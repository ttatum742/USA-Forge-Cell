USA FORGE CELL - FANUC Robot Force Control System
===================================================

OVERVIEW
--------
This is a FANUC robot force control system for a dual-furnace forging cell. 
The system combines FANUC LS robot programs with Python applications that 
perform real-time torque monitoring and derivative calculation to enable 
force-sensitive operations without dedicated force sensors.

QUICK START
-----------
1. Setup Python environment:
   conda env create -f usa-forge-cell-env.yaml
   conda activate usa-forge-cell

2. Run the force control system:
   cd Programs/Python
   python torque_derivative.py

3. Or use batch automation (Windows):
   cd Programs/batch
   run_derivative_calc.bat

SYSTEM REQUIREMENTS
-------------------
- Python with conda environment
- FANUC robot controller at IP 192.168.0.1
- Ethernet/IP communication enabled
- Key Python dependencies: cpppo, numpy, matplotlib, scipy

ARCHITECTURE
------------
- Robot Programs: Programs/Fanuc/ (MAIN.LS, PUSHTOBACKSTOP.LS, etc.)
- Python Apps: Programs/Python/ (torque_derivative.py for production)
- Automation: Programs/batch/ (Windows batch files)

CRITICAL PARAMETERS
-------------------
- Robot IP: 192.168.0.1 (hardcoded)
- Update Rate: 100-500Hz
- Skip Condition: R[80:j1dt]<=(-800) for J1 torque derivative
- Filter Coefficient: alpha = 0.3

REGISTER MAPPING
----------------
- R[50-55]: Joint torques (read from robot)
- R[60-65]: Joint angles  
- R[70-72]: Cartesian forces (calculated)
- R[80-91]: Torque derivatives (written to robot)
- R[22-25]: Furnace state management

DEVELOPMENT TOOLS
-----------------
- *_charting.py files: Include real-time plotting for calibration
- network*.py files: Communication testing utilities
- roboguide06_16/: Historical/development robot programs

SAFETY NOTES
------------
- Test in simulation first (roboguide programs available)
- Use charting programs to visualize torque patterns before threshold changes
- Verify SAFETY_CHECK.LS functionality
- Implement gradual threshold changes

For detailed information, see CLAUDE.md in the project root.