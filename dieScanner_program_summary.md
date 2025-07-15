# Die Scanner Program Summary

## Overview

The `dieScanner_keyence_dbGoodcalc.py` program is a comprehensive die scanning system that uses a Keyence laser sensor mounted on a FANUC robot to locate the center of a 4.5-inch diameter die for precision forging operations. The system performs continuous edge detection scanning with real-time robot position tracking and Arduino communication to calculate the die center with millimeter accuracy.

## Critical Safety Context

This system is designed for a 2000°F forging operation where incorrect die center calculations could result in **operator fatalities** if the hot part is misplaced during stamping press operations. The program includes multiple validation layers but currently uses fallback values that pose safety risks.

## Key Classes and Architecture

### Data Models
- **`ScanPoint`**: Individual measurement point with position (x,y,z), height, validity flag, and timestamp
- **`EdgePoint`**: Detected edge with position (x,y), edge type (drop_off/rise/stable), and confidence score
- **`DieCenter`**: Final result containing center coordinates, diameter, average height, confidence, and edge points

### Core Classes

#### `DieScanRobotInterface` (extends `proxy_simple`)
- **Purpose**: FANUC robot communication using CPPPO Ethernet/IP protocol
- **Register Mapping**: 
  - R[86-96]: Command/status and position data
  - R[161-180]: Edge point coordinates (up to 8 points)
  - R[182-184]: Next position components
  - R[140-143]: Measurement system data
- **Key Methods**:
  - `connect()`: Establish robot connection
  - `read_robot_position()`: Get current TCP position
  - `_write_parameter()` / `_read_parameter()`: Register communication

#### `KeyenceArduinoInterface`
- **Purpose**: Arduino interface for Keyence laser sensor using `keyence_always_on` sketch
- **Communication**: Serial connection (COM8, 115200 baud)
- **Key Methods**:
  - `connect()`: Establish serial connection with Arduino
  - `read_height()`: Get current height measurement
  - `calibrate()`: Perform sensor calibration with 65mm reference
  - `send_command()`: Synchronous command/response communication

#### `ContinuousDieScanner` (Main Coordinator)
- **Purpose**: Orchestrates complete scanning workflow
- **Key Parameters**:
  - Expected die diameter: 114.3mm (4.5 inches)
  - Base height threshold: 2.0mm for edge detection
  - Minimum outer radius: 45mm, Maximum: 70mm
  - Interior exclusion radius: 35mm
  - Minimum angular coverage: 225° (reduced for testing)
  - Minimum perimeter edges: 5 (reduced for testing)

## Main Scanning Workflow

### Phase 1: System Setup
1. **System Connection**: Connect to robot (192.168.0.1) and Arduino
2. **Sensor Pickup**: Robot picks up sensor jig (command 1)
3. **Position Setup**: Move to scan start position (command 2)
4. **Calibration**: Calibrate sensor at known reference position (65mm)

### Phase 2: 5-Pass Y-Direction Scanning
**Pattern**: Start +5" X, +5" Y from calibration position
1. **Pass 1**: Scan 5" in -Y direction
2. **Pass 2**: Move -0.25" X, scan 5" in +Y direction  
3. **Pass 3**: Move -0.25" X, scan 5" in -Y direction
4. **Pass 4**: Move -2.25" X, scan 5" in +Y direction
5. **Pass 5**: Move -0.25" X, scan 5" in -Y direction

### Phase 3: Cross-Direction X-Scanning
- **Adaptive positioning** based on preliminary center from Y-scanning
- **6-pass X-direction** scanning (3 right + 3 left of center)
- **Focused scanning** around detected edge radius

### Phase 4: Advanced Edge Refinement
1. **Mini-spiral refinement** around detected edges
2. **Coverage gap analysis** to fill missing perimeter areas
3. **Quality validation** and outlier removal

### Phase 5: Center Calculation
**Multi-method approach** with consensus algorithm:
1. **RANSAC circle fitting** (40% weight) - robust to outliers
2. **Weighted least squares** (30% weight) - uses edge quality scores
3. **Taubin circle fitting** (20% weight) - geometric accuracy
4. **Kasa least squares** (10% weight) - backup method

## Edge Detection Algorithm

### Adaptive Thresholding
- **Base threshold**: 2.0mm height change
- **Adaptive factor**: 1.5x local variance
- **Multi-scale detection**: Tests multiple threshold scales (1.0x, 1.5x, 2.0x)

### Edge Quality Scoring
- **Height difference** (40% weight): Magnitude of height change
- **Gradient strength** (30% weight): Sharpness of transition
- **Geometric consistency** (20% weight): Radius and circular pattern fit
- **Noise assessment** (10% weight): Local measurement stability

### Edge Classification
- **Interior edges**: < 35mm from center (filtered out)
- **Outer edges**: 45-70mm from center (used for center calculation)
- **Enhanced classification**: Multi-point confirmation with 3 consecutive points

## Safety Features and Validation

### Current Safety Measures
- **Minimum edge count**: Requires at least 5 outer edges
- **Angular coverage**: Requires minimum 225° perimeter coverage
- **Distance filtering**: Removes edges > 63.5mm from preliminary center
- **Confidence scoring**: Multi-method consensus with confidence percentage
- **Edge quality**: Filters low-quality edges below 0.6 threshold

### Critical Safety Gaps
- **Hardcoded fallbacks**: Uses fixed center coordinates (398.0, 809.0) when insufficient data
- **Reduced thresholds**: Minimum requirements lowered for testing
- **Insufficient validation**: No complete failure mode for unsafe calculations

## Robot Communication Protocol

### Command/Status Flow
- **Command codes**: 1=pickup, 2=move, 5=scan, 7=final position
- **Status codes**: 0=complete, 10=ready, 11=move request, 12=scan complete, 3=error
- **Position system**: Robot calculates LPOS on position request (R[185])

### Register Usage
- **R[86-96]**: Primary communication registers
- **R[161-180]**: Edge point coordinates for robot use
- **R[182-184]**: Next position components for robot movement
- **R[140-143]**: Measurement system data

## Dependencies and Requirements

### Python Dependencies
- **cpppo**: Ethernet/IP communication with FANUC robot
- **numpy**: Numerical computations and array operations
- **scipy**: Circle fitting algorithms (optional)
- **serial**: Arduino communication
- **matplotlib**: Debug visualization and plotting
- **threading**: Background position tracking

### Hardware Requirements
- **FANUC robot** with Ethernet/IP capability (IP: 192.168.0.1)
- **Arduino** with Keyence sensor interface (COM8, 115200 baud)
- **Keyence laser sensor** with analog output
- **Robot-mounted sensor jig** for positioning

## Performance Characteristics

### Scan Parameters
- **Scan step size**: 1.2mm initial, 0.5mm for refinement
- **Scan speed**: ~100-500Hz measurement rate
- **Total scan time**: ~30 seconds for complete 5-pass sequence
- **Coverage area**: ~120mm x 120mm scan envelope

### Accuracy Targets
- **Target accuracy**: ±1mm center location
- **Acceptable accuracy**: ±2mm center location
- **Die diameter**: 114.3mm (4.5 inches)
- **Expected radius**: 57.15mm

## Debug and Visualization Features

### Data Export
- **CSV export**: Complete scan data with timestamps
- **Heatmap generation**: Surface visualization for analysis
- **Console output**: Comprehensive results display
- **ASCII visualization**: Terminal-based surface map

### Logging and Monitoring
- **Height tracking**: Rolling buffer for variance calculation
- **Edge confirmation**: Multi-point validation system
- **Coverage analysis**: Perimeter coverage and density metrics
- **Error tracking**: Invalid reading statistics

## Program Entry Points

### Main Execution
- **`main()`**: Entry point for debug scanning
- **`run_debug_scan()`**: Complete scanning sequence
- **`continuous_scan_worker()`**: Core scanning logic

### Key Workflow Methods
- **`perform_new_5_pass_scan()`**: Execute 5-pass Y-direction scanning
- **`perform_cross_direction_scan()`**: X-direction validation scanning
- **`calculate_final_center()`**: Multi-method center calculation
- **`write_results_to_robot()`**: Output results to robot registers

## Current Issues and Limitations

### Safety Concerns
- **Hardcoded fallbacks** pose operator safety risks
- **Reduced validation thresholds** for testing purposes
- **No complete failure modes** for unsafe calculations

### Performance Issues
- **Matplotlib imports** slow initialization
- **CSV export** in production environment
- **Debug logging** clutters operator interface
- **Unused code** affects performance

### Architectural Issues
- **Monolithic design** makes maintenance difficult
- **Mixed responsibilities** in single classes
- **No proper error handling** for production use
- **Threading complexity** with position tracking

This program requires significant refactoring for production use with proper safety validation, modular architecture, and elimination of debug features that could compromise operator safety in the 2000°F forging environment.