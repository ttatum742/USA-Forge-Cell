PROGRAM PID_Temperature_Control
VAR
    // Process Variables
    Temperature_Raw     : INT;          // Raw ADC value from sensor
    Temperature_Scaled  : REAL;         // Scaled temperature value
    Setpoint           : REAL := 500.0; // Temperature setpoint
    Control_Output     : REAL;          // PID output (0-100%)
    Output_Raw         : INT;           // Raw DAC value for output
    
    // PID Parameters
    Kp                 : REAL := 1.0;   // Proportional gain
    Ki                 : REAL := 0.1;   // Integral gain
    Kd                 : REAL := 0.01;  // Derivative gain
    
    // PID Internal Variables
    Error              : REAL;          // Current error
    Error_Previous     : REAL;          // Previous error
    Integral           : REAL;          // Integral accumulator
    Derivative         : REAL;          // Derivative term
    Output_Min         : REAL := 0.0;   // Minimum output limit
    Output_Max         : REAL := 100.0; // Maximum output limit
    
    // Auto-Tuning Variables
    AutoTune_Enable    : BOOL := FALSE; // Enable auto-tuning
    AutoTune_Active    : BOOL := FALSE; // Auto-tuning in progress
    AutoTune_Complete  : BOOL := FALSE; // Auto-tuning completed
    AutoTune_Step      : INT := 0;      // Auto-tuning step counter
    AutoTune_Timer     : TON;           // Auto-tuning timer
    
    // Relay Auto-Tuning Parameters
    Relay_Output       : REAL := 20.0;  // Relay amplitude (% of full scale)
    Oscillation_Count  : INT := 0;      // Number of oscillations detected
    Peak_Values        : ARRAY[1..10] OF REAL; // Store peak values
    Valley_Values      : ARRAY[1..10] OF REAL; // Store valley values
    Time_Values        : ARRAY[1..10] OF TIME; // Store time values
    Peak_Count         : INT := 0;      // Peak counter
    Valley_Count       : INT := 0;      // Valley counter
    
    // Ultimate Gain and Period Calculation
    Ultimate_Gain      : REAL;          // Ku - Ultimate gain
    Ultimate_Period    : REAL;          // Pu - Ultimate period
    
    // Slope detection for peak/valley finding
    Slope_Previous     : REAL;          // Previous slope
    Slope_Current      : REAL;          // Current slope
    
    // System Control
    PID_Enable         : BOOL := TRUE;  // Enable PID control
    Manual_Mode        : BOOL := FALSE; // Manual control mode
    Manual_Output      : REAL := 0.0;   // Manual output value
    
    // Scaling Parameters
    Temp_Min           : REAL := 0.0;   // Minimum temperature (0V)
    Temp_Max           : REAL := 1000.0;// Maximum temperature (10V)
    
    // Timing
    Scan_Time          : TIME := T#100MS; // PID scan time
    PID_Timer          : TON;
    
    // Status and Alarms
    High_Temp_Alarm    : BOOL;          // High temperature alarm
    Low_Temp_Alarm     : BOOL;          // Low temperature alarm
    Sensor_Fault       : BOOL;          // Sensor fault detection
    
END_VAR

// Main Program Execution
BEGIN

//==============================================================================
// INPUT SCALING AND CONDITIONING
//==============================================================================

// Read analog input (0-10V sensor)
Temperature_Raw := %IW0;  // Assuming 12-bit ADC (0-4095)

// Scale to engineering units
Temperature_Scaled := (REAL(Temperature_Raw) / 4095.0) * (Temp_Max - Temp_Min) + Temp_Min;

// Sensor fault detection (check for open circuit or short)
IF Temperature_Raw < 10 OR Temperature_Raw > 4085 THEN
    Sensor_Fault := TRUE;
ELSE
    Sensor_Fault := FALSE;
END_IF;

//==============================================================================
// AUTO-TUNING ALGORITHM 
//==============================================================================

CASE AutoTune_Step OF
    
    0: // Initialize Auto-Tuning
        IF AutoTune_Enable AND NOT AutoTune_Active THEN
            AutoTune_Active := TRUE;
            AutoTune_Complete := FALSE;
            AutoTune_Step := 1;
            Oscillation_Count := 0;
            Peak_Count := 0;
            Valley_Count := 0;
            Integral := 0.0;  // Reset integral
            PID_Enable := FALSE; // Disable normal PID
        END_IF;
    
    1: // Apply Relay Output
        IF AutoTune_Active THEN
            // Simple relay with hysteresis
            IF Temperature_Scaled < Setpoint THEN
                Control_Output := 50.0 + Relay_Output; // Above setpoint
            ELSE
                Control_Output := 50.0 - Relay_Output; // Below setpoint
            END_IF;
            
            // Detect peaks and valleys
            Slope_Current := Temperature_Scaled - Error_Previous;
            
            // Peak detection (slope changes from positive to negative)
            IF Slope_Previous > 0.1 AND Slope_Current < -0.1 AND Peak_Count < 10 THEN
                Peak_Count := Peak_Count + 1;
                Peak_Values[Peak_Count] := Temperature_Scaled;
                Time_Values[Peak_Count] := TIME();
            END_IF;
            
            // Valley detection (slope changes from negative to positive)
            IF Slope_Previous < -0.1 AND Slope_Current > 0.1 AND Valley_Count < 10 THEN
                Valley_Count := Valley_Count + 1;
                Valley_Values[Valley_Count] := Temperature_Scaled;
            END_IF;
            
            Slope_Previous := Slope_Current;
            
            // Check if we have enough oscillations
            IF Peak_Count >= 4 AND Valley_Count >= 4 THEN
                AutoTune_Step := 2;
            END_IF;
            
            // Safety timeout
            AutoTune_Timer(IN := TRUE, PT := T#600S); // 10 minute timeout
            IF AutoTune_Timer.Q THEN
                AutoTune_Step := 99; // Timeout error
            END_IF;
        END_IF;
    
    2: // Calculate Ultimate Parameters
        // Calculate average amplitude
        Ultimate_Gain := (4.0 * Relay_Output) / 
                        (3.14159 * ABS(Peak_Values[Peak_Count] - Valley_Values[Valley_Count]));
        
        // Calculate period from time between peaks
        IF Peak_Count >= 2 THEN
            Ultimate_Period := REAL(TIME_TO_DINT(Time_Values[Peak_Count] - Time_Values[Peak_Count-1])) / 1000.0;
        END_IF;
        
        AutoTune_Step := 3;
    
    3: // Apply Ziegler-Nichols Tuning Rules
        // Ziegler-Nichols PID tuning
        Kp := 0.6 * Ultimate_Gain;
        Ki := (2.0 * Kp) / Ultimate_Period;
        Kd := (Kp * Ultimate_Period) / 8.0;
        
        // Complete auto-tuning
        AutoTune_Active := FALSE;
        AutoTune_Complete := TRUE;
        AutoTune_Enable := FALSE;
        PID_Enable := TRUE; // Re-enable PID control
        AutoTune_Step := 0;
    
    99: // Error state
        AutoTune_Active := FALSE;
        AutoTune_Complete := FALSE;
        AutoTune_Enable := FALSE;
        PID_Enable := TRUE;
        AutoTune_Step := 0;

END_CASE;

//==============================================================================
// PID CONTROL ALGORITHM
//==============================================================================

// Execute PID control at regular intervals
PID_Timer(IN := TRUE, PT := Scan_Time);

IF PID_Timer.Q AND PID_Enable AND NOT Manual_Mode AND NOT Sensor_Fault THEN
    PID_Timer(IN := FALSE);
    
    // Calculate error
    Error := Setpoint - Temperature_Scaled;
    
    // Proportional term
    // P_Term := Kp * Error  (calculated in output equation)
    
    // Integral term with windup protection
    Integral := Integral + (Ki * Error * REAL(TIME_TO_DINT(Scan_Time)) / 1000.0);
    
    // Integral windup protection
    IF Integral > Output_Max THEN
        Integral := Output_Max;
    ELSIF Integral < Output_Min THEN
        Integral := Output_Min;
    END_IF;
    
    // Derivative term (derivative on measurement to avoid setpoint kicks)
    Derivative := Kd * (Error - Error_Previous) / (REAL(TIME_TO_DINT(Scan_Time)) / 1000.0);
    
    // Calculate PID output
    Control_Output := (Kp * Error) + Integral + Derivative;
    
    // Output limiting
    IF Control_Output > Output_Max THEN
        Control_Output := Output_Max;
    ELSIF Control_Output < Output_Min THEN
        Control_Output := Output_Min;
    END_IF;
    
    // Store previous error for next calculation
    Error_Previous := Error;
END_IF;

// Manual mode override
IF Manual_Mode THEN
    Control_Output := Manual_Output;
END_IF;

//==============================================================================
// OUTPUT SCALING AND CONTROL
//==============================================================================

// Scale output to DAC range (assuming 12-bit DAC, 0-4095)
Output_Raw := REAL_TO_INT((Control_Output / 100.0) * 4095.0);

// Write to analog output
%QW0 := Output_Raw;

//==============================================================================
// ALARMS AND SAFETY
//==============================================================================

// High temperature alarm
High_Temp_Alarm := Temperature_Scaled > (Temp_Max * 0.9);

// Low temperature alarm  
Low_Temp_Alarm := Temperature_Scaled < (Temp_Min + 50.0);

// Safety shutdown on sensor fault
IF Sensor_Fault THEN
    Control_Output := 0.0;
    Output_Raw := 0;
    %QW0 := 0;
END_IF;

END_PROGRAM