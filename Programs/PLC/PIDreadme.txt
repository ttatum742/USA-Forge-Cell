==============================================================================
PID Temperature Control with Auto-Tuning for AutomationDirect P1000 PLC
==============================================================================
This program implements a PID controller with automatic tuning capabilities
for furnace temperature control using a 0-10V feedback signal.

HARDWARE REQUIREMENTS:
- Analog Input Module for 0-10V temperature sensor
- Analog Output Module for furnace control
- P1000 PLC with sufficient memory

I/O MAPPING:
- AI0: Temperature sensor (0-10V)
- AO0: Furnace control output (0-10V or 4-20mA)




==============================================================================
LADDER LOGIC EQUIVALENT FOR KEY FUNCTIONS
==============================================================================

RUNG 1: Auto-Tune Enable Logic
+--] [---+---] [---+---]/[---+---( )---+
   AutoTune  PID_Enable  AutoTune   AutoTune
   Request              Active     Enable

RUNG 2: PID Enable Logic  
+---]/[---+---]/[---+---] [---+---( )---+
   AutoTune  Manual   PID_Timer   PID_Enable
   Active    Mode     .Q

RUNG 3: High Temp Alarm
+---] [---+---] [---+---( )---+
   Temp>Max    NOT      High_Temp
   *0.9     Sensor_Fault  Alarm

RUNG 4: Safety Output Control
+---] [---+---( )---+
   Sensor    Output
   Fault     Reset
   
RUNG 5: Manual Mode Output
+---] [---+---[MOVE]---+
   Manual    Manual_Output
   Mode      -> Control_Output

==============================================================================
CONFIGURATION NOTES:
==============================================================================

1. Analog Input Configuration:
   - Configure AI0 for 0-10V input
   - Set scaling for your temperature range
   - Enable input filtering if available

2. Analog Output Configuration:
   - Configure AO0 for appropriate output (0-10V or 4-20mA)
   - Set proper scaling for your furnace control

3. Auto-Tuning Procedure:
   - Set desired setpoint
   - Enable AutoTune_Enable
   - Monitor AutoTune_Active status
   - Wait for AutoTune_Complete
   - PID parameters will be automatically calculated

4. Tuning Parameters:
   - Relay_Output: Adjust based on system response (typically 10-30%)
   - Scan_Time: Adjust based on process dynamics (100ms to 1s typical)
   - Safety limits and alarms as per process requirements

5. Safety Considerations:
   - Implement proper interlocks
   - Set appropriate temperature limits
   - Monitor for sensor failures
   - Include emergency stop functionality