PROGRAM WeightCalculator
(* 
   Calculates approximate part weight for cylindrical parts
   Material: 4140 Steel (density = 0.284 lb/in³)
   
   Author: Thomas Tatum
   Date: 8/11/2025
   Version: 1.0
*)

VAR
    // Input Variables
    PartDiameter    : REAL := 0.0;      // Part diameter in inches
    PartLength      : REAL := 0.0;      // Part length in inches
    CalculateWeight : BOOL := FALSE;    // Trigger to calculate weight
    
    // Output Variables
    PartWeight      : REAL := 0.0;      // Calculated weight in pounds
    CalculationDone : BOOL := FALSE;    // Indicates calculation complete
    ErrorFlag       : BOOL := FALSE;    // Error flag for invalid inputs
    
    // Internal Variables
    Radius          : REAL := 0.0;      // Part radius (diameter/2)
    Volume          : REAL := 0.0;      // Part volume in cubic inches
    SteelDensity    : REAL := 0.284;    // 4140 steel density in lb/in³
    PI              : REAL := 3.14159265359;
    
    // Constants for validation
    MIN_DIAMETER    : REAL := 0.5;    // Minimum diameter (in)
    MAX_DIAMETER    : REAL := 1.5;    // Maximum diameter (in)
    MIN_LENGTH      : REAL := 2.5;    // Minimum length (in)
    MAX_LENGTH      : REAL := 12.0;   // Maximum length (in)
    
    // Status variables
    StatusMessage   : STRING[50] := 'Ready';
END_VAR

BEGIN
    // Reset outputs when not calculating
    IF NOT CalculateWeight THEN
        CalculationDone := FALSE;
        ErrorFlag := FALSE;
        StatusMessage := 'Ready';
    END_IF;
    
    // Main calculation logic
    IF CalculateWeight THEN
        // Input validation
        IF (PartDiameter < MIN_DIAMETER) OR (PartDiameter > MAX_DIAMETER) THEN
            ErrorFlag := TRUE;
            PartWeight := 0.0;
            StatusMessage := 'Error: Invalid diameter';
            CalculationDone := TRUE;
            
        ELSIF (PartLength < MIN_LENGTH) OR (PartLength > MAX_LENGTH) THEN
            ErrorFlag := TRUE;
            PartWeight := 0.0;
            StatusMessage := 'Error: Invalid length';
            CalculationDone := TRUE;
            
        ELSE
            // Valid inputs - perform calculation
            ErrorFlag := FALSE;
            
            // Calculate radius
            Radius := PartDiameter / 2.0;
            
            // Calculate volume (π × r² × length)
            Volume := PI * (Radius * Radius) * PartLength;
            
            // Calculate weight (volume × density)
            PartWeight := Volume * SteelDensity;
            
            // Set completion status
            CalculationDone := TRUE;
            StatusMessage := 'Calculation complete';
        END_IF;
    END_IF;
    
END_PROGRAM