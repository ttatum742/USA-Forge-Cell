/*
 * Keyence Laser Sensor Interface - Always On Version
 * 
 * This Arduino sketch interfaces with a Keyence laser displacement sensor
 * providing analog 0-5V output. The sensor is always powered on.
 * The Arduino acts as an ADC and USB interface for the industrial PC.
 * 
 * Hardware Connections:
 * - Analog Pin A0: Laser sensor analog output (0-5V)
 * - Digital Pin 2: Relay control for sensor zeroing
 * - USB: Communication with industrial PC
 * - Sensor Power: Connected directly to external power supply (always on)
 * 
 * Communication Protocol:
 * - Commands from PC: "READ", "STATUS", "CALIBRATE", "SET_FILTER"
 * - Responses to PC: "HEIGHT:XXX.X", "STATUS:...", "CALIBRATE_OK", "FILTER_OK"
 */

// Pin definitions
const int LASER_ANALOG_PIN = A0;    // Analog input from laser sensor
const int SENSOR_ZERO_PIN = 2;      // Digital output to control sensor zeroing relay

// Measurement parameters
const float VOLTAGE_REF = 5.0;      // Arduino reference voltage
const int ADC_RESOLUTION = 1024;    // 10-bit ADC resolution

// Sensor calibration parameters (ref sensor documentation)
const float SENSOR_MIN_DISTANCE = 55.0;  // mm - minimum measurement range
const float SENSOR_MAX_DISTANCE = 105.0; // mm - maximum measurement range
const float VOLTAGE_MIN = 0.0;            // V - voltage at max distance
const float VOLTAGE_MAX = 5.0;            // V - voltage at min distance

// Communication variables
String inputCommand = "";
bool commandReady = false;

// Measurement filtering - DISABLED (sensor has built-in filtering)
// Keyence sensor already provides averaged and filtered output
bool useArduinoFiltering = false;   // Set to true only if needed for debugging

// Calibration variables
bool calibrationMode = false;
float calibrationOffset = 0.0;     // Offset adjustment from calibration

// Performance monitoring
unsigned long lastMeasurementTime = 0;
unsigned long measurementCount = 0;
float measurementRate = 0.0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.setTimeout(100);
  
  // Initialize pins
  pinMode(LASER_ANALOG_PIN, INPUT);
  pinMode(SENSOR_ZERO_PIN, OUTPUT);
  digitalWrite(SENSOR_ZERO_PIN, LOW);  // Initially off
  
  // Set ADC reference to external 5V
  analogReference(DEFAULT);
  
  // No filter initialization needed - using sensor's built-in filtering
  
  Serial.println("Keyence Laser Interface Ready - Always On Mode");
  Serial.println("Sensor is always powered and ready for measurements");
  Serial.println("Commands: READ, STATUS, CALIBRATE, SET_FILTER [1-10]");
  
  // Start measurement rate tracking
  lastMeasurementTime = millis();
}

void loop() {
  // Check for incoming commands - only process when complete line is available
  if (Serial.available()) {
    inputCommand = Serial.readStringUntil('\n');
    inputCommand.trim();
    
    // Only process if we actually received a command (not empty)
    if (inputCommand.length() > 0) {
      commandReady = true;
    }
  }
  
  // Process commands
  if (commandReady) {
    processCommand(inputCommand);
    commandReady = false;
    inputCommand = ""; // Clear the command
  }
  
  delay(5); // Small delay to prevent overwhelming the serial port
}

void processCommand(String command) {
  if (command == "READ") {
    float distance = readLaserDistance();
    Serial.print("HEIGHT:");
    Serial.println(distance, 1);
    
    // Update measurement rate
    measurementCount++;
    updateMeasurementRate();
  }
  else if (command == "STATUS") {
    reportStatus();
  }
  else if (command == "CALIBRATE") {
    performCalibration();
  }
  else if (command.startsWith("SET_FILTER")) {
    Serial.println("FILTER:DISABLED - Sensor provides built-in filtering");
  }
  else if (command == "RESET") {
    resetSystem();
  }
  else if (command == "INFO") {
    printSystemInfo();
  }
  else {
    Serial.println("ERROR:UNKNOWN_COMMAND");
    Serial.println("Available commands: READ, STATUS, CALIBRATE, SET_FILTER [1-10], RESET, INFO");
  }
}

// Filter functions removed - using sensor's built-in filtering

float readLaserDistance() {
  // Read analog value
  int rawValue = analogRead(LASER_ANALOG_PIN);
  
  // Convert to voltage
  float voltage = (rawValue * VOLTAGE_REF) / ADC_RESOLUTION;
  
  // Convert voltage to distance
  float distance = voltageToDistance(voltage);
  
  // Apply calibration offset
  distance += calibrationOffset;
  
  // No Arduino-side filtering - sensor already provides filtered output
  // This gives immediate, responsive readings without lag
  
  return distance;
}

float voltageToDistance(float voltage) {
  // Check if voltage is within valid range
  if (voltage < VOLTAGE_MIN || voltage > VOLTAGE_MAX) {
    return -999.0; // Out of range
  }
  
  // Linear interpolation between min and max
  // Note: Higher voltage = closer distance for most Keyence sensors
  float normalizedVoltage = (voltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN);
  float distance = SENSOR_MAX_DISTANCE - (normalizedVoltage * (SENSOR_MAX_DISTANCE - SENSOR_MIN_DISTANCE));
  
  return distance;
}

// applyFilter() function removed - using sensor's built-in filtering

void reportStatus() {
  int rawValue = analogRead(LASER_ANALOG_PIN);
  float voltage = (rawValue * VOLTAGE_REF) / ADC_RESOLUTION;
  float distance = readLaserDistance();
  
  Serial.print("STATUS:");
  Serial.print("POWER=ALWAYS_ON");
  Serial.print(",RAW=");
  Serial.print(rawValue);
  Serial.print(",VOLTAGE=");
  Serial.print(voltage, 3);
  Serial.print(",DISTANCE=");
  Serial.print(distance, 1);
  Serial.print(",FILTER=DISABLED");
  Serial.print(",RATE=");
  Serial.print(measurementRate, 1);
  Serial.print(",OFFSET=");
  Serial.print(calibrationOffset, 2);
  Serial.println();
}

void performCalibration() {
  Serial.println("CALIBRATION:STARTING");
  Serial.println("Place reference object at known distance and send distance in mm");
  Serial.println("Format: CAL_DISTANCE [value] or send CANCEL to abort");
  
  calibrationMode = true;
  unsigned long calibrationStart = millis();
  
  while (calibrationMode && (millis() - calibrationStart < 30000)) { // 30 second timeout
    if (Serial.available()) {
      String calCommand = Serial.readStringUntil('\n');
      calCommand.trim();
      
      if (calCommand == "CANCEL") {
        calibrationMode = false;
        Serial.println("CALIBRATION:CANCELLED");
        return;
      }
      else if (calCommand.startsWith("CAL_DISTANCE")) {
        // Parse the reference distance
        int spaceIndex = calCommand.indexOf(' ');
        if (spaceIndex > 0) {
          float referenceDistance = calCommand.substring(spaceIndex + 1).toFloat();
          
          if (referenceDistance > 0) {
            // Take multiple measurements for calibration
            float totalMeasured = 0;
            int validMeasurements = 0;
            
            Serial.println("Zeroing sensor...");
            zeroSensor();
            
            Serial.println("Taking calibration measurements...");
            
            for (int i = 0; i < 20; i++) {
              float measured = readLaserDistance();
              if (measured > -999) {
                totalMeasured += measured;
                validMeasurements++;
              }
              delay(50);
            }
            
            if (validMeasurements > 10) {
              float averageMeasured = totalMeasured / validMeasurements;
              calibrationOffset = referenceDistance - averageMeasured;
              // No filter to reinitialize - immediate calibration effect
              
              Serial.print("CALIBRATION:COMPLETE,OFFSET=");
              Serial.println(calibrationOffset, 2);
              
              calibrationMode = false;
              return;
            } else {
              Serial.println("CALIBRATION:ERROR - Insufficient valid measurements");
            }
          } else {
            Serial.println("CALIBRATION:ERROR - Invalid reference distance");
          }
        } else {
          Serial.println("CALIBRATION:ERROR - Invalid format. Use: CAL_DISTANCE [value]");
        }
      }
    }
    delay(100);
  }
  
  if (calibrationMode) {
    Serial.println("CALIBRATION:TIMEOUT");
    calibrationMode = false;
  }
}

// setFilterSamples() function removed - filtering disabled

void resetSystem() {
  // Reset calibration
  calibrationOffset = 0.0;
  
  // Reset measurement tracking
  measurementCount = 0;
  measurementRate = 0.0;
  lastMeasurementTime = millis();
  
  Serial.println("RESET:COMPLETE");
}

void updateMeasurementRate() {
  unsigned long currentTime = millis();
  unsigned long timeDiff = currentTime - lastMeasurementTime;
  
  // Update rate every 1000 measurements or every 5 seconds
  if (measurementCount % 1000 == 0 || timeDiff > 5000) {
    if (timeDiff > 0) {
      measurementRate = (measurementCount * 1000.0) / timeDiff;
    }
    lastMeasurementTime = currentTime;
    measurementCount = 0;
  }
}

void printSystemInfo() {
  Serial.println("SYSTEM_INFO:");
  Serial.println("===================");
  Serial.println("Sensor Type: Keyence Laser (Always On)");
  Serial.print("ADC Resolution: ");
  Serial.print(ADC_RESOLUTION);
  Serial.println(" bits");
  Serial.print("Reference Voltage: ");
  Serial.print(VOLTAGE_REF);
  Serial.println(" V");
  Serial.print("Measurement Range: ");
  Serial.print(SENSOR_MIN_DISTANCE);
  Serial.print(" - ");
  Serial.print(SENSOR_MAX_DISTANCE);
  Serial.println(" mm");
  Serial.print("Voltage Range: ");
  Serial.print(VOLTAGE_MIN);
  Serial.print(" - ");
  Serial.print(VOLTAGE_MAX);
  Serial.println(" V");
  Serial.println("Filter: DISABLED (using sensor built-in filtering)");
  Serial.print("Calibration Offset: ");
  Serial.print(calibrationOffset);
  Serial.println(" mm");
  Serial.print("Measurement Rate: ");
  Serial.print(measurementRate);
  Serial.println(" Hz");
  Serial.println("===================");
}

void zeroSensor() {
  // Pulse the sensor zeroing relay for 60ms
  digitalWrite(SENSOR_ZERO_PIN, HIGH);
  delay(60);
  digitalWrite(SENSOR_ZERO_PIN, LOW);
  
  // Wait for sensor to stabilize after zeroing
  delay(100);
  
  Serial.println("Sensor zeroing complete");
}