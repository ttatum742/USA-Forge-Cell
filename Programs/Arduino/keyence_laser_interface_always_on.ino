/*
 * Keyence Laser Sensor Interface - Always On Version
 * 
 * This Arduino sketch interfaces with a Keyence laser displacement sensor
 * providing analog 0-5V output. The sensor is always powered on.
 * The Arduino acts as an ADC and USB interface for the industrial PC.
 * 
 * Hardware Connections:
 * - Analog Pin A0: Laser sensor analog output (0-5V)
 * - USB: Communication with industrial PC
 * - Sensor Power: Connected directly to external power supply (always on)
 * 
 * Communication Protocol:
 * - Commands from PC: "READ", "STATUS", "CALIBRATE", "SET_FILTER"
 * - Responses to PC: "HEIGHT:XXX.X", "STATUS:...", "CALIBRATE_OK", "FILTER_OK"
 */

// Pin definitions
const int LASER_ANALOG_PIN = A0;    // Analog input from laser sensor

// Measurement parameters
const float VOLTAGE_REF = 5.0;      // Arduino reference voltage
const int ADC_RESOLUTION = 1024;    // 10-bit ADC resolution

// Sensor calibration parameters (adjust based on your Keyence sensor)
const float SENSOR_MIN_DISTANCE = 10.0;  // mm - minimum measurement range
const float SENSOR_MAX_DISTANCE = 100.0; // mm - maximum measurement range
const float VOLTAGE_MIN = 0.5;            // V - voltage at max distance
const float VOLTAGE_MAX = 4.5;            // V - voltage at min distance

// Communication variables
String inputCommand = "";
bool commandReady = false;

// Measurement filtering
int filterSamples = 5;              // Number of samples for averaging (adjustable)
float lastMeasurements[10];         // Buffer for up to 10 samples
int filterIndex = 0;
bool filterInitialized = false;

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
  
  // Initialize filter array
  for (int i = 0; i < 10; i++) {
    lastMeasurements[i] = 0;
  }
  
  // Set ADC reference to external 5V
  analogReference(DEFAULT);
  
  // Initialize filter with current readings
  initializeFilter();
  
  Serial.println("Keyence Laser Interface Ready - Always On Mode");
  Serial.println("Sensor is always powered and ready for measurements");
  Serial.println("Commands: READ, STATUS, CALIBRATE, SET_FILTER [1-10]");
  
  // Start measurement rate tracking
  lastMeasurementTime = millis();
}

void loop() {
  // Check for incoming commands
  if (Serial.available()) {
    inputCommand = Serial.readStringUntil('\n');
    inputCommand.trim();
    commandReady = true;
  }
  
  // Process commands
  if (commandReady) {
    processCommand(inputCommand);
    commandReady = false;
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
    setFilterSamples(command);
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

void initializeFilter() {
  // Fill filter buffer with initial readings
  for (int i = 0; i < filterSamples; i++) {
    int rawValue = analogRead(LASER_ANALOG_PIN);
    float voltage = (rawValue * VOLTAGE_REF) / ADC_RESOLUTION;
    float distance = voltageToDistance(voltage);
    lastMeasurements[i] = distance;
    delay(10);
  }
  filterIndex = 0;
  filterInitialized = true;
}

float readLaserDistance() {
  // Read analog value
  int rawValue = analogRead(LASER_ANALOG_PIN);
  
  // Convert to voltage
  float voltage = (rawValue * VOLTAGE_REF) / ADC_RESOLUTION;
  
  // Convert voltage to distance
  float distance = voltageToDistance(voltage);
  
  // Apply calibration offset
  distance += calibrationOffset;
  
  // Apply filtering if initialized
  if (filterInitialized) {
    distance = applyFilter(distance);
  }
  
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

float applyFilter(float newValue) {
  // Add new value to circular buffer
  lastMeasurements[filterIndex] = newValue;
  filterIndex = (filterIndex + 1) % filterSamples;
  
  // Calculate moving average
  float sum = 0;
  int validSamples = 0;
  
  for (int i = 0; i < filterSamples; i++) {
    if (lastMeasurements[i] > -999) { // Only average valid measurements
      sum += lastMeasurements[i];
      validSamples++;
    }
  }
  
  if (validSamples > 0) {
    return sum / validSamples;
  } else {
    return newValue; // Return unfiltered if no valid samples
  }
}

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
  Serial.print(",FILTER=");
  Serial.print(filterSamples);
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

void setFilterSamples(String command) {
  int spaceIndex = command.indexOf(' ');
  if (spaceIndex > 0) {
    int newFilterSamples = command.substring(spaceIndex + 1).toInt();
    
    if (newFilterSamples >= 1 && newFilterSamples <= 10) {
      filterSamples = newFilterSamples;
      
      // Reinitialize filter with new sample count
      initializeFilter();
      
      Serial.print("FILTER_OK:");
      Serial.println(filterSamples);
    } else {
      Serial.println("ERROR:FILTER_RANGE - Use 1-10 samples");
    }
  } else {
    Serial.println("ERROR:FILTER_FORMAT - Use: SET_FILTER [1-10]");
  }
}

void resetSystem() {
  // Reset calibration
  calibrationOffset = 0.0;
  
  // Reset filter
  filterSamples = 5;
  initializeFilter();
  
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
  Serial.print("Sensor Type: Keyence Laser (Always On)\n");
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
  Serial.print("Filter Samples: ");
  Serial.println(filterSamples);
  Serial.print("Calibration Offset: ");
  Serial.print(calibrationOffset);
  Serial.println(" mm");
  Serial.print("Measurement Rate: ");
  Serial.print(measurementRate);
  Serial.println(" Hz");
  Serial.println("===================");
}