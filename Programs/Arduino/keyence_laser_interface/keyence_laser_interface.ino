/*
 * Keyence Laser Sensor Interface for Die Scanner
 * 
 * This Arduino sketch interfaces with a Keyence laser displacement sensor
 * providing analog 0-5V output. The Arduino acts as an ADC and USB interface
 * for the industrial PC running the die scanner software.
 * 
 * Hardware Connections:
 * - Analog Pin A0: Laser sensor analog output (0-5V)
 * - Digital Pin 2: Relay control for sensor power
 * - USB: Communication with industrial PC
 * 
 * Communication Protocol:
 * - Commands from PC: "POWER_ON", "POWER_OFF", "READ"
 * - Responses to PC: "POWER_ON_OK", "POWER_OFF_OK", "HEIGHT:XXX.X"
 */

// Pin definitions
const int LASER_ANALOG_PIN = A0;    // Analog input from laser sensor
const int SENSOR_POWER_PIN = 2;     // Digital output to control sensor power relay

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
bool sensorPowered = false;

// Measurement filtering
const int FILTER_SAMPLES = 5;      // Number of samples for averaging
float lastMeasurements[FILTER_SAMPLES];
int filterIndex = 0;
bool filterInitialized = false;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.setTimeout(100);
  
  // Initialize pins
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  pinMode(LASER_ANALOG_PIN, INPUT);
  
  // Turn off sensor power initially
  digitalWrite(SENSOR_POWER_PIN, LOW);
  sensorPowered = false;
  
  // Initialize filter array
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    lastMeasurements[i] = 0;
  }
  
  // Set ADC reference to external 5V
  analogReference(DEFAULT);
  
  Serial.println("Keyence Laser Interface Ready");
  Serial.println("Commands: POWER_ON, POWER_OFF, READ");
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
  
  delay(10); // Small delay to prevent overwhelming the serial port
}

void processCommand(String command) {
  if (command == "POWER_ON") {
    powerOnSensor();
  }
  else if (command == "POWER_OFF") {
    powerOffSensor();
  }
  else if (command == "READ") {
    if (sensorPowered) {
      float distance = readLaserDistance();
      Serial.print("HEIGHT:");
      Serial.println(distance, 1);
    } else {
      Serial.println("ERROR:SENSOR_NOT_POWERED");
    }
  }
  else if (command == "STATUS") {
    reportStatus();
  }
  else {
    Serial.println("ERROR:UNKNOWN_COMMAND");
  }
}

void powerOnSensor() {
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  sensorPowered = true;
  
  // Wait for sensor to stabilize
  delay(500);
  
  // Initialize filter with current readings
  initializeFilter();
  
  Serial.println("POWER_ON_OK");
}

void powerOffSensor() {
  digitalWrite(SENSOR_POWER_PIN, LOW);
  sensorPowered = false;
  filterInitialized = false;
  
  Serial.println("POWER_OFF_OK");
}

void initializeFilter() {
  // Fill filter buffer with initial readings
  for (int i = 0; i < FILTER_SAMPLES; i++) {
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
  if (!sensorPowered) {
    return -999.0; // Error code for sensor not powered
  }
  
  // Read analog value
  int rawValue = analogRead(LASER_ANALOG_PIN);
  
  // Convert to voltage
  float voltage = (rawValue * VOLTAGE_REF) / ADC_RESOLUTION;
  
  // Convert voltage to distance
  float distance = voltageToDistance(voltage);
  
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
  filterIndex = (filterIndex + 1) % FILTER_SAMPLES;
  
  // Calculate moving average
  float sum = 0;
  int validSamples = 0;
  
  for (int i = 0; i < FILTER_SAMPLES; i++) {
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
  Serial.print("STATUS:");
  Serial.print("POWER=");
  Serial.print(sensorPowered ? "ON" : "OFF");
  
  if (sensorPowered) {
    int rawValue = analogRead(LASER_ANALOG_PIN);
    float voltage = (rawValue * VOLTAGE_REF) / ADC_RESOLUTION;
    float distance = readLaserDistance();
    
    Serial.print(",RAW=");
    Serial.print(rawValue);
    Serial.print(",VOLTAGE=");
    Serial.print(voltage, 2);
    Serial.print(",DISTANCE=");
    Serial.print(distance, 1);
  }
  
  Serial.println();
}