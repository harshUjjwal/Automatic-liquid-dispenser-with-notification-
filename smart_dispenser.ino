/**
 * Smart Liquid Dispenser System
 * 
 * An intelligent liquid dispensing system with environmental monitoring,
 * IoT capabilities, and safety features.
 * 
 * Features:
 * - Touchless dispensing using IR sensor
 * - Real-time environmental monitoring (DHT11)
 * - Liquid level tracking
 * - IoT notifications (Serial/ESP8266)
 * - LCD status display
 * - Safety features and motor protection
 * 
 * @author Your Name
 * @version 1.0.0
 * @license MIT
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// Pin Definitions
const int IR_SENSOR_PIN = 2;        // IR sensor input
const int MOTOR_PIN1 = 3;           // L293D Input 1
const int MOTOR_PIN2 = 4;           // L293D Input 2
const int MOTOR_ENABLE_PIN = 5;     // L293D Enable Pin
const int DHT_PIN = 6;              // DHT11 sensor
const int LEVEL_SENSOR_PIN = A0;    // Liquid level sensor
const int STATUS_LED = 13;          // Status LED

// System Constants
const unsigned long DISPENSE_TIME = 2000;     // Dispensing duration (ms)
const unsigned long COOLDOWN_TIME = 1000;     // Cooldown between dispenses (ms)
const unsigned long LCD_REFRESH = 2000;       // LCD refresh interval (ms)
const unsigned long SENSOR_READ_INTERVAL = 500; // Sensor reading interval (ms)
const unsigned long DEBOUNCE_DELAY = 50;      // IR sensor debounce time (ms)
const int LOW_LEVEL_THRESHOLD = 20;           // Low liquid warning threshold (%)
const int CRITICAL_LEVEL = 10;               // Critical level threshold (%)
const float DISPENSE_DECREMENT = 2.5;        // Level decrease per dispense (%)

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 16, 2);  // (0x27 is typical address, adjust if needed)

// DHT Configuration
DHT dht(DHT_PIN, DHT11);

// System State Structure
struct SystemState {
  float liquidLevel = 100.0;        // Current liquid level (%)
  float temperature = 0.0;          // Current temperature (°C)
  float humidity = 0.0;            // Current humidity (%)
  bool isDispensing = false;       // Dispensing state
  bool inCooldown = false;         // Cooldown state
  bool lowLevelAlerted = false;    // Low level notification state
  bool criticalLevelAlerted = false; // Critical level notification state
  unsigned long lastDispenseTime = 0;    // Last dispense timestamp
  unsigned long lastSensorRead = 0;      // Last sensor reading timestamp
  unsigned long lastLCDUpdate = 0;       // Last LCD update timestamp
  unsigned long lastDebounceTime = 0;    // Last IR sensor trigger time
  int lastIRState = HIGH;               // Last IR sensor state
  int stableIRState = HIGH;             // Debounced IR sensor state
} state;

/**
 * Initial setup of the system
 * Configures pins, initializes sensors and display
 */
void setup() {
  // Initialize serial communication
  Serial.begin(115200);  // Higher baud rate for IoT simulation
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  
  // Initialize DHT sensor
  dht.begin();
  
  // Configure pins
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(LEVEL_SENSOR_PIN, INPUT);
  
  // Initial system state
  stopMotor();
  updateDisplay();
  sendNotification("System initialized and ready");
}

/**
 * Main program loop
 * Handles sensor reading, motor control, and system updates
 */
void loop() {
  unsigned long currentTime = millis();
  
  // Handle IR sensor with debouncing
  handleIRSensor(currentTime);
  
  // Regular sensor readings and display updates
  handlePeriodicTasks(currentTime);
  
  // Handle dispensing and cooldown states
  handleDispenseState(currentTime);
  
  // Check liquid levels and send notifications
  checkLiquidLevels();
}

/**
 * Handles IR sensor reading and debouncing
 */
void handleIRSensor(unsigned long currentTime) {
  int reading = digitalRead(IR_SENSOR_PIN);
  if (reading != state.lastIRState) {
    state.lastDebounceTime = currentTime;
  }
  
  if ((currentTime - state.lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != state.stableIRState) {
      state.stableIRState = reading;
      if (state.stableIRState == LOW) {
        handleDispenseRequest();
      }
    }
  }
  state.lastIRState = reading;
}

/**
 * Handles periodic tasks like sensor reading and display updates
 */
void handlePeriodicTasks(unsigned long currentTime) {
  // Update sensors
  if (currentTime - state.lastSensorRead >= SENSOR_READ_INTERVAL) {
    readSensors();
    state.lastSensorRead = currentTime;
  }
  
  // Update display
  if (currentTime - state.lastLCDUpdate >= LCD_REFRESH) {
    updateDisplay();
    state.lastLCDUpdate = currentTime;
  }
}

/**
 * Handles the dispensing state and cooldown period
 */
void handleDispenseState(unsigned long currentTime) {
  if (state.isDispensing) {
    if (currentTime - state.lastDispenseTime >= DISPENSE_TIME) {
      finishDispensing();
    }
  }
  
  if (state.inCooldown && (currentTime - state.lastDispenseTime >= COOLDOWN_TIME)) {
    state.inCooldown = false;
  }
}

/**
 * Processes a dispense request
 */
void handleDispenseRequest() {
  if (!state.isDispensing && !state.inCooldown && state.liquidLevel > CRITICAL_LEVEL) {
    startDispensing();
  }
}

/**
 * Starts the dispensing process
 */
void startDispensing() {
  state.isDispensing = true;
  state.lastDispenseTime = millis();
  startMotor();
  digitalWrite(STATUS_LED, HIGH);
  sendNotification("Dispensing liquid");
}

/**
 * Completes the dispensing process
 */
void finishDispensing() {
  stopMotor();
  state.isDispensing = false;
  state.inCooldown = true;
  state.liquidLevel -= DISPENSE_DECREMENT;
  state.liquidLevel = max(0.0, state.liquidLevel);
  digitalWrite(STATUS_LED, LOW);
  sendNotification("Dispense complete");
}

/**
 * Reads values from all sensors
 */
void readSensors() {
  // Read DHT11 sensor
  float newTemp = dht.readTemperature();
  float newHumidity = dht.readHumidity();
  
  if (!isnan(newTemp) && !isnan(newHumidity)) {
    state.temperature = newTemp;
    state.humidity = newHumidity;
  }
  
  // Read liquid level sensor (or simulate)
  #ifdef USE_REAL_SENSOR
    int rawLevel = analogRead(LEVEL_SENSOR_PIN);
    state.liquidLevel = map(rawLevel, 0, 1023, 0, 100);
  #endif
}

/**
 * Checks liquid levels and sends notifications if needed
 */
void checkLiquidLevels() {
  if (state.liquidLevel <= CRITICAL_LEVEL && !state.criticalLevelAlerted) {
    sendNotification("CRITICAL: Liquid level below 10%");
    state.criticalLevelAlerted = true;
  } else if (state.liquidLevel <= LOW_LEVEL_THRESHOLD && !state.lowLevelAlerted) {
    sendNotification("WARNING: Low liquid level");
    state.lowLevelAlerted = true;
  } else if (state.liquidLevel > LOW_LEVEL_THRESHOLD) {
    state.lowLevelAlerted = false;
    state.criticalLevelAlerted = false;
  }
}

/**
 * Updates the LCD display with current status
 */
void updateDisplay() {
  lcd.clear();
  
  // First line: Status and level
  lcd.setCursor(0, 0);
  if (state.liquidLevel <= CRITICAL_LEVEL) {
    lcd.print("REFILL REQUIRED!");
  } else if (state.isDispensing) {
    lcd.print("Dispensing...");
  } else if (state.inCooldown) {
    lcd.print("Please Wait...");
  } else {
    lcd.print("Ready  L:");
    lcd.print((int)state.liquidLevel);
    lcd.print("%");
  }
  
  // Second line: Environmental data
  lcd.setCursor(0, 1);
  lcd.print(state.temperature, 1);
  lcd.print("C ");
  lcd.print(state.humidity, 0);
  lcd.print("% ");
}

/**
 * Starts the motor
 */
void startMotor() {
  analogWrite(MOTOR_ENABLE_PIN, 255);
  digitalWrite(MOTOR_PIN1, HIGH);
  digitalWrite(MOTOR_PIN2, LOW);
}

/**
 * Stops the motor
 */
void stopMotor() {
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, LOW);
  analogWrite(MOTOR_ENABLE_PIN, 0);
}

/**
 * Sends a notification through Serial (IoT simulation)
 */
void sendNotification(const char* message) {
  Serial.print("IoT Notification: ");
  Serial.println(message);
  
  // Here you would typically send data to ESP8266 or other IoT module
  // Example: esp8266.println(message);
} 