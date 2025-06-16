#include <SD.h>
#include <SPI.h>

// Hardware Pins
#define VOLTAGE_PIN 25    // Voltage sensor
#define CURRENT_PIN 26    // Current sensor
#define RED_LED 2         // Status LED
#define GREEN_LED 15      // Status LED
#define CS_PIN 5          // SD card chip select

// UART Communication
#define NANO_RX 16        // GPIO16 (RX2)
#define NANO_TX 17        // GPIO17 (TX2)
HardwareSerial NanoSerial(2);  // Using UART2

// System Variables
bool obstacleDetected = false;
unsigned long lastObstacleTime = 0;

void setup() {
  // Initialize serial ports
  Serial.begin(115200);     // For debugging
  NanoSerial.begin(115200, SERIAL_8N1, NANO_RX, NANO_TX);  // Communication with Nano

  // Initialize LEDs
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, HIGH);  
  digitalWrite(RED_LED, HIGH);


  // Initialize SD card
 /* if (!SD.begin(CS_PIN)) {
    Serial.println("SD Card Mount Failed!");
    while(1);  // Halt if SD card fails
  }
  Serial.println("System Ready");*/
}

void loop() {

  Serial.print("Sytem has been started");
  //  Read power sensors
  float voltage = readVoltage();
  float current = readCurrent();

  updateLEDs();
  delay(10);  // Small delay to prevent watchdog triggers
}

float readVoltage() {
  // Adjust scaling factor according to your voltage divider
  return analogRead(VOLTAGE_PIN) * (12.6 / 4095.0);  // 12.6V max for 3.3V output
}
float readCurrent() {
  // ACS712 calibration (adjust for your specific sensor)
  return (analogRead(CURRENT_PIN) - 1850) * 0.0264;  // 1850 = zero point, 0.0264 = mV per A
}


void logSensorData(String nanoData, float voltage, float current) {
  File dataFile = SD.open("/datalog.txt", FILE_APPEND);
  
  if (dataFile) {
    // Timestamp
    dataFile.print(millis());
    dataFile.print("ms, ");
    
    // Power data
    dataFile.print("V:");
    dataFile.print(voltage, 2);
    dataFile.print("V, I:");
    dataFile.print(current, 2);
    dataFile.print("A, ");
    
    // Nano sensor data
    dataFile.println(nanoData);
    
    dataFile.close();
  } else {
    Serial.println("Error writing to SD card!");
  }
}

void updateLEDs() {
  static unsigned long lastToggle = 0;
  static bool ledState = false;
  
  // Toggle every 500ms (0.5 seconds)
  if (millis() - lastToggle >= 500) {
    lastToggle = millis();
    ledState = !ledState; // Toggle the state
    
    // Alternate the LEDs
    digitalWrite(RED_LED, ledState);
    digitalWrite(GREEN_LED, !ledState);
    
    // Optional: Send status to Nano (remove if not needed)
    if (ledState) {
      NanoSerial.println("LED_RED");
    } else {
      NanoSerial.println("LED_GREEN");
    }
    
    // Debug output
    Serial.print("LED State - RED: ");
    Serial.print(ledState ? "ON" : "OFF");
    Serial.print(" | GREEN: ");
    Serial.println(!ledState ? "ON" : "OFF");
  }
}