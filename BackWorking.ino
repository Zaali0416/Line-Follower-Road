#include <SparkFun_TB6612.h>
#include <SoftwareSerial.h>


// Motor Pins
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 2
#define PWMB 8
#define STBY 5


// Define sensor patterns
#define RIGHT_TURN 0b11000
#define LEFT_TURN 0b00011
#define CENTER 0b00100
#define SOFT_RIGHT_1 0b11100
#define SOFT_RIGHT_2 0b01100
#define SOFT_LEFT_1 0b00111
#define SOFT_LEFT_2 0b00110
#define ALL_BLACK 0b00000


// Sensor Pins
const int sensorPins[] = { A5, A4, A3, A2, A1};
#define LDR1 A0
#define LDRR A6
#define TRIG_PIN 9
#define ECHO_PIN 10
#define Colour_Threshhold 400


// Motor Config
const int offsetA = 1;
const int offsetB = 1;
Motor motorLeft = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorRight = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


// Speed Settings
int baseSpeed = 150;
int turnSpeed = 100;


// Button Pins
#define CALIBRATE_PIN 11 // Reset button
#define START_PIN 12
#define LED_PIN 13


// Tracking Variables
int lastKnownDirection = 0; // -1=left, 0=center, 1=right
bool lineLost = false;
unsigned long lostTime = 0;
const int SEARCH_DELAY = 300;
#define RedThreshold 290


//Encoder
#define Wheel_lines 4
#define Wheel_circumference 14
#define White_Distance 2.1
float Distance_Travel = 0;
int Go = 0;
bool reversed = false; // global flag
int lines;


//Path  saving
#define MAX_PATH_LENGTH 10
char path[MAX_PATH_LENGTH];
const int pathLength = sizeof(path) / sizeof(path[0]);
int temp_P = 0;
char current_state;
char previousState = 'F';


//Status
bool CompletedExecution = false;


void setup() {
  Serial.begin(115200);

  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(CALIBRATE_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LDRR,INPUT);
}

void loop() {
  if (digitalRead(START_PIN) == LOW) {
    delay(500);
    Serial.println("Starting Robot");
    digitalWrite(LED_PIN, HIGH);
    lineFollow();
  }
}

void lineFollow() {

  while (true) {
    // Check reset button
    /*if (digitalRead(START_PIN) == LOW) {
    delay(500);
    Serial.println("Starting Robot");
    digitalWrite(LED_PIN, HIGH);
    lineFollow();
  }*/

    if (digitalRead(CALIBRATE_PIN) == LOW) { // Check if D12 is Low to Start the Robot
      delay(500);
      Serial.println("Reseting robot");
      digitalWrite(LED_PIN, HIGH);
      CompletedExecution = false;
      lineFollow();
    }

    // Read sensors
    int LDR_1 = analogRead(LDRR);
    
    /*int LDR_2 = analogRead(LDR2);
    int avg_ldr = (LDR_1 + LDR_2) / 2;
    float distance = getDistance();*/

    byte lineState = readLineSensors();

    if (!CompletedExecution) {

      float distance =getDistance();

      // Obstacle detection
      if (distance > 0 && distance < 15 && !reversed) {

        motorLeft.brake();
        motorRight.brake();
        delay(100);

        if(LDR_1>=RedThreshold)
        {

          //Acknowlege Red
          Serial.print("Red Obstacle is Detected!");
          motorLeft.brake();
          motorRight.brake();
          delay(100);

         //Path Display
         path[0] = previousState;

        for (int i = 0; i < 10; i++) {
          Serial.print("Path:");
          Serial.print(path[i]);
          Serial.print(", ");
          }

          //Sending Data to Esp32
          SendDataToEsp();

          //Reversing
          Reverse(lineState);
          reversed = true;
          Serial.println("Reverse done");
        }

         else {
          motorLeft.brake();
          motorRight.brake();
          delay(100);
        }

      } else {
        // Line following logic
        if (lineState != 0b00000 && lineState != 0b11111) {
          lineLost = false;
          updateLastKnownDirection(lineState);
          followLine(lineState);
          reversed = false;
        } else {
          handleLostLine();
        }
      }

      delay(20);
    }
  }
}

// --- Helper Functions --- //

void updateLastKnownDirection(byte state) {
  if (state & 0b11000) lastKnownDirection = 1; // Right
  else if (state & 0b00011) lastKnownDirection = -1; // Left
  else lastKnownDirection = 0; // Center
}

void handleLostLine() {
  if (!lineLost) {
    lineLost = true;
    lostTime = millis();
    Serial.println("Line Lost - Searching");
  }

  // Search pattern
  if (millis() - lostTime < SEARCH_DELAY) {
    motorLeft.drive(-baseSpeed / 2);
    motorRight.drive(-baseSpeed / 2);
  } else if (millis() - lostTime < SEARCH_DELAY * 2) {
    if (lastKnownDirection < 0) {
      motorLeft.drive(-turnSpeed);
      motorRight.drive(turnSpeed);
    } else {
      motorLeft.drive(turnSpeed);
      motorRight.drive(-turnSpeed);
    }
  } else {
    motorLeft.drive(baseSpeed / 2);
    motorRight.drive(baseSpeed / 2);
  }
}

void searchLine() {
  motorLeft.brake();
  motorRight.brake();
  delay(200);

  // Search in last known direction
  for (int i = 0; i < 3; i++) {
    if (lastKnownDirection < 0) {
      motorLeft.drive(-turnSpeed);
      motorRight.drive(turnSpeed);
    } else {
      motorLeft.drive(turnSpeed);
      motorRight.drive(-turnSpeed);
    }
    delay(200);

    byte sensorState = readLineSensors();
    if (sensorState != 0b00000 && sensorState != 0b11111) {
      lineLost = false;
      return;
    }
  }

  // Final attempt forward
  motorLeft.drive(baseSpeed / 2);
  motorRight.drive(baseSpeed / 2);
  delay(300);
}

byte readLineSensors() {
  byte state = 0;
  for (int i = 0; i < 5; i++) {
    state |= (!digitalRead(sensorPins[i]) << i);
  }
  return state;
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  return pulseIn(ECHO_PIN, HIGH) * 0.034 / 2;
}

void followLine(byte state) {

  const int softTurnRatio = 2; // For soft turns (1/2 speed)

  switch (state) {
  case RIGHT_TURN:
    motorLeft.drive(baseSpeed);
    motorRight.drive(-turnSpeed);
    Serial.println(" | RIGHT TURN");
    current_state = 'R';

    break;

  case LEFT_TURN:
    motorLeft.drive(-turnSpeed);
    motorRight.drive(baseSpeed);
    Serial.println(" | LEFT TURN");
    current_state = 'L';

    break;

  case CENTER:
    motorLeft.drive(baseSpeed);
    motorRight.drive(baseSpeed);
    Serial.println(" | FORWARD");
    current_state = 'F';

    break;

  case SOFT_RIGHT_1:
  case SOFT_RIGHT_2:
    motorLeft.drive(baseSpeed);
    motorRight.drive(turnSpeed / softTurnRatio);
    Serial.println(" | SOFT RIGHT");
    current_state = 'R';

    break;

  case SOFT_LEFT_1:
  case SOFT_LEFT_2:
    motorLeft.drive(turnSpeed / softTurnRatio);
    motorRight.drive(baseSpeed);
    Serial.println(" | SOFT LEFT");
    current_state = 'L';

    break;

  case ALL_BLACK:
    int chorono = millis();

    if (millis() - chorono == 6000) {
      if (state != ALL_BLACK) {
        followLine(state);
      } else {
        motorLeft.brake();
        motorRight.brake();
        CompletedExecution = true;
      }
    }

  default:
    motorLeft.drive(baseSpeed);
    motorRight.drive(baseSpeed);
    Serial.print(" | DEFAULT FORWARD - Unknown state: ");
    Serial.println(state, BIN); // Log unexpected state
  }

  if (current_state != previousState) {

    previousState = current_state;
    path[temp_P + 1] = previousState;
    temp_P++;

    //lines = Counting_Line(); // Only call once per loop()

  }
}

//============UART===========
void SendDataToEsp() {
  if (CompletedExecution) {
    Serial.print("PATH:");
    for (int i = 0; i < pathLength; i++) {
      Serial.print(path[i]);
      if (i < pathLength - 1) Serial.print(",");
    }
    Serial.println();
    delay(100); // Small delay after sending
  }
}

//=====================================ENCODE+REVERSE===============================

/*int Counting_Line() {
  static bool onWhite = false;

  int irValue = analogRead(IR_EN);

  if (irValue <= 80 && !onWhite) {
    Go++;
    onWhite = true;
    Serial.print("Front Temp = ");
    Serial.println(Go);
  }

  if (irValue > 80) {
    onWhite = false;
  }
  Serial.println(Go);
  return Go;
}*/

void Reverse(byte initialState) {
  digitalWrite(LED_PIN, HIGH);
  Serial.println("Reversing and rotating to find line...");

  unsigned long startTime = millis();
  const unsigned long maxRotateDuration = 5000; // 5 seconds max rotation

  // Begin rotating
  motorLeft.drive(-165); // Left backward
  motorRight.drive(165); // Right forward

  while (millis() - startTime < maxRotateDuration) {
    delay(100); // Allow some time to rotate before checking

    byte currentState = readLineSensors();

    if (isOnValidLine(currentState)) {
      Serial.print("Line re-detected: ");
      Serial.println(currentState, BIN);

      motorLeft.brake();
      motorRight.brake();
      return;
    }
  }

  // If line not found within time limit, continue rotating in place
  Serial.println("Line not found. Continuing rotation...");
  motorLeft.drive(-200); // Continue rotating
  motorRight.drive(200);
}

bool isOnValidLine(byte state) {
  return (
    state == CENTER ||
    state == LEFT_TURN ||
    state == RIGHT_TURN ||
    state == SOFT_RIGHT_1 ||
    state == SOFT_RIGHT_2 ||
    state == SOFT_LEFT_1 ||
    state == SOFT_LEFT_2
  );
}
float distance_travelled(int lines) {
  return (lines * Wheel_circumference);
}