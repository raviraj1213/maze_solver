
//  BLACK LINE MAZE SOLVER WITH N20 ENCODER MOTORS
//  ============================================
//  For black line on white surface using L298N driver
//  and 5 IR sensor array for line detection
//*/
//
//// Pin Definitions for L298N Motor Driver
const int LEFT_MOTOR_IN1 = 7;
const int LEFT_MOTOR_IN2 = 8;
const int LEFT_MOTOR_EN = 9;
const int RIGHT_MOTOR_IN1 = 10;
const int RIGHT_MOTOR_IN2 = 12;
const int RIGHT_MOTOR_EN = 11;

// Encoder Pins
const int LEFT_ENCODER_C1 = 2;
const int LEFT_ENCODER_C2 = 4;
const int RIGHT_ENCODER_C1 = 3;
const int RIGHT_ENCODER_C2 = 5;

// Line Sensor Pins
const int SENSOR_LEFT = A0;
const int SENSOR_EDGE_LEFT = A1;
const int SENSOR_CENTER = A2;
const int SENSOR_EDGE_RIGHT = A3;
const int SENSOR_RIGHT = A4;

// Start Button
const int START_BUTTON = 13;

// Encoder Variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Motor Control Parameters
const int BASE_SPEED = 80;
const int MAX_SPEED = 160;
const int MIN_SPEED = 30;
const float COUNTS_PER_REV = 2970.0;
const float WHEEL_DIAMETER = 43.0;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
const float ROBOT_WIDTH = 250.0;

// PID Control Parameters
float KP = 0.5;  // Increased for black line
float KD = 0.2;
float KI = 0;

// Line Following Threshold for Black Line
const int THRESHOLD = 660;  // Adjust based on your sensor readings

// Junction Detection Flags
bool isLeftPath = false;
bool isRightPath = false;
bool isStraightPath = false;
bool needsUturn = false;
bool isEndNode = false;
int pathCount = 0;
bool mazeCompleted = false;
String pathTaken = "";

// PID Variables
int lastError = 0;
int integral = 0;

void setup() {
  Serial.begin(9600);

  // Configure Pins
  pinMode(START_BUTTON, INPUT_PULLUP);
  
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);

  // Encoder Pins
  pinMode(LEFT_ENCODER_C1, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_C2, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_C1, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_C2, INPUT_PULLUP);

  // Attach Interrupts for Encoders
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_C1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_C1), rightEncoderISR, CHANGE);
}

void loop() {
  // Debug sensor readings
  printSensorValues();
  
  // Wait for Start Button Press
  while (digitalRead(START_BUTTON)) {
    
    delay(100);
  }
  delay(1000);

  // Phase 1: Explore Maze
  while (!mazeCompleted) {
    printSensorValues();
    followLine();
    checkJunction();
    stopRobot();
    delay(50);
    makeDecision();
  }

  // Optimize Path
  optimizePath();

  // Wait for Second Button Press
  while (digitalRead(START_BUTTON)) {
    Serial.println("Press start button for optimized run");
    delay(100);
  }
  delay(1000);

  // Phase 2: Follow Optimized Path
  followOptimizedPath();
}

void printSensorValues() {
  Serial.print("Left: ");
  Serial.print(analogRead(SENSOR_LEFT));
  Serial.print(" Edge_Left: ");
  Serial.print(analogRead(SENSOR_EDGE_LEFT));
  Serial.print(" Center: ");
  Serial.print(analogRead(SENSOR_CENTER));
  Serial.print(" Edge_Right: ");
  Serial.print(analogRead(SENSOR_EDGE_RIGHT));
  Serial.print(" Right: ");
  Serial.println(analogRead(SENSOR_RIGHT));
  delay(100);
}

void leftEncoderISR() {
  if (digitalRead(LEFT_ENCODER_C1) == digitalRead(LEFT_ENCODER_C2)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_C1) == digitalRead(RIGHT_ENCODER_C2)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

void followLine() {
  while (!isAtJunction()) {
    int error = calculateError();
    int correction = computePID(error);

    int leftSpeed = BASE_SPEED - correction;
    int rightSpeed = BASE_SPEED + correction;

    leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

    setMotorSpeeds(leftSpeed, rightSpeed);
  }
}

int calculateError() {
  // For black line, positive error when robot drifts right
  return analogRead(SENSOR_EDGE_RIGHT) - analogRead(SENSOR_EDGE_LEFT);
}

int computePID(int error) {
  integral += error;
  int derivative = error - lastError;
  lastError = error;

  return (KP * error) + (KI * integral) + (KD * derivative);
}

bool isAtJunction() {
  // For black line, detect when sensors see black (low values)
  return (analogRead(SENSOR_LEFT) < THRESHOLD || analogRead(SENSOR_RIGHT) < THRESHOLD);
}


void checkJunction() {
  resetJunctionFlags();
  
  // Check if all sensors see white (high values)
  if (analogRead(SENSOR_LEFT) > THRESHOLD && 
      analogRead(SENSOR_EDGE_LEFT) > THRESHOLD &&
      analogRead(SENSOR_CENTER) > THRESHOLD &&
      analogRead(SENSOR_EDGE_RIGHT) > THRESHOLD &&
      analogRead(SENSOR_RIGHT) > THRESHOLD) {
    needsUturn = true;
    return;  // Exit function since we need to U-turn
  }

  // Rest of the original junction checking code
  if (analogRead(SENSOR_LEFT) < THRESHOLD) isLeftPath = true;
  if (analogRead(SENSOR_RIGHT) < THRESHOLD) isRightPath = true;
  
  moveDistance(50);
  
  if (analogRead(SENSOR_CENTER) < THRESHOLD) isStraightPath = true;
}


//void checkJunction() {
//  resetJunctionFlags();
//
//  if (analogRead(SENSOR_LEFT) < THRESHOLD) isLeftPath = true;
//  if (analogRead(SENSOR_RIGHT) < THRESHOLD) isRightPath = true;
//
//  moveDistance(50);  // Move forward to check straight path
//
//  if (analogRead(SENSOR_CENTER) < THRESHOLD) isStraightPath = true;
//
//  if (isEndNodeDetected()) {
//    isEndNode = true;
//  }
//
//  if (!isLeftPath && !isRightPath && !isStraightPath) {
//    needsUturn = true;
//  }
//
//  pathCount = isLeftPath + isRightPath + isStraightPath;
//}

void resetJunctionFlags() {
  isLeftPath = false;
  isRightPath = false;
  isStraightPath = false;
  needsUturn = false;
  isEndNode = false;
  pathCount = 0;
}

bool isEndNodeDetected() {
  return (analogRead(SENSOR_LEFT) < THRESHOLD &&
          analogRead(SENSOR_EDGE_LEFT) < THRESHOLD &&
          analogRead(SENSOR_CENTER) < THRESHOLD &&
          analogRead(SENSOR_EDGE_RIGHT) < THRESHOLD &&
          analogRead(SENSOR_RIGHT) < THRESHOLD);
}

void makeDecision() {
  if (isEndNode) {
    pathTaken += 'E';
    mazeCompleted = true;
    stopRobot();
    return;
  }

  if (isLeftPath) {
    pathTaken += 'L';
    turnByAngle(45);
  } else if (isStraightPath) {
    pathTaken += 'S';
  } else if (isRightPath) {
    pathTaken += 'R';
    turnByAngle(-45);
  } else if(needsUturn){
    pathTaken += 'U';
    turnByAngle(45);
    turnByAngle(45);
   
  }
}

void optimizePath() {
  for (int i = 0; i < 4; i++) {
    pathTaken.replace("LUL", "S");
    pathTaken.replace("LUS", "R");
    pathTaken.replace("RUL", "U");
    pathTaken.replace("SUL", "R");
  }
}

void followOptimizedPath() {
  int endIndex = pathTaken.indexOf('E');

  for (int i = 0; i <= endIndex; i++) {
    followLine();

    switch (pathTaken.charAt(i)) {
      case 'L':
        turnByAngle(45);
        break;
      case 'R':
        turnByAngle(-45);
        break;
      case 'U':
        turnByAngle(90);
        break;
      case 'E':
        stopRobot();
        while (1);  // Stop at end
        break;
    }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left Motor
  digitalWrite(LEFT_MOTOR_IN1, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(LEFT_MOTOR_IN2, leftSpeed >= 0 ? LOW : HIGH);
  analogWrite(LEFT_MOTOR_EN, abs(leftSpeed));

  // Right Motor
  digitalWrite(RIGHT_MOTOR_IN1, rightSpeed >= 0 ? LOW : HIGH);  // Reversed for correct direction
  digitalWrite(RIGHT_MOTOR_IN2, rightSpeed >= 0 ? HIGH : LOW);  // Reversed for correct direction
  analogWrite(RIGHT_MOTOR_EN, abs(rightSpeed));
}

void moveDistance(float distance_mm) {
  long targetCounts = (distance_mm / WHEEL_CIRCUMFERENCE) * COUNTS_PER_REV;
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  
  while (abs(leftEncoderCount) < targetCounts && abs(rightEncoderCount) < targetCounts) {
    setMotorSpeeds(BASE_SPEED, BASE_SPEED);
  }

  stopRobot();
}

void turnByAngle(float angle) {
  float arcLength = (angle / 360.0) * (PI * ROBOT_WIDTH);
  long targetCounts = (arcLength / WHEEL_CIRCUMFERENCE) * COUNTS_PER_REV;
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  if (angle > 0) {
    while (abs(leftEncoderCount) < targetCounts) {
      setMotorSpeeds(-BASE_SPEED, BASE_SPEED);
    }
  } else {
    while (abs(rightEncoderCount) < targetCounts) {
      setMotorSpeeds(BASE_SPEED, -BASE_SPEED);
    }
  }

  stopRobot();
}

void stopRobot() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_EN, 0);
  analogWrite(RIGHT_MOTOR_EN, 0);
}
