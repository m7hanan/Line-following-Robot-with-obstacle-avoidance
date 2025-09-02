// --- IR Sensor Pins ---
#define leftIR 2
#define middleIR 3
#define rightIR 4

// --- L298N Motor Control Pins ---
#define ENA 5
#define IN1 6
#define IN2 7
#define ENB 10
#define IN3 8
#define IN4 9

// --- Ultrasonic Sensor Pins ---
#define trigPin 11
#define echoPin 12

// --- Push Buttons & LEDs ---
#define button1 13    // Mode 1 select
#define button2 A0    // Mode 2 select
#define greenLED A1   // Indicator for Mode 1
#define blueLED A5    // Blue LED on ATmega pin 27 (Arduino A5)
#define redLED A2     // Indicator for Obstacle Detection

// --- Speed Values ---
int linearSpeed = 60;
int fastSpeed = 70;
int turnSpeed = 60;

// --- Obstacle Detection ---
bool obstacleDetected = false;
const int OBSTACLE_DISTANCE = 15; // 15 cm
const int OBSTACLE_CONFIRM_COUNT = 2; // Need 2 consecutive detections

// --- Non-blocking Turn ---
unsigned long turnStartTime = 0;
bool isTurning = false;
String turnDirection = "";

// --- Obstacle LED Blinking ---
unsigned long previousBlinkTime = 0;
const long blinkInterval = 500; // Blink interval in milliseconds
bool ledState = false;

// --- Sensor Timing ---
unsigned long lastIRReadTime = 0;
const int IR_READ_INTERVAL = 20; // Read IR sensors every 20ms
unsigned long lastUltraSonicTime = 0;
const int ULTRASONIC_READ_INTERVAL = 100; // Read ultrasonic every 100ms

// --- Debounce Variables ---
int obstacleDetectionCount = 0;

// --- Mode Variable ---
int mode = 0; // 0 = No mode selected yet

// ------------------- SETUP -------------------
void setup() {
  pinMode(leftIR, INPUT);
  pinMode(middleIR, INPUT);
  pinMode(rightIR, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW); // Ensure trigger is low initially

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Buttons & LEDs
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  // --- Set PWM frequency same for both motors (490 Hz) ---
  TCCR0B = TCCR0B & 0b11111000 | 0x03;  // Pin 5
  TCCR1B = TCCR1B & 0b11111000 | 0x03;  // Pin 9

  setMode(0); // Start with NO MODE
  stopCar();

  Serial.begin(9600);
  delay(1000); // Allow sensors to stabilize
}

// ------------------- LOOP -------------------
void loop() {
  // --- Mode Selection ---
  if (digitalRead(button1) == LOW) { 
    setMode(1);
    delay(300); // Debounce
  }
  if (digitalRead(button2) == LOW) {
    setMode(2);
    delay(300); // Debounce
  }

  // --- Obstacle LED Blinking ---
  if (obstacleDetected) {
    unsigned long currentTime = millis();
    if (currentTime - previousBlinkTime >= blinkInterval) {
      previousBlinkTime = currentTime;
      ledState = !ledState;
      digitalWrite(redLED, ledState);
    }
  } else {
    digitalWrite(redLED, LOW);
  }

  // --- Mode Behavior ---
  if (mode == 1) {
    lineFollow();
  } else if (mode == 2) {
    obstacleLineFollow();
  } else {
    // No mode selected yet
    stopCar();
  }
}

// ------------------- MODE FUNCTION -------------------
void setMode(int newMode) {
  mode = newMode;
  obstacleDetected = false; // Reset obstacle detection when changing modes
  isTurning = false; // Reset turning state
  obstacleDetectionCount = 0; // Reset detection counter
  
  if (mode == 1) {
    digitalWrite(greenLED, HIGH);
    digitalWrite(blueLED, LOW);
    digitalWrite(redLED, LOW);
    Serial.println("Mode 1 Selected: Line following only");
  } 
  else if (mode == 2) {
    digitalWrite(greenLED, LOW);
    digitalWrite(blueLED, HIGH);
    digitalWrite(redLED, LOW);
    Serial.println("Mode 2 Selected: Line following + Obstacle detection");
  } 
  else {
    digitalWrite(greenLED, LOW);
    digitalWrite(blueLED, LOW);
    digitalWrite(redLED, LOW);
    Serial.println("No mode selected. Waiting...");
  }
}

// ------------------- LINE FOLLOWING -------------------
void lineFollow() {
  unsigned long currentTime = millis();
  
  // Read IR sensors at controlled intervals
  if (currentTime - lastIRReadTime >= IR_READ_INTERVAL) {
    lastIRReadTime = currentTime;
    
    int leftValue = digitalRead(leftIR);
    int middleValue = digitalRead(middleIR);
    int rightValue = digitalRead(rightIR);

    if (leftValue == HIGH && middleValue == HIGH && rightValue == HIGH) {
      stopCar();
    }
    else if (leftValue == LOW && middleValue == LOW && rightValue == LOW) {
      moveForward(linearSpeed);
    }
    else if (leftValue == LOW && middleValue == HIGH && rightValue == LOW) {
      moveForward(fastSpeed);
    }
    else if (leftValue == HIGH && middleValue == LOW && rightValue == LOW) {
      startTurn("LEFT");
    }
    else if (leftValue == LOW && middleValue == LOW && rightValue == HIGH) {
      startTurn("RIGHT");
    }
    else {
      moveForward(fastSpeed);
    }
  }
}

// ------------------- LINE FOLLOW + OBSTACLE -------------------
void obstacleLineFollow() {
  unsigned long currentTime = millis();
  
  // Check ultrasonic at controlled intervals (less frequent than IR)
  if (currentTime - lastUltraSonicTime >= ULTRASONIC_READ_INTERVAL) {
    lastUltraSonicTime = currentTime;
    
    int distance = getDistance();
    
    if (distance <= OBSTACLE_DISTANCE && distance > 0) {
      obstacleDetectionCount++;
      
      // Only confirm obstacle after multiple detections (debouncing)
      if (obstacleDetectionCount >= OBSTACLE_CONFIRM_COUNT && !obstacleDetected) {
        Serial.println("Obstacle detected! Stopping...");
        obstacleDetected = true;
        stopCar();
        isTurning = false;
      }
    } else {
      obstacleDetectionCount = 0; // Reset counter
      
      if (obstacleDetected) {
        Serial.println("Obstacle cleared! Resuming line following...");
        obstacleDetected = false;
      }
    }
  }

  // If no obstacle, continue with line following
  if (!obstacleDetected) {
    if (isTurning) {
      continueTurn();
    } else {
      lineFollow();
    }
  }
}

// ------------------- IMPROVED ULTRASONIC READING -------------------
int getDistance() {
  // Ensure no interrupts during ultrasonic reading
  noInterrupts();
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read pulse with timeout to prevent hanging
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  
  interrupts(); // Re-enable interrupts
  
  if (duration == 0) {
    // Timeout occurred, return a safe value
    return OBSTACLE_DISTANCE + 10;
  }
  
  int distance = duration * 0.034 / 2;
  return distance;
}

// ------------------- TURNING -------------------
void startTurn(String direction) {
  turnDirection = direction;
  turnStartTime = millis();
  isTurning = true;

  if (direction == "LEFT") {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, turnSpeed);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, turnSpeed);
  } else if (direction == "RIGHT") {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, turnSpeed);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, turnSpeed);
  }
}

void continueTurn() {
  if (millis() - turnStartTime >= 300) {
    isTurning = false;

    int leftValue = digitalRead(leftIR);
    int middleValue = digitalRead(middleIR);
    int rightValue = digitalRead(rightIR);

    if (middleValue == HIGH || (leftValue == LOW && rightValue == LOW)) {
      moveForward(linearSpeed);
    }
    else if ((turnDirection == "LEFT" && leftValue == HIGH) ||
             (turnDirection == "RIGHT" && rightValue == HIGH)) {
      startTurn(turnDirection);
    }
  }
}

// ------------------- MOTOR CONTROL -------------------
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
