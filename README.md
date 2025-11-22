# tys
just tys

// --- 1. PIN DEFINITIONS ---

// Define pins for the L298N Motor Driver
// Left Motor (Motor A)
const int ENA = 9;   // Enable pin for Motor A (PWM pin for speed control)
const int IN1 = 10;  // Input 1 for Motor A
const int IN2 = 11;  // Input 2 for Motor A

// Right Motor (Motor B)
const int ENB = 5;   // Enable pin for Motor B (PWM pin for speed control)
const int IN3 = 6;   // Input 3 for Motor B
const int IN4 = 7;   // Input 4 for Motor B

// Define digital pins for the IR Sensors
// IMPORTANT: Assuming sensor outputs LOW when it detects the black line
const int leftSensor = 2;  
const int rightSensor = 3; 

// Define motor speed (0-255)
const int motorSpeed = 150; 

// -------------------------------------------------------------------

void setup() {
  // Set motor control pins as OUTPUT
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set sensor pins as INPUT
  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);

  // Set initial motor speed using PWM pins
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);
  Serial.println("Line Follower Robot Initialized");
}

// -------------------------------------------------------------------

void loop() {
  // Read sensor values
  int leftValue = digitalRead(leftSensor);
  int rightValue = digitalRead(rightSensor);

  // Optional: Print values for debugging
  Serial.print("Left Sensor (2): ");
  Serial.print(leftValue == LOW ? "LINE" : "WHITE"); // LOW = LINE, HIGH = WHITE
  Serial.print(" | Right Sensor (3): ");
  Serial.println(rightValue == LOW ? "LINE" : "WHITE");

  // LOGIC: (Assuming LOW = Black Line detected, HIGH = White Surface)

  // Case 1: Both sensors see the WHITE background (0, 0 in LOW/HIGH logic, both are on the line)
  // The robot is centered on the line (or both are off a very wide line) -> Move Forward
  if (leftValue == LOW && rightValue == LOW) {
    moveForward();
  }
  // Case 2: Left sensor on WHITE, Right sensor on LINE (HIGH, LOW)
  // The line has shifted to the RIGHT -> Turn Right to bring the left sensor back over the line
  else if (leftValue == HIGH && rightValue == LOW) {
    turnRight();
  }
  // Case 3: Left sensor on LINE, Right sensor on WHITE (LOW, HIGH)
  // The line has shifted to the LEFT -> Turn Left to bring the right sensor back over the line
  else if (leftValue == LOW && rightValue == HIGH) {
    turnLeft();
  }
  // Case 4: Both sensors see the BLACK line (HIGH, HIGH)
  // The robot has completely lost the line, reached an intersection, or a stop point -> Stop
  else {
    halt();
  }
}

// -------------------------------------------------------------------
// --- MOTOR CONTROL FUNCTIONS ---

void moveForward() {
 Left Motor Forward (IN1=HIGH, IN2=LOW)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
 Right Motor Forward (IN3=HIGH, IN4=LOW)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
   Right Motor Forward (drives the turn)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
    Left Motor Backward (optional: for sharper turn) or Stop (for pivot/gentler turn)
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW); // Stopping the left motor
}

void turnRight() {
   Left Motor Forward (drives the turn)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
   Right Motor Backward (optional: for sharper turn) or Stop (for pivot/gentler turn)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW); // Stopping the right motor
}

void halt() {
   Stop both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
