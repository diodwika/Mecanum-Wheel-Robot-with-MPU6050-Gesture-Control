#include <esp_now.h>
#include <WiFi.h>

// Define a data structure for receiving pitch and roll
typedef struct struct_message {
  float pitch;
  float roll;
} struct_message;

struct_message myData;

// Constants for PID control
float Kp = 1.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.0;  // Derivative gain

float setpoint = 0.0;  // Target setpoint for pitch and roll

// Variables for PID control
float lastErrorPitch = 0.0;
float lastErrorRoll = 0.0;
float integralPitch = 0.0;
float integralRoll = 0.0;

// Motor control pins
const int motor1Pin1 = 26; // GPIO26 - Motor 1, Output 1
const int motor1Pin2 = 27; // GPIO27 - Motor 1, Output 2
const int motor1EnablePin = 14; // GPIO14 - Motor 1, Enable

const int motor2Pin1 = 33; // GPIO33 - Motor 2, Output 1
const int motor2Pin2 = 25; // GPIO32 - Motor 2, Output 2
const int motor2EnablePin = 32; // GPIO15 - Motor 2, Enable

const int motor3Pin1 = 15; // GPIO18 - Motor 3, Output 1
const int motor3Pin2 = 13; // GPIO19 - Motor 3, Output 2
const int motor3EnablePin = 22; // GPIO23 - Motor 3, Enable

const int motor4Pin1 = 18; // GPIO5 - Motor 4, Output 1
const int motor4Pin2 = 19; // GPIO17 - Motor 4, Output 2
const int motor4EnablePin = 23; // GPIO16 - Motor 4, Enable

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(struct_message)) {
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.println("Data received:");
    Serial.print("Pitch: ");
    Serial.println(myData.pitch);
    Serial.print("Roll: ");
    Serial.println(myData.roll);
    Serial.println();

    // Control the motors based on received data
    controlMotors(myData.pitch, myData.roll);
  } else {
    Serial.println("Received data size doesn't match expected size");
  }
}

void controlMotors(float pitch, float roll) {
  // Calculate the errors for pitch and roll
  float errorPitch = pitch - setpoint;
  float errorRoll = roll - setpoint;

  // Calculate the integral terms for pitch and roll
  integralPitch += errorPitch;
  integralRoll += errorRoll;

  // Calculate the derivative terms for pitch and roll
  float derivativePitch = errorPitch - lastErrorPitch;
  float derivativeRoll = errorRoll - lastErrorRoll;

  // Calculate the control output for pitch and roll
  float outputPitch = Kp * errorPitch + Ki * integralPitch + Kd * derivativePitch;
  float outputRoll = Kp * errorRoll + Ki * integralRoll + Kd * derivativeRoll;

  // Update the last errors for pitch and roll
  lastErrorPitch = errorPitch;
  lastErrorRoll = errorRoll;

  // Motor control based on pitch and roll values
  float motorSpeedPitch = outputPitch;  // Adjust this as needed
  float motorSpeedRoll = outputRoll;    // Adjust this as needed
  
  // Combine output
  float motorSpeed = motorSpeedPitch + motorSpeedRoll;

  // Determine motor direction and enable motors
  if (pitch > 20) {
    // Motors move forward
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    
    digitalWrite(motor3Pin1, HIGH);
    digitalWrite(motor3Pin2, LOW);
    
    digitalWrite(motor4Pin1, HIGH);
    digitalWrite(motor4Pin2, LOW);
  } 

  else if (pitch < -20) {
    // Motors move backward
    motorSpeed = -motorSpeed;  // Invert speed for backward motion

    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, HIGH);
    
    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, HIGH);
  } 
  
  else if (roll > 20) {
    // Motors move right
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, HIGH);
    
    digitalWrite(motor4Pin1, HIGH);
    digitalWrite(motor4Pin2, LOW);
  } 

  else if (roll < -20) {
    // Motors move left
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);

    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    
    digitalWrite(motor3Pin1, HIGH);
    digitalWrite(motor3Pin2, LOW);

    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, HIGH);
  } 

  else if (pitch > 20 && roll > 20) {
    // Motors move tilt forward riht
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);

    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    
    digitalWrite(motor3Pin1, HIGH);
    digitalWrite(motor3Pin2, LOW);

    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, LOW);
  } 

  else if (pitch > 20 && roll < -20) { 
    // Motors move tilt forward left
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);

    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, LOW);

    digitalWrite(motor4Pin1, HIGH);
    digitalWrite(motor4Pin2, LOW);
  } 

  else if (pitch < -20 && roll > 20) {
    // Motors move tilt backward right
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);

    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, HIGH);

    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, LOW);
  } 

  else if (pitch < -20 && roll < -20) {
    // Motors move tilt backward left
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);

    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, LOW);

    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, HIGH);
  }

  else {
    // Motors stop
    motorSpeed = 0.0;

    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);

    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);

    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, LOW);

    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, LOW);
  }

  // Apply the motor speed to control the motors (e.g., using PWM)
  // You may need to add PWM control for each motor if you are using PWM for speed control
  analogWrite(motor1EnablePin, motorSpeed);
  analogWrite(motor2EnablePin, motorSpeed);
  analogWrite(motor3EnablePin, motorSpeed);
  analogWrite(motor4EnablePin, motorSpeed);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);

  // Initialize motor control pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor1EnablePin, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor2EnablePin, OUTPUT);

  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);
  pinMode(motor3EnablePin, OUTPUT);

  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);
  pinMode(motor4EnablePin, OUTPUT);
}

void loop() {
  //Car movement
  if (myData.pitch > 20){
    Serial.print("Forward");
  }
  else if (myData.pitch < -20){
    Serial.print("Backward");
  }
  else if (myData.roll > 20){
    Serial.print("Right");
  }
  else if (myData.roll < -20){
    Serial.print("Left");
  }
  else if (myData.pitch > 20 && myData.roll > 20){
    Serial.print("Tilt Forward Right");
  }
  else if (myData.pitch > 20 && myData.roll < -20){
    Serial.print("Tilt Forward Left");
  }
  else if (myData.pitch < -20 && myData.roll > 20){
    Serial.print("Tilt Backward Right");
  }
  else if (myData.pitch > 20 && myData.roll < -20){
    Serial.print("Tilt Backward Left");
  }
  else{
    Serial.print("Stop");
  }
}
