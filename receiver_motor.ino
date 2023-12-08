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
float lastError = 0.0;
float integral = 0.0;

// Motor control pins
const int motorPin1 = 26; // GPIO26
const int motorPin2 = 27; // GPIO27
const int enablePin = 14; // Pin for motor driver enable

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(struct_message)) {
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.println("Data received:");
    Serial.print("Pitch: ");
    Serial.println(myData.pitch);
    Serial.print("Roll: ");
    Serial.println(myData.roll);
    Serial.println();

    // Control the motor based on received data
    controlMotor(myData.pitch, myData.roll);
  } else {
    Serial.println("Received data size doesn't match expected size");
  }
}

void controlMotor(float pitch, float roll) {
  // Calculate the error
  float error = pitch - setpoint;

  // Calculate the integral term
  integral += error;

  // Calculate the derivative term
  float derivative = error - lastError;

  // Calculate the control output
  float output = Kp * error + Ki * integral + Kd * derivative;

  // Update the last error
  lastError = error;

  // Motor control based on pitch and roll values
  float motorSpeed = output;  // Adjust this as needed

  // Determine motor direction and enable motor driver
  if (pitch > 20) {
    // Motor moves forward
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else if (pitch < -20) {
    // Motor moves backward
    motorSpeed = -motorSpeed;  // Invert speed for backward motion
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else {
    // Motor stops
    motorSpeed = 0.0;
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }

  // Apply the motor speed to control the motor (e.g., using PWM)
  analogWrite(enablePin, motorSpeed);  // Assuming you are using PWM for motor speed control
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
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
}

void loop() {
  // You can add your own code here if needed, but most of the data processing
  // happens in the OnDataRecv callback.
}
