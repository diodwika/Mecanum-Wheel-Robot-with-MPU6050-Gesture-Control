#include <esp_now.h>
#include <WiFi.h>

#define triggerPin 5
#define echoPin 2

typedef struct struct_message {
  float pitch;
  float roll;
} struct_message;

struct_message myData;

const int motor1Pin1 = 26;
const int motor1Pin2 = 27;
const int motor1EnablePin = 14;

const int motor2Pin1 = 33;
const int motor2Pin2 = 25;
const int motor2EnablePin = 32;

const int motor3Pin1 = 15;
const int motor3Pin2 = 13;
const int motor3EnablePin = 22;

const int motor4Pin1 = 19;
const int motor4Pin2 = 18;
const int motor4EnablePin = 23;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(struct_message)) {
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.println("Data received:");
    Serial.print("Pitch: ");
    Serial.println(myData.pitch);
    Serial.print("Roll: ");
    Serial.println(myData.roll);
    Serial.println();

    // Set motor direction based on received data
    directionMotor(myData.pitch, myData.roll);

    float speed = abs(myData.pitch) + abs(myData.roll);

    analogWrite(motor1EnablePin, speed);
    analogWrite(motor2EnablePin, speed);
    analogWrite(motor3EnablePin, speed);
    analogWrite(motor4EnablePin, speed);

  } else {
    Serial.println("Received data size doesn't match expected size");
  }
}

long measureFrontDistance() {
  long duration, distance;
  
  // Kirim pulsa ultrasonik
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Baca durasi pantulan ultrasonik
  duration = pulseIn(echoPin, HIGH);

  // Hitung jarak berdasarkan waktu tempuh
  distance = (duration * 0.0343) / 2;

  delay(90);

  return distance;
}

void directionMotor(float pitch, float roll) {
  long frontDistance = measureFrontDistance();

  if (pitch > 20 && frontDistance > 10) {
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
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);

    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);

    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, LOW);

    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, LOW);
  }
  
  //float motorSpeed = myData.pitch + myData.roll;
  //float motorSpeedScaled = constrain(abs(motorSpeed), 0, 255);
}


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

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

  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  long duration, distance;
  
  // Kirim pulsa ultrasonik
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Baca durasi pantulan ultrasonik
  duration = pulseIn(echoPin, HIGH);

  // Hitung jarak berdasarkan waktu tempuh
  distance = (duration * 0.0343) / 2;

  delay(90);
}
