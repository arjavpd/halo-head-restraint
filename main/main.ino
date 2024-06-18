#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

//LIDAR I2C PINS
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33

//LIDAR SHUTDOWN PINS
// set the pins to shutdown
#define SHT_LOX1 27
#define SHT_LOX2 26
#define SHT_LOX3 25
#define SHT_LOX4 13

//X MOTOR PINS
#define X_MOTOR_ENA 15  // enables the motor
#define X_MOTOR_DIR 18  // determines the direction
#define X_MOTOR_PUL 19  // executes a step

//Y MOTOR PINS
#define Y_MOTOR_ENA 2  // enables the motor
#define Y_MOTOR_DIR 4  // determines the direction
#define Y_MOTOR_PUL 5  // executes a step

//<------------CONSTANTS------------>
// MIN/MAX SAFE DISTANCE (mm)
const int MIN_SAFE_DIST = 50;
const int MAX_SAFE_DIST = 100;

// MAX Extension DISTANCE (mm)
const int X_MAX_EXTENSION = 100;
const int Y_MAX_EXTENSION = 60;

// MIN ACCELERATION VALUE TO DETECT CRASH
const int SAFETY_ACCEL = 0;

// MIN SAFETY VALUE FOR REAR DISTANCE TO DETECT CRASH
const int REAR_SAFETY_DISTANCE = 150;

//<------------GLOBAL VARIABLES------------>
// DISTANCE OF THE FIRST SENSOR (mm)
int topSensorDist;
// DISTANCE OF THE SECOND SENSOR (mm)
int middleSensorDist;
// DISTANCE OF THE THIRD SENSOR (mm)
int bottomSensorDist;

// DISTANCE OF THE REAR SENSOR (mm) - Default set to 1000 to avoid error
int rearObjDist = 1000;

// ACCELERATION (m/s^2)
int accel;

const int motorPulseInterval = 350;  // interval between pulse state changes

//LIDAR OBJECTS
Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL6180X lox3 = Adafruit_VL6180X();
Adafruit_VL6180X lox4 = Adafruit_VL6180X();

//<--------------------------------------------------->

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(1);
  }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);

  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  Serial.println("All in reset mode...(pins are low)");

  //X MOTOR SET UP
  pinMode(X_MOTOR_ENA, OUTPUT);
  pinMode(X_MOTOR_DIR, OUTPUT);
  pinMode(X_MOTOR_PUL, OUTPUT);
  digitalWrite(X_MOTOR_ENA, LOW);   // enable in inverted low
  digitalWrite(X_MOTOR_PUL, HIGH);  // falling edge

  //Y MOTOR SET UP
  pinMode(Y_MOTOR_ENA, OUTPUT);
  pinMode(Y_MOTOR_DIR, OUTPUT);
  pinMode(Y_MOTOR_PUL, OUTPUT);
  digitalWrite(Y_MOTOR_ENA, LOW);   // enable in inverted low
  digitalWrite(Y_MOTOR_PUL, HIGH);  // falling edge

  Serial.println("Starting...");
  setID();
}

void loop() {
  Serial.println("<----------START SENSOR READING---------->");
  Serial.println("<--Checking for Crash 😬-->");
  readLidarSensor(lox4);
  if (rearObjDist <= REAR_SAFETY_DISTANCE) {
    readMPU6050();
    if (accel >= SAFETY_ACCEL) {
      Serial.println("<--CRASH DETECTED!! 😱-->");
      Serial.println();
      senseHeadPosition();
      processHeadPosition();
    }
  }
  delay(1000);  //adjust this delay to change the speed of the whole system loop
}

void senseHeadPosition() {
  Serial.println("<--Sensing Head Position 👀-->");
  readLidarSensor(lox1);
  readLidarSensor(lox2);
  readLidarSensor(lox3);
  Serial.println();
}

void readMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel = a.acceleration.x;

  Serial.print("Accel X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  // Serial.print("Rotation X: ");
  // Serial.print(g.gyro.x);
  // Serial.print(", Y: ");
  // Serial.print(g.gyro.y);
  // Serial.print(", Z: ");
  // Serial.print(g.gyro.z);
  // Serial.println(" rad/s");

  // Serial.print("Temperature: ");
  // Serial.print(temp.temperature);
  // Serial.println(" degC");

  Serial.println("");
}

void readLidarSensor(Adafruit_VL6180X &vl) {
  // Serial.print(" Addr: ");
  // Serial.print(vl.getAddress(), HEX);

  float lux = vl.readLux(VL6180X_ALS_GAIN_5);
  //Serial.print(" Lux: "); Serial.print(lux);
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  //assign the distances their value
  if (status == VL6180X_ERROR_NONE) {
    if (vl.getAddress() == 0x30) {
      Serial.print("Top Sensor: ");
      Serial.print(range);
      Serial.println(" mm");
      topSensorDist = range;
    }
    if (vl.getAddress() == 0x31) {
      Serial.print("Middle Sensor: ");
      Serial.print(range);
      Serial.println(" mm");
      middleSensorDist = range;
    }
    if (vl.getAddress() == 0x32) {
      Serial.print("Bottom Sensor: ");
      Serial.print(range);
      Serial.println(" mm");
      bottomSensorDist = range;
    }
    if (vl.getAddress() == 0x33) {
      Serial.print("Rear Sensor: ");
      Serial.print(range);
      Serial.println(" mm");
      rearObjDist = range;
    }
  }

  //if top sensor detects nothing, give it a distinct value
  else if (vl.getAddress() == 0x30) {
    topSensorDist = 1000;
    Serial.print("Top Sensor: NO HEAD DETECTED");
    Serial.println();
  }

  //if middle sensor detects nothing, give it a distinct value
  else if (vl.getAddress() == 0x31) {
    middleSensorDist = 1000;
    Serial.print("Middle Sensor: NO HEAD DETECTED");
    Serial.println();
  }

  //if bottom sensor detects nothing, give it a distinct value
  else if (vl.getAddress() == 0x32) {
    bottomSensorDist = 1000;
    Serial.print("Bottom Sensor: NO HEAD DETECTED");
    Serial.println();
  }

  //if rear sensor detects nothing, give it a distinct value
  else if (vl.getAddress() == 0x33) {
    rearObjDist = 1000;
    Serial.print("Rear Sensor: NO OBJECT DETECTED");
    Serial.println();
  }
}

void processHeadPosition() {
  Serial.println("<--Process Head Position 🤔-->");
  //top sensor detects nothing
  if (topSensorDist == 1000) {

    //middle sensor detects nothing
    if (middleSensorDist == 1000) {

      //bottom sensor detects nothing
      if (bottomSensorDist == 1000) {
        Serial.print("X EXTEND: ");
        Serial.print(X_MAX_EXTENSION);
        Serial.println(" mm");
        Serial.println();
        actuateX(true, X_MAX_EXTENSION);
      }

      //bottom sensor detects a head
      else {
        Serial.print("Y RETRACT ");
        Serial.print(Y_MAX_EXTENSION);
        Serial.println(" mm");
        Serial.println();
        actuateY(false, Y_MAX_EXTENSION);
      }
    }

    //the height is good
    else {
      Serial.print("CORRECT HEIGHT");
      Serial.println();

      //the middle sensor is in range
      if (middleSensorDist > MIN_SAFE_DIST && middleSensorDist < MAX_SAFE_DIST) {
        Serial.print("CORRECT DISTANCE");
        Serial.println();
      }

      //the middle sensor is too close
      else if (middleSensorDist < MIN_SAFE_DIST) {
        Serial.print("X RETRACT ");
        Serial.print(MIN_SAFE_DIST - middleSensorDist);
        Serial.println(" mm");
        Serial.println();
        actuateX(false, MIN_SAFE_DIST - middleSensorDist);
      }

      //the middle sensor is too far
      else {
        Serial.print("X EXTEND ");
        Serial.print(middleSensorDist - MAX_SAFE_DIST);
        Serial.println(" mm");
        Serial.println();
        actuateX(true, middleSensorDist - MAX_SAFE_DIST);
      }
    }
  }

  //top sensor detects a head
  else if (topSensorDist != 1000) {
    Serial.print("Y EXTEND ");
    Serial.print(Y_MAX_EXTENSION / 2);
    Serial.println(" mm");
    Serial.println();
    actuateY(true, Y_MAX_EXTENSION / 2);
  }
}


void setID() {
  // initializing MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("");
  delay(100);

  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  delay(10);

  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);

  // initing LOX1
  if (!lox1.begin()) {
    Serial.println(F("Failed to boot first VL6180X"));
    while (1)
      ;
  }
  lox1.setAddress(LOX1_ADDRESS);
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!lox2.begin()) {
    Serial.println(F("Failed to boot second VL6180X"));
    while (1)
      ;
  }
  lox2.setAddress(LOX2_ADDRESS);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX3
  if (!lox3.begin()) {
    Serial.println(F("Failed to boot third VL6180X"));
    while (1)
      ;
  }
  lox3.setAddress(LOX3_ADDRESS);

  // activating LOX4
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  //initing LOX4
  if (!lox4.begin()) {
    Serial.println(F("Failed to boot fourth VL6180X"));
    while (1)
      ;
  }
  lox4.setAddress(LOX4_ADDRESS);
}

//MOTOR Settings
//Microstep: 4
//Pulse/rev: 800
void actuateX(bool extend, int distance) {
  boolean motorPulseState = LOW;                                    // pulse state
  int pulses = round((static_cast<float>(distance) / 60.0) * 800);  //distance (mm) * 60mm/1rev * 800 pulses/rev = pulses
  // Set the direction based on the extend parameter
  digitalWrite(X_MOTOR_DIR, extend ? LOW : HIGH);  // LOW for extending (CW), HIGH for retracting (CCW)

  for (int i = 0; i < pulses; i++) {
    motorPulseState = !motorPulseState;          // inverts the state of the variable
    digitalWrite(X_MOTOR_PUL, motorPulseState);  // assigns the new state to the port
    delayMicroseconds(motorPulseInterval);
  }
}

//MOTOR Settings
//Microstep: 4
//Pulse/rev: 800
void actuateY(bool extend, int distance) {
  boolean motorPulseState = LOW;                                    // pulse state
  int pulses = round((static_cast<float>(distance) / 60.0) * 800);  //distance (mm) * 60mm/1rev * 800 pulses/rev = pulses
  // Set the direction based on the extend parameter
  digitalWrite(Y_MOTOR_DIR, extend ? LOW : HIGH);  // LOW for extending (CW), HIGH for retracting (CCW)

  for (int i = 0; i < pulses; i++) {
    motorPulseState = !motorPulseState;          // inverts the state of the variable
    digitalWrite(Y_MOTOR_PUL, motorPulseState);  // assigns the new state to the port
    delayMicroseconds(motorPulseInterval);
  }
}
