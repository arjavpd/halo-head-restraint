#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33

// set the pins to shutdown
#define SHT_LOX1 27
#define SHT_LOX2 26
#define SHT_LOX3 25
#define SHT_LOX4 13

Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL6180X lox3 = Adafruit_VL6180X();
Adafruit_VL6180X lox4 = Adafruit_VL6180X();

// Setup mode for doing reads
typedef enum {RUN_MODE_DEFAULT} runmode_t;
runmode_t run_mode = RUN_MODE_DEFAULT;

// ADDED - MINIMUM SAFE DISTANCE (mm)
const int MIN_DIST = 50;
// ADDED - MAXIMUM SAFE DISTANCE (mm)
const int MAX_DIST = 100;
const int XmaxExtension = 190;
const int YmaxExtension = 100;
// ADDED - DISTANCE OF THE FIRST SENSOR (mm)
int ONE_DIST;
// ADDED - DISTANCE OF THE SECOND SENSOR (mm)
int TWO_DIST;
// ADDED - DISTANCE OF THE THIRD SENSOR (mm)
int THREE_DIST;
// ADDED - DISTANCE OF THE REAR SENSOR (mm)
int REAR_DIST = 1000;
// REAR DISTANCE TO DETECT CRASH
const int safetyDistance = 150;

int accel;
const int safetyAccel = 0;


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
    while (1);
  }
  lox1.setAddress(LOX1_ADDRESS);
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!lox2.begin()) {
    Serial.println(F("Failed to boot second VL6180X"));
    while (1);
  }
  lox2.setAddress(LOX2_ADDRESS);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX3
  if (!lox3.begin()) {
    Serial.println(F("Failed to boot third VL6180X"));
    while (1);
  }
  lox3.setAddress(LOX3_ADDRESS);

  // activating LOX4
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  //initing LOX4
  if (!lox4.begin()) {
    Serial.println(F("Failed to boot fourth VL6180X"));
    while (1);
  }
  lox4.setAddress(LOX4_ADDRESS);
}

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

  Serial.println("Starting...");
  setID();
}

void loop() {
  readSensor(lox4);
  if (REAR_DIST <= safetyDistance){
    readMPU6050();
    if (accel >= safetyAccel){
      headPosition();
      checkPosition();
    }
  }
  delay(10);
}

void headPosition() {
  readSensor(lox1);
  readSensor(lox2);
  readSensor(lox3);
  Serial.println();
}

void readMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel = a.acceleration.x;

  Serial.print("Acceleration X: ");
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

void readSensor(Adafruit_VL6180X &vl) {
  Serial.print(" Addr: ");
  Serial.print(vl.getAddress(), HEX);

  float lux = vl.readLux(VL6180X_ALS_GAIN_5);
  //Serial.print(" Lux: "); Serial.print(lux);
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  //assign the distances their value
  if (status == VL6180X_ERROR_NONE) {
    Serial.print(" Range: "); Serial.print(range);
    Serial.println();
    if (vl.getAddress() == 0x30) {
      ONE_DIST = range;
    }
    if (vl.getAddress() == 0x31) {
      TWO_DIST = range;
    }
    if (vl.getAddress() == 0x32) {
      THREE_DIST = range;
    }
    if (vl.getAddress() == 0x33) {
      REAR_DIST = range;
    }
  }

    //if top sensor detects nothing, give it a distinct value
  else if (vl.getAddress() == 0x30) {
    ONE_DIST = 1000;
    Serial.print(" NO HEAD DETECTED");
    Serial.println();
  }

  //if middle sensor detects nothing, give it a distinct value
  else if (vl.getAddress() == 0x31) {
    TWO_DIST = 1000;
    Serial.print(" NO HEAD DETECTED");
    Serial.println();
  }

  //if bottom sensor detects nothing, give it a distinct value
  else if (vl.getAddress() == 0x32) {
    THREE_DIST = 1000;
    Serial.print(" NO HEAD DETECTED");
    Serial.println();
  }

  //if rear sensor detects nothing, give it a distinct value
  else if (vl.getAddress() == 0x33) {
    REAR_DIST = 1000;
    Serial.print(" NO OBJECT DETECTED");
    Serial.println();
  }

}

void checkPosition() {
  
  //top sensor detects nothing
  if (ONE_DIST == 1000) {

    //middle sensor detects nothing
    if (TWO_DIST == 1000) {

      //bottom sensor detects nothing
      if (THREE_DIST == 1000) {
        Serial.print("MOVE FORWARD ");
        Serial.print(XmaxExtension);
        Serial.println();
      }

      //bottom sensor detects a head
      else {
        Serial.print("Y RETRACT ");
        Serial.print(YmaxExtension);
        Serial.println();
      }
    }

    //the height is good
    else {
      Serial.print("CORRECT HEIGHT");
      Serial.println();

      //the middle sensor is in range
      if (TWO_DIST > MIN_DIST && TWO_DIST < MAX_DIST) {
        Serial.print("CORRECT DISTANCE");

//        //the bottom sensor is in range
//        if (THREE_DIST > MIN_DIST && THREE_DIST < MAX_DIST) {
//          Serial.print("CORRECT DISTANCE & TILT");
//          Serial.println();
//        }
//
//        //the bottom sensor is too close
//        else if (THREE_DIST < MIN_DIST) {
//          Serial.print("TILT FORWARD");
//          Serial.println();
//        }
//
//        //the bottom sensor is too far
//        else {
//          Serial.print("TILT BACK");
//          Serial.println();
//        }
      }

      //the middle sensor is too close
      else if (TWO_DIST < MIN_DIST) {

//        //the bottom sensor is too close
//        if (THREE_DIST < MIN_DIST) {
          Serial.print("X RETRACT ");
          Serial.print(MIN_DIST - TWO_DIST);
          Serial.println();
//        }

//        //the bottom sensor is in range
//        else {
//          Serial.print("TILT BACK");
//          Serial.println();
//        }
      }

      //the middle sensor is too far
      else {

//        //the bottom sensor is too far
//        if (THREE_DIST > MAX_DIST) {
          Serial.print("X EXTEND ");
          Serial.print(TWO_DIST - MAX_DIST);
          Serial.println();
//        }

//        //the bottom sensor is in range
//        else {
//          Serial.print("TILT FORWARD");
//          Serial.println();
//        }
      }
    }
  }

  //top sensor detects a head
  else if (ONE_DIST != 1000) {
    Serial.print("Y EXTEND ");
    Serial.print(YmaxExtension/2);
    Serial.println();
  }
}
