// these constants won't change. They represent the pin numbers of the sensors' input and output:
const int trigPinTop = 7;
const int echoPinTop = 8;
const int trigPinBottom = 9;   
const int echoPinBottom = 10;  


// Left motor
int enA = 3;
int in1 = 5;
int in2 = 4;
// Right motor
int enB = 11;
int in3 = 6;
int in4 = 13;

//TODO: adjust the values as needed
//TODO: adjust the values as needed 
//Add comment! 
//the smallest distance  from the headrest the head should be at (in centimeters)
const int MINIMUM_SAFE_DISTANCE = 6;
//the largest head width the sensor will read (in centimeters)
const int MAX_HEAD_WIDTH = 15;

void setup() {
  // initialize serial communication:
  // trigPin sends out the ultrasonic pulse
  // echo pin listens to the echo
  Serial.begin(9600);
  pinMode(trigPinTop, OUTPUT);
  pinMode(echoPinTop, INPUT);
  pinMode(trigPinBottom, OUTPUT);
  pinMode(echoPinBottom, INPUT);
}

void loop() {
  // establish variables for the duration of the ping, and the distance result in centimeters:
  long durationTop, durationBottom, cmTop, cmBottom;

  // Reading from the top sensor
  cmTop = getDistance(trigPinTop, echoPinTop);

  // Reading from the bottom sensor
  cmBottom = getDistance(trigPinBottom, echoPinBottom);

  // Uncomment to see sensor readings
  // Serial.print("Top Sensor: ");
  // Serial.print(cmTop);
  // Serial.print("cm, Bottom Sensor: ");
  // Serial.print(cmBottom);
  // Serial.print("cm");
  // Serial.println();
  // Serial.println();

  checkDistance(cmTop , cmBottom);

  Serial.println();
  delay(2000);
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object, we take half of the distance traveled.
  return microseconds / 29 / 2;
}

long getDistance(int trigPin, int echoPin) {
  long duration, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read from the input of the sensor
  duration = pulseIn(echoPin, HIGH);

  // Convert the time into a distance
  cm = microsecondsToCentimeters(duration);

  return cm;
}

//4 Cases
//Case 1: Head too far
//Case 2: Head tilted forward, 
//Case 2.1: Head Too Low
//Case 3: Ideal Case  
void checkDistance(long cmTop, long cmBottom){
  if (cmTop > MINIMUM_SAFE_DISTANCE && cmBottom > MINIMUM_SAFE_DISTANCE) {
    Serial.print("head too far");
    moveCloser();
  } else if (cmTop > MINIMUM_SAFE_DISTANCE && cmBottom <= MINIMUM_SAFE_DISTANCE) {
    if (cmTop > MAX_HEAD_WIDTH) {
      Serial.print("head too low");
      moveDown();
    } else {
      Serial.print("head tilted too far forward");
      tiltForward();
    }
  } else {
    Serial.print("head in ideal position");
  }
}

//TODO: build out actuator functions 
void moveCloser(){
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
analogWrite(enA, 200);
delay(100);
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
analogWrite(enB, 200);

Serial.println();
Serial.print("moving closer...");
}

void moveDown() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 400);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 400);

  Serial.println();
  Serial.print("moving down...");
}

void tiltForward() {
  digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
analogWrite(enA, 400);

digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
analogWrite(enB, 400);

Serial.println();
Serial.print("tilting forward...");
}

