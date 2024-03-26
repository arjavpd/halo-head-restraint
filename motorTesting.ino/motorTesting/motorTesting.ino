//The purpose of this program is to test the motors in an isolated set up. 
//Both the X and Y motors have the following settings: Microstep -> 4, Pulse/rev -> 800
//Keep in mind the switches on the TB6600 motor driver. Theey control the microstep and the current limit. 
//The X motor uses a Nema 17 while the Y motor uses a Nema 23.

//X MOTOR PINS 
const int enaX = 15;          // enables the motor
const int dirX = 18;          // determines the direction
const int pulX = 19;          // executes a step

//X MOTOR PINS 
const int enaY = 2;         
const int dirY = 4;          
const int pulY = 5;         

//CONSTANTS 
const int motorpulseInterval = 350;  // interval between pulse state changes
boolean motorpulseState = LOW;        // pulse state

void setup() {
  //SETTING UP X MOTOR
  pinMode(enaX, OUTPUT);
  pinMode(dirX, OUTPUT);
  pinMode(pulX, OUTPUT);
  digitalWrite(enaX, LOW);   // enable in inverted low
  digitalWrite(pulX, HIGH);  // falling edge

  //SETTING UP Y MOTOR 
  pinMode(enaY, OUTPUT);
  pinMode(dirY, OUTPUT);
  pinMode(pulY, OUTPUT);
  digitalWrite(enaY, LOW);   // enable in inverted low
  digitalWrite(pulY, HIGH);  // falling edge
}

void loop() {
  int distance = 50; //sample distance in mm
  int pulses = round((static_cast<float>(distance) / 60.0) * 1600); //formula to convert distance to pulses 
  
  //X motor actuation 
  digitalWrite(dirX, LOW); //set direction (LOW: CW, HIGH: CCW)

  for (int i = 0; i < pulses; i++) {
    motorpulseState = !motorpulseState;    // inverts the state of the variable
    digitalWrite(pulX, motorpulseState);  // assigns the new state to the port
    delayMicroseconds(motorpulseInterval);
  }

  delay(1000);  // 1 second delay between movements 

  digitalWrite(dirX, HIGH); 

  for (int i = 0; i < pulses; i++) {
    motorpulseState = !motorpulseState;            
    digitalWrite(pulX, motorpulseState);  
    delayMicroseconds(motorpulseInterval);
  }
  
  delay(1000); 

  //Y motor actuation 
  digitalWrite(dirY, LOW); 

  for (int i = 0; i < pulses; i++) {
    motorpulseState = !motorpulseState;           
    digitalWrite(pulY, motorpulseState); 
    delayMicroseconds(motorpulseInterval);
  }

  delay(1000);  

  digitalWrite(dirY, HIGH); 

  for (int i = 0; i < pulses; i++) {
    motorpulseState = !motorpulseState;           
    digitalWrite(pulY, motorpulseState);  
    delayMicroseconds(motorpulseInterval);
  }

  delay(1000); 
}
