#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

//#include <HCSR04.h>

ros::NodeHandle nh;

int right_speed = 100;
int left_speed = 100;

//std_msgs::Int32 distance_cm;
//ros::Publisher pub("from_arduino", &distance_cm);

void messageCb(const std_msgs::Int32MultiArray& received_msg) {
  right_speed = received_msg.data[0];
  left_speed = received_msg.data[1];
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("to_arduino", messageCb);


// Define motor driver pins //L
// Motor A
const int motorA1 = 8;  // Connect to the IN1 pin on L298
const int motorA2 = 9;  // Connect to the IN2 pin on L298
const int motorAEnable = 6; // Connect to the ENA pin on L298
const byte encoder0PinA1 = 2; //A Pin -> interrupt pin 0
const byte encoder0PinA2 = 4; //B Pin -> digital pin 4
byte encoder0pinALast;
double durationA=0, rpmA;
boolean DirectionA;
double val_outputA = 0;
double SetpointA = 0;
double Kp_A = 3, Ki_A = 0.1, Kd_A = 0;
PID APID(&rpmA, &val_outputA, &SetpointA, Kp_A, Ki_A, Kd_A, DIRECT);

// Motor B    //R
const int motorB1 = 10;   // Connect to the IN3 pin on L298
const int motorB2 = 11;   // Connect to the IN4 pin on L298
const int motorBEnable = 5; // Connect to the ENB pin on L298
const byte encoder0PinB1 = 3; //A Pin -> interrupt pin 1
const byte encoder0PinB2 = 12;  //B Pin -> digital pin 12
byte encoder0pinBLast;
double durationB=0, rpmB;
boolean DirectionB;
double val_outputB = 0;
double SetpointB = 0;
double Kp_B = 3, Ki_B = 0.1, Kd_B = 0;
PID BPID(&rpmB, &val_outputB, &SetpointB, Kp_B, Ki_B, Kd_B, DIRECT);

// const int pingPin = 7; // Trigger Pin of Ultrasonic Sensor
// const int echoPin = 13; // Echo Pin of Ultrasonic Sensor

// Initialize sensor that uses digital pins 13 and 12.
//const byte triggerPin = 7;
//const byte echoPin = 13;
//UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

void motor_setup() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorAEnable, OUTPUT);

  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorBEnable, OUTPUT);
}

void EncoderInit(){
  DirectionA = true; //default -> forward
  pinMode(encoder0PinA2 ,INPUT);
  attachInterrupt(0, WheelSpeed_A, CHANGE);

  DirectionB = true; //default -> forward
  pinMode(encoder0PinB2 ,INPUT);
  attachInterrupt(1, WheelSpeed_B, CHANGE);
}

void WheelSpeed_A(){
  int Lstate = digitalRead(encoder0PinA1);
  if((encoder0pinALast == LOW) && Lstate == HIGH){
    int valA = digitalRead(encoder0PinA2);
    if(valA == LOW && DirectionA){
      DirectionA = false;
    }
    else if(valA == HIGH && !DirectionA){
      DirectionA = true;
    }
  }
  encoder0pinALast = Lstate;

  if(!DirectionA)  durationA++;
  else durationA--;
}

void WheelSpeed_B(){
  int Rstate = digitalRead(encoder0PinB1);
  if((encoder0pinBLast == LOW) && Rstate == HIGH){
    int valB = digitalRead(encoder0PinB2);
    if(valB == LOW && DirectionB){
      DirectionB = false;
      
    }
    else if(valB == HIGH && !DirectionB){
      DirectionB = true;
    }
  }
  encoder0pinBLast = Rstate;

  if(!DirectionB)  durationB++;
  else durationB--;
}

void controlMotor(int pwm, int enPin, int in1Pin, int in2Pin) {
    if (pwm >= 0) {
        analogWrite(enPin, pwm);
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    } else if (pwm < 0) {
        analogWrite(enPin, abs(pwm));
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    }
}

void motorA(int pwmA) {
    controlMotor(pwmA, motorAEnable, motorA1, motorA2);
}

void motorB(int pwmB) {
    controlMotor(pwmB, motorBEnable, motorB1, motorB2);
}


void setup() {
  //Serial.begin(91600);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(pub);
  motor_setup();
  APID.SetMode(AUTOMATIC);
  APID.SetSampleTime(100);
  APID.SetOutputLimits(0, 200);
  BPID.SetMode(AUTOMATIC);
  BPID.SetSampleTime(100);
  BPID.SetOutputLimits(0, 200);
  EncoderInit();
  motorA(left_speed);
  motorB(right_speed);
}


void loop() {
  nh.spinOnce();

  if(val_outputA * left_speed < 0 || val_outputB * right_speed < 0){
    motorA(left_speed/abs(left_speed));
    motorB(right_speed/abs(right_speed));
    delay(500);
    durationA = 0;
    durationB = 0;
    delay(100);
  }

  SetpointA = abs(left_speed);
  SetpointB = abs(right_speed);

  //Serial.print(SetpointA);
  //Serial.print(" ");
  //Serial.print(SetpointB);
  //Serial.print(" ");
  
  //motorA(left_speed);
  rpmA = abs(durationA)/2;       
  //Serial.print(rpmA);
  //Serial.print(" ");
  APID.Compute();
  durationA = 0;
  
  //motorB(right_speed);
  rpmB = abs(durationB)/2;
  //Serial.println(rpmB);
  
  BPID.Compute();
  durationB = 0;
  if(left_speed < 0) val_outputA = -val_outputA;
  if(right_speed < 0) val_outputB = -val_outputB;
  motorA(val_outputA);
  motorB(val_outputB);

  // long duration, cm;
  // pinMode(pingPin, OUTPUT);
  // digitalWrite(pingPin, LOW);
  // delayMicroseconds(2);
  // digitalWrite(pingPin, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(pingPin, LOW);
  // pinMode(echoPin, INPUT);
  // duration = pulseIn(echoPin, HIGH);
  // cm = (float)((duration / 2) / 29.41);
  // cm = microsecondsToCentimeters(duration);
  // Serial.print(inches);
  // Serial.print("in, ");
  // Serial.print(cm);
  // Serial.print("cm");
  // Serial.println();
  // float cm = distanceSensor.measureDistanceCm();
  // distance_cm.data = int(cm);
  // pub.publish(&distance_cm);
 
  delay(100);
}

// long microsecondsToCentimeters(long microseconds) {
//    return microseconds / 29 / 2;
// }
