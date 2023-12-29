#include <HCSR04.h>
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;
ros::Publisher pub("from_arduino", &distance_cm);
std_msgs::Int32 distance_cm;

const int pingPin = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 13; // Echo Pin of Ultrasonic Sensor

void setup() {
  nh.getHardware()->setBaud(91600);
  nh.initNode();
  nh.advertise(pub);
}

void loop() {
    nh.spinOnce();
    long duration, cm;
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pingPin, LOW);
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
    cm = (float)((duration / 2) / 29.41);
    // cm = microsecondsToCentimeters(duration);
    // Serial.print(inches);
    // Serial.print("in, ");
    // Serial.print(cm);
    // Serial.print("cm");
    // Serial.println();
    // float cm = distanceSensor.measureDistanceCm();
    distance_cm.data = int(cm);
    pub.publish(&distance_cm);
    delay(500);
}