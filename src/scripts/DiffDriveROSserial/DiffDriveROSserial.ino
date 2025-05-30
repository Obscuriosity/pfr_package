
/*
   Author    : Tony Willett
   Date      : Ail o Chwefror 2025
   Description:
   Rosserial Node.
   For Diff Drive Robot.
   Subscribes to effort message to run the two Motors.
   Publishes wheel encoder messages.
*/

#include <ros.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>


std_msgs::UInt64 enc_lf_msg;
std_msgs::UInt64 enc_lb_msg;
std_msgs::UInt64 enc_rf_msg;
std_msgs::UInt64 enc_rb_msg;

long lpwm, rpwm;
int motorPWM[2];
int forPins[2] = { 16, 18 };  // 16 right forward, 18 left forward
int bacPins[2] = { 17, 19 };  // 17 right backward, 19 right backward
int l_enc_int = 34;  // left encoder interrupt pin
int l_enc_dig = 35;  // left encoder digital pin
int r_enc_int = 36;  // right encoder interrupt pin
int r_enc_dig = 39;  // right encoder digital pin

unsigned long previousMillis;
int interval;
volatile unsigned long leftFor, leftBac;
volatile unsigned long rightFor, rightBac;
bool Lenc, Renc;

//char rbuffer[10];
ros::NodeHandle nh;

void l_motorCB(const std_msgs::Float64& msg) {
  // Assign msg value to motor array.
  motorPWM[0] = int(msg.data);
}

void r_motorCB(const std_msgs::Float64& msg) {
  // Assign msg value to motor array.
  motorPWM[1] = int(msg.data);
}

void l_setpointCB(const std_msgs::Float64& setpoint) {
  // monitor setpoint, if it is zero set pwm to zero
  if (setpoint.data == 0.0) {
    motorPWM[0] = 0;
  }
}

void r_setpointCB(const std_msgs::Float64& setpoint) {
  // monitor setpoint, if it is zero set pwm to zero
  if (setpoint.data == 0.0) {
    motorPWM[1] = 0;
  }
}

ros::Publisher enc_lf_pub("/enc_lf", &enc_lf_msg);
ros::Publisher enc_lb_pub("/enc_lb", &enc_lb_msg);
ros::Publisher enc_rf_pub("/enc_rf", &enc_rf_msg);
ros::Publisher enc_rb_pub("/enc_rb", &enc_rb_msg);
ros::Subscriber<std_msgs::Float64> l_motor("/left_wheel/control_effort", &l_motorCB);
ros::Subscriber<std_msgs::Float64> r_motor("/right_wheel/control_effort", &r_motorCB);
ros::Subscriber<std_msgs::Float64> l_setpoint("/left_wheel/setpoint", &l_setpointCB);
ros::Subscriber<std_msgs::Float64> r_setpoint("/right_wheel/setpoint", &r_setpointCB);

void setup() {
  nh.initNode();
  nh.advertise(enc_lf_pub);
  nh.advertise(enc_lb_pub);
  nh.advertise(enc_rf_pub);
  nh.advertise(enc_rb_pub);
  nh.subscribe(l_motor);
  nh.subscribe(r_motor);
  nh.subscribe(l_setpoint);
  nh.subscribe(r_setpoint);

  //initialize pins
  attachInterrupt(l_enc_int, countL, RISING);
  attachInterrupt(r_enc_int, countR, RISING);
  pinMode(l_enc_dig, INPUT);
  pinMode(r_enc_dig, INPUT);

  previousMillis = millis();
  interval = 20;

  nh.loginfo("Serial Node started...");
}

void loop() {
  if (millis() > previousMillis + interval) {

    previousMillis = millis();

    enc_lf_msg.data = leftFor;
    enc_lb_msg.data = leftBac;
    enc_rf_msg.data = rightFor;
    enc_rb_msg.data = rightBac;
    enc_lf_pub.publish(&enc_lf_msg);
    enc_lb_pub.publish(&enc_lb_msg);
    enc_rf_pub.publish(&enc_rf_msg);
    enc_rb_pub.publish(&enc_rb_msg);

    runMotors();

    nh.spinOnce();
  }
}

void runMotors() {
  for (int i = 0; i < 2; i++) {
    motorPWM[i] = max(min(motorPWM[i], 225), -225);
    if (motorPWM[i] > 0) {
      analogWrite(forPins[i], motorPWM[i]);
      analogWrite(bacPins[i], 0);
    } else if (motorPWM[i] < 0) {
      analogWrite(forPins[i], 0);
      analogWrite(bacPins[i], motorPWM[i] * -1);
    } else {
      analogWrite(forPins[i], 0);
      analogWrite(bacPins[i], 0);
    }
  }
}

void countL() {
  Lenc = digitalRead(l_enc_dig);
  if (Lenc == HIGH) {
    leftBac++;
  } else {
    leftFor++;
  }
}

void countR() {
  Renc = digitalRead(r_enc_dig);
  if (Renc == HIGH) {
    rightFor++;
  } else {
    rightBac++;
  }
}
