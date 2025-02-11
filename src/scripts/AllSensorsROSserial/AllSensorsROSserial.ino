
/*
   Author    : Tony Willett
   Date      : Nawfed o Chwefror 2025
   Description:
   Rosserial Node.
   For Diff Drive Robot.
   Subscribes to effort message to run the two Motors.
   Publishes wheel encoder messages.
   Subscribes to front and back Bumper switches.
   Subscribes to Four front sonar.
*/

#include <ros.h>
#include <NewPing.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>
#include <stdlib.h>

#define bpr_lf_pin 13
#define bpr_mf_pin 14
#define bpr_rf_pin 15
#define bpr_lb_pin 27
#define bpr_mb_pin 33
#define bpr_rb_pin 32
#define SONAR_NUM 4          //The number of sensors. 
#define MAX_DISTANCE 50

bool reading[6];
bool last_reading[6];
long last_debounce_time[6];
long debounce_delay = 30;
bool published[] = {true, true, true, true, true, true};
int switchPins[] = {bpr_lf_pin, bpr_mf_pin, bpr_rf_pin, bpr_lb_pin, bpr_mb_pin, bpr_rb_pin};

std_msgs::UInt64 enc_lf_msg;
std_msgs::UInt64 enc_lb_msg;
std_msgs::UInt64 enc_rf_msg;
std_msgs::UInt64 enc_rb_msg;
std_msgs::Bool bpr_lf_msg;
std_msgs::Bool bpr_mf_msg;
std_msgs::Bool bpr_rf_msg;
std_msgs::Bool bpr_lb_msg;
std_msgs::Bool bpr_mb_msg;
std_msgs::Bool bpr_rb_msg;
sensor_msgs::Range snr_1_msg;
sensor_msgs::Range snr_2_msg;
sensor_msgs::Range snr_3_msg;
sensor_msgs::Range snr_4_msg;

long lpwm, rpwm;
int motorPWM[2];
int forPins[2] = { 16, 18 };  // 16 right forward, 18 left forward
int bacPins[2] = { 17, 19 };  // 17 right backward, 19 right backward
int l_enc_int = 21;  // left encoder interrupt pin
int l_enc_dig = 22;  // left encoder digital pin
int r_enc_int = 25;  // right encoder interrupt pin
int r_enc_dig = 26;  // right encoder digital pin

unsigned long previousMillis;
int interval;
volatile unsigned long leftFor, leftBac;
volatile unsigned long rightFor, rightBac;
bool Lenc, Renc;

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(36, 36, MAX_DISTANCE),
  NewPing(39, 39, MAX_DISTANCE),
  NewPing(34, 34, MAX_DISTANCE),
  NewPing(35, 35, MAX_DISTANCE)
};
int snrReadings[4]; // list to store readings

//char rbuffer[10];
ros::NodeHandle nh;

void l_motorCB(const std_msgs::Float64& msg) {
  // Assign value to motor array
  motorPWM[0] = int(msg.data);
}

void r_motorCB(const std_msgs::Float64& msg) {
  // Assign value to motor array
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

void sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26;
  range_name.min_range = 0.0;
  range_name.max_range = 2.0;
}

// Wheel Encoder messages
ros::Publisher enc_lf_pub("/enc_lf", &enc_lf_msg);
ros::Publisher enc_lb_pub("/enc_lb", &enc_lb_msg);
ros::Publisher enc_rf_pub("/enc_rf", &enc_rf_msg);
ros::Publisher enc_rb_pub("/enc_rb", &enc_rb_msg);
// Wheel Motor messages
ros::Subscriber<std_msgs::Float64> l_motor("/left_wheel/control_effort", &l_motorCB);
ros::Subscriber<std_msgs::Float64> r_motor("/right_wheel/control_effort", &r_motorCB);
ros::Subscriber<std_msgs::Float64> l_setpoint("/left_wheel/setpoint", &l_setpointCB);
ros::Subscriber<std_msgs::Float64> r_setpoint("/right_wheel/setpoint", &r_setpointCB);
// Bumper messages
ros::Publisher bpr_lf_pub("/bpr_lf", &bpr_lf_msg);
ros::Publisher bpr_mf_pub("/bpr_mf", &bpr_mf_msg);
ros::Publisher bpr_rf_pub("/bpr_rf", &bpr_rf_msg);
ros::Publisher bpr_lb_pub("/bpr_lb", &bpr_lb_msg);
ros::Publisher bpr_mb_pub("/bpr_mb", &bpr_mb_msg);
ros::Publisher bpr_rb_pub("/bpr_rb", &bpr_rb_msg);
// Sonar messages
ros::Publisher snr_1_pub("/snr_1", &snr_1_msg);
ros::Publisher snr_2_pub("/snr_2", &snr_2_msg);
ros::Publisher snr_3_pub("/snr_3", &snr_3_msg);
ros::Publisher snr_4_pub("/snr_4", &snr_4_msg);

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
  nh.advertise(bpr_lf_pub);
  nh.advertise(bpr_mf_pub);
  nh.advertise(bpr_rf_pub);
  nh.advertise(bpr_lb_pub);
  nh.advertise(bpr_mb_pub);
  nh.advertise(bpr_rb_pub);
  nh.advertise(snr_1_pub);
  nh.advertise(snr_2_pub);
  nh.advertise(snr_3_pub);
  nh.advertise(snr_4_pub);

  sensor_msg_init(snr_1_msg, "/snr_1");
  sensor_msg_init(snr_2_msg, "/snr_2");
  sensor_msg_init(snr_3_msg, "/snr_3");
  sensor_msg_init(snr_4_msg, "/snr_4");

  //initialize pins
  attachInterrupt(l_enc_int, countL, RISING);
  attachInterrupt(r_enc_int, countR, RISING);
  pinMode(l_enc_dig, INPUT);
  pinMode(r_enc_dig, INPUT);
  pinMode(bpr_lf_pin, INPUT_PULLUP);
  pinMode(bpr_mf_pin, INPUT_PULLUP);
  pinMode(bpr_rf_pin, INPUT_PULLUP);
  pinMode(bpr_lb_pin, INPUT_PULLUP);
  pinMode(bpr_mb_pin, INPUT_PULLUP);
  pinMode(bpr_rb_pin, INPUT_PULLUP);

  for (int i = 0; i < 6; i++) {
    last_reading[i] = ! digitalRead(switchPins[i]);
  }
  previousMillis = millis();
  interval = 20;

  nh.loginfo("Serial Node started...");
}

void loop() {
  if (millis() > previousMillis + interval) {

    previousMillis = millis();

    for (int i = 0; i < 6; i++) {
      reading[i] = ! digitalRead(switchPins[i]);
    }
    for (int i = 0; i < 6; i++) {
      if (last_reading[i] != reading[i]) {
        last_debounce_time[i] = millis();
        published[i] = false;
      }
    }
    //if the button value has not changed for the debounce delay, we know its stable
    for (int i = 0; i < 6; i++) {
      if ( !published[i] && (millis() - last_debounce_time[i]) > debounce_delay) {
        if (i == 0) {
          bpr_lf_msg.data = reading[i];
          bpr_lf_pub.publish(&bpr_lf_msg);
          published[i] = true;
        }
        if (i == 1) {
          bpr_mf_msg.data = reading[i];
          bpr_mf_pub.publish(&bpr_mf_msg);
          published[i] = true;
        }
        if (i == 2) {
          bpr_rf_msg.data = reading[i];
          bpr_rf_pub.publish(&bpr_rf_msg);
          published[i] = true;
        }
        if (i == 3) {
          bpr_lb_msg.data = reading[i];
          bpr_lb_pub.publish(&bpr_lb_msg);
          published[i] = true;
        }
        if (i == 4) {
          bpr_mb_msg.data = reading[i];
          bpr_mb_pub.publish(&bpr_mb_msg);
          published[i] = true;
        }
        if (i == 5) {
          bpr_rb_msg.data = reading[i];
          bpr_rb_pub.publish(&bpr_rb_msg);
          published[i] = true;
        }
      }
    }
    for (int i = 0; i < 6; i++) {
      last_reading[i] = reading[i];
    }
    for (int i = 0; i < SONAR_NUM; i++) {
      snrReadings[i] = sonar[i].ping_cm();
    }

    enc_lf_msg.data = leftFor;
    enc_lb_msg.data = leftBac;
    enc_rf_msg.data = rightFor;
    enc_rb_msg.data = rightBac;
    enc_lf_pub.publish(&enc_lf_msg);
    enc_lb_pub.publish(&enc_lb_msg);
    enc_rf_pub.publish(&enc_rf_msg);
    enc_rb_pub.publish(&enc_rb_msg);
    snr_1_msg.range = snrReadings[0];
    snr_2_msg.range = snrReadings[1];
    snr_3_msg.range = snrReadings[2];
    snr_4_msg.range = snrReadings[3];
    snr_1_msg.header.stamp = nh.now();
    snr_1_pub.publish(&snr_1_msg);
    snr_2_msg.header.stamp = nh.now();
    snr_2_pub.publish(&snr_2_msg);
    snr_3_msg.header.stamp = nh.now();
    snr_3_pub.publish(&snr_3_msg);
    snr_4_msg.header.stamp = nh.now();
    snr_4_pub.publish(&snr_4_msg);

    runMotors();

    nh.spinOnce();
  }
}

void runMotors() {
  for (int i = 0; i < 2; i++) {
    // Ensure value stays within -255 and 255
    motorPWM[i] = max(min(motorPWM[i], 255.0), -255.0);
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
