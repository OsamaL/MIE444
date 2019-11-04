#include <Arduino.h>
#include <string.h>
#include <L298N.h>
#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
// #include <std_msgs/String.h>
// #include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
// #include <ros/time.h>
// #include <stdlib.h>
#include <main.h>

//initializing all the variables
const int LOOPTIME = 100;     //Looptime in millisecond

const int LEFT_ENC_A = 18;               //A channel for encoder of left motor                    
const int LEFT_ENC_B = 19;               //B channel for encoder of left motor
const int RIGHT_ENC_A = 20;              //A channel for encoder of right motor         
const int RIGHT_ENC_B = 21;              //B channel for encoder of right motor 

const int ENCODER_TICKS_PER_REV = 32;
const int ENCODER_TICKS_TOLERANCE = 1; //32 ticks if counting rising and falling for one encoder channel

const double wheel_radius = 0.034;                   //Wheel wheel_, in m
const double wheelbase = 0.2;                        //Wheelbase, in m

const double ppr[2] = {5909.0/20.0, 10332.0/20.0};
const double raw_to_meters[2] = {2 * PI * wheel_radius / ppr[0],
                                 2 * PI * wheel_radius / ppr[1]};

// Define motors
// EN, IN1, IN2
L298N L_motor(5, 6, 7);
L298N R_motor(8, 9, 10);

// PID Parameters
double Kp[2] = {150, 2000};
double Ki[2] = {2000.00, 0.00};
double Kd[2] = {2.00, 0.00};
const double spdLimit[2] = {127, 127};

// Raw position from encodersdtostrf() 
double goal_vel[2] = {0, 0};   // m/s
volatile long raw_pos[2];      // in encoder ticks
double output_pwm[2] = {0, 0};
double actual_vel[2] = {0, 0}; // m/s

volatile long prev_raw_pos[2] = {0, 0}; // in encoder ticks
volatile long prev_pid_time = 0;

PID L_PID(&actual_vel[0], &output_pwm[0], &goal_vel[0], Kp[0], Ki[0], Kd[0], DIRECT);
PID R_PID(&actual_vel[1], &output_pwm[1], &goal_vel[1], Kp[1], Ki[1], Kd[1], DIRECT);

// Setup ROS
ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_vel("/cmd_vel", cmd_vel_cb);

void setup() {
	delay(100); // This fixes the PID NaN issues. it's spooky.
	L_motor.stop();
	R_motor.stop();
	setup_encoders();
	setup_PID();
	goal_vel[0] = 0.00;
	goal_vel[1] = 0.00;

	// Setup ROS
	nh.getHardware()->setBaud(115200);
	nh.initNode();
	nh.loginfo("Node initialized");
	nh.subscribe(cmd_vel);
}

void loop() {
	nh.spinOnce();
	// do some stuff here to assign goal_vel

	update_PID();
	update_motors();
	delay(10);
}

void update_PID() {
	// compute the velocities in m/s from raw_pos
	long cur_pid_time = millis();
	double delta_t = 0.001 * double(cur_pid_time - prev_pid_time); // in seconds

	actual_vel[0] = raw_to_meters[0] * double(raw_pos[0] - prev_raw_pos[0]) / delta_t;
	actual_vel[1] = raw_to_meters[1] * double(raw_pos[1] - prev_raw_pos[1]) / delta_t;

	prev_raw_pos[0] = raw_pos[0];
	prev_raw_pos[1] = raw_pos[1];
	prev_pid_time = cur_pid_time;

    L_PID.Compute();
    R_PID.Compute();
}

void setup_PID() {
    L_PID.SetMode(AUTOMATIC);
    L_PID.SetOutputLimits(-spdLimit[0], spdLimit[0]);
    R_PID.SetMode(AUTOMATIC);
    R_PID.SetOutputLimits(-spdLimit[1], spdLimit[1]);
}

void setup_encoders() {
	pinMode(LEFT_ENC_A, INPUT); 
	pinMode(LEFT_ENC_B, INPUT); 
	digitalWrite(LEFT_ENC_A, HIGH);                // turn on pullup resistor
	digitalWrite(LEFT_ENC_B, HIGH);
	attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), encoderLeftMotor, RISING);

	// Define the rotary encoder for right motor
	pinMode(RIGHT_ENC_A, INPUT); 
	pinMode(RIGHT_ENC_B, INPUT); 
	digitalWrite(RIGHT_ENC_A, HIGH);                // turn on pullup resistor
	digitalWrite(RIGHT_ENC_B, HIGH);
	attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), encoderRightMotor, RISING);
}

//Left motor encoder counter
void encoderLeftMotor() {
	if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) raw_pos[0]--;
	else raw_pos[0]++;
}

//Right motor encoder counter
void encoderRightMotor() {
	if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) raw_pos[1]--;
	else raw_pos[1]++;
}

void update_motors() {	
	L_motor.setSpeed(abs(output_pwm[0]));
	if(abs(goal_vel[0]) < 0.0005) {
		L_motor.stop();
	} else if ( output_pwm[0] < 0) {
		L_motor.forward();
	} else {
		L_motor.backward();
	}

	R_motor.setSpeed(abs(output_pwm[1]));
	if(abs(goal_vel[1]) < 0.0005) {
		R_motor.stop();
	} else if ( output_pwm[1] < 0) {
		R_motor.forward();
	} else {
		R_motor.backward();
	}



	// R_motor.setSpeed(abs(output_pwm[1]));
	// if ( output_pwm[1] < 0) R_motor.forward();
	// else R_motor.backward();
}

void cmd_vel_cb(const geometry_msgs::Twist& twist_input) {
	double goal_speed = twist_input.linear.x;
	double goal_angular = twist_input.angular.z;
	goal_vel[0] = (2*goal_speed - goal_angular*wheelbase)/(2);
	goal_vel[1] = (2*goal_speed + goal_angular*wheelbase)/(2);
	nh.loginfo(String("Got cmd_vel: " + String(goal_speed, 3) +
	           " ," + String(goal_angular, 3)).c_str());
	nh.loginfo(String("goal_vel: " + String(goal_vel[0], 3) +
	           " ," + String(goal_vel[1], 3)).c_str());
}
