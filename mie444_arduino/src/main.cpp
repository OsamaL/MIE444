#include <Arduino.h>
#include <string.h>
#include <L298N.h>
#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <ros/time.h>
#include <main.h>
#include <Servo.h>

Servo grip_servo;

//initializing all the variables
const int LOOPTIME = 100;     //Looptime in millisecond

const int LEFT_ENC_A = 18;               //A channel for encoder of left motor                    
const int LEFT_ENC_B = 19;               //B channel for encoder of left motor
const int RIGHT_ENC_A = 20;              //A channel for encoder of right motor         
const int RIGHT_ENC_B = 21;              //B channel for encoder of right motor 

const int ENCODER_TICKS_PER_REV = 32;
const int ENCODER_TICKS_TOLERANCE = 1; //32 ticks if counting rising and falling for one encoder channel

const double wheel_radius = 0.034/2.0*0.742757*0.978125;                   //Wheel wheel_, in m
const double wheelbase = 0.2/1.375;         //Wheelbase, in m

const double ppr[2] = {23636.0/20.0*1.015, 41454.0/20.0};
const double raw_to_meters[2] = {(2.0 * PI * wheel_radius) / ppr[0],
                                 (2.0 * PI * wheel_radius) / ppr[1]};
const double meters_to_raw[2] = {1.0/raw_to_meters[0],
                                 1.0/raw_to_meters[1]};

// Define motors
// EN, IN1, IN2
L298N R_motor(5, 6, 7);
L298N L_motor(8, 9, 10);

// random globals
unsigned long last_oscope_print = 0;

// PID Parameters
double Kp[2] = {2500, 3700};
double Ki[2] = {1000.0, 800.00};
double Kd[2] = {250.00, 100.0};
const double spdLimit[2] = {200, 200};

// Raw position from encodersdtostrf() 
double goal_speed = 0;
double goal_angular = 0;
double goal_pos[2] = {0, 0};   // m
volatile long raw_pos[2];      // in encoder ticks
double output_pwm[2] = {0, 0};
double actual_pos[2] = {0, 0}; // m/s

volatile long prev_time = 0;

PID L_PID(&actual_pos[0], &output_pwm[0], &goal_pos[0], Kp[0], Ki[0], Kd[0], P_ON_E, DIRECT);
PID R_PID(&actual_pos[1], &output_pwm[1], &goal_pos[1], Kp[1], Ki[1], Kd[1], P_ON_E, DIRECT);

// Setup ROS
ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_vel("/cmd_vel", cmd_vel_cb);

ros::Subscriber<std_msgs::String> cmd_special("/cmd_special", cmd_special_cb);

// nav_msgs::Odometry odom_msg;
// ros::Publisher odom("odom", &odom_msg);
geometry_msgs::Pose pose_msg;
ros::Publisher pose_pub("/pose", &pose_msg);

char base_link_tf[] = "/base_link";
char odom_tf[] = "/odom";

void setup() {
	delay(100); // This fixes the PID NaN issues. it's spooky.
	grip_servo.attach(11);
	L_motor.stop();
	R_motor.stop();
	setup_encoders();
	setup_PID();
	goal_pos[0] = 0.00;
	goal_pos[1] = 0.00;

	// Setup ROS
	nh.getHardware()->setBaud(115200);
	nh.initNode();
	nh.loginfo("Node initialized");
	nh.subscribe(cmd_vel);
	nh.subscribe(cmd_special);
	nh.advertise(pose_pub);
}

void loop() {
	nh.spinOnce();
	update_odom();
	update_PID();
	update_motors();
}

void update_PID() {
	actual_pos[0] = raw_to_meters[0] * raw_pos[0];
	actual_pos[1] = raw_to_meters[1] * raw_pos[1];

	// integrate the velocity into a desired position
	long cur_time = micros();
	double delta_t = 0.000001 * double(cur_time - prev_time); // in seconds
	
	goal_pos[0] += delta_t * (2.0*goal_speed - goal_angular*wheelbase)/2.0;
	goal_pos[1] += delta_t * (2.0*goal_speed + goal_angular*wheelbase)/2.0;

	// Serial.print(" a_vel ");
	// dtostrf(actual_pos[0], 7, 4, str_temp);
	// Serial.print(str_temp);
	// Serial.print(" ");
	// dtostrf(actual_pos[1], 7, 4, str_temp);
	// Serial.print(str_temp);

	// Serial.print(" raw ");
	// dtostrf(raw_pos[0], 2, 0, str_temp);
	// Serial.print(str_temp);
	// Serial.print(" ");
	// dtostrf(raw_pos[1], 2, 0, str_temp);
	// Serial.print(str_temp);

    L_PID.Compute();
    R_PID.Compute();

	// Serial.print(" pwm ");
	// dtostrf(output_pwm[0], 5, 1, str_temp);
	// Serial.print(str_temp);
	// Serial.print(" ");
	// dtostrf(output_pwm[1], 5, 1, str_temp);
	// Serial.print(str_temp);

	// PRINT_oscilloscope(actual_pos[0], goal_pos[0], 1000, 0.0055, 0.1, 0.4);
	// PRINT_oscilloscope(actual_pos[1], goal_pos[1], 1000, 0.0055, 0.1, 0.4);
	
	// PRINT_oscilloscope(actual_pos[0]-goal_pos[0], 0, 1000, 0.0055, 0.1, 0.1);
	// PRINT_oscilloscope(actual_pos[1]-goal_pos[1], 0, 1000, 0.0055, 0.1, 0.1);
	prev_time = cur_time;
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
	attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), LEFT_ENC_A_handler, CHANGE);
	attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), LEFT_ENC_B_handler, CHANGE);

	// Define the rotary encoder for right motor
	pinMode(RIGHT_ENC_A, INPUT); 
	pinMode(RIGHT_ENC_B, INPUT); 
	digitalWrite(RIGHT_ENC_A, HIGH);                // turn on pullup resistor
	digitalWrite(RIGHT_ENC_B, HIGH);
	attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), RIGHT_ENC_A_handler, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), RIGHT_ENC_B_handler, CHANGE);
}

// Encoder handlers

void LEFT_ENC_A_handler() {
    if (digitalRead(LEFT_ENC_A)) { // rising edge
        digitalRead(LEFT_ENC_B) ? raw_pos[0]-- : raw_pos[0]++;
    } else { // falling edge
        digitalRead(LEFT_ENC_B) ? raw_pos[0]++ : raw_pos[0]--;
    }
}

void LEFT_ENC_B_handler() {
    if (digitalRead(LEFT_ENC_B)) { // rising edge
        digitalRead(LEFT_ENC_A) ? raw_pos[0]++ : raw_pos[0]--;
    } else { // falling edge
        digitalRead(LEFT_ENC_A) ? raw_pos[0]-- : raw_pos[0]++;
    }
}

void RIGHT_ENC_A_handler() {
    if (digitalRead(RIGHT_ENC_A)) { // rising edge
        digitalRead(RIGHT_ENC_B) ? raw_pos[1]-- : raw_pos[1]++;
    } else { // falling edge
        digitalRead(RIGHT_ENC_B) ? raw_pos[1]++ : raw_pos[1]--;
    }
}

void RIGHT_ENC_B_handler() {
    if (digitalRead(RIGHT_ENC_B)) { // rising edge
        digitalRead(RIGHT_ENC_A) ? raw_pos[1]++ : raw_pos[1]--;
    } else { // falling edge
        digitalRead(RIGHT_ENC_A) ? raw_pos[1]-- : raw_pos[1]++;
    }
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
	if ( output_pwm[0] < 0) {
		L_motor.forward();
	} else {
		L_motor.backward();
	}

	R_motor.setSpeed(abs(output_pwm[1]));
	if ( output_pwm[1] < 0) {
		R_motor.forward();
	} else {
		R_motor.backward();
	}
}

void cmd_special_cb(const std_msgs::String& str) {
	String s(str.data);
	if (s == String("open")) {
		grip_servo.write(130);
		nh.loginfo(String("opening").c_str());
	} else if (s == String("close")) {
		grip_servo.write(0);
		nh.loginfo(String("closing").c_str());
	}
	nh.loginfo(String("got cmd_special: " + String(str.data)).c_str());
}


void cmd_vel_cb(const geometry_msgs::Twist& twist_input) {
	goal_speed = twist_input.linear.x;
	goal_angular = twist_input.angular.z;

	nh.loginfo(String("Got cmd_vel: " + String(goal_speed, 3) +
	           " ," + String(goal_angular, 3)).c_str());
	nh.loginfo(String("goal_pos: " + String(goal_pos[0], 3) +
	           " ," + String(goal_pos[1], 3)).c_str());
}

void PRINT_oscilloscope(double val, double goal_val, unsigned long period, double inc, double minf, double maxf){
	unsigned long t = micros();
    if (micros() - last_oscope_print > period) {
		Serial.print(":");
        for (double i = -minf; i < maxf; i+=inc){
            if (abs(val - i) <= 0.5*inc){
                Serial.print('|');
            } else if (abs(i - goal_val) <= 0.5*inc) {
                Serial.print('#');
            } else {
                Serial.print(' ');
            }
        }
        last_oscope_print = t;
		Serial.print(":");
        // Serial.print(last_oscope_print);
    }
}

void update_odom()
{
	static double x_pos = 0;
	static double y_pos = 0;
	static double th_pos = 0;
	static long old_raw_pos[] = {0, 0};
	static long old_micros = micros();
	static long seq_count = 0;

	long new_micros = micros();
	double dt = new_micros - old_micros;

	long new_raw_pos[] = {raw_pos[0], raw_pos[1]};
	int d_encoder[2];

	d_encoder[0] = new_raw_pos[0] - old_raw_pos[0];
	d_encoder[1] = new_raw_pos[1] - old_raw_pos[1];

	double d_center  = (d_encoder[0] * raw_to_meters[0]
	                  + d_encoder[1] * raw_to_meters[0]) / 2.0;
	double d_th      = (d_encoder[1] * raw_to_meters[1]
	                  - d_encoder[0] * raw_to_meters[0]) / wheelbase;

	x_pos += d_center * cos(th_pos);
	y_pos += d_center * sin(th_pos);
	th_pos += d_th;

	if (th_pos >= 2.0 * PI) {
		th_pos -= 2.0 * PI;
	} else if (th_pos <= -2.0 * PI) {
		th_pos += 2.0 * PI;
	}

	pose_msg.position.x = x_pos;
	pose_msg.position.y = y_pos;
	pose_msg.position.z = 0.0;

	pose_msg.orientation.x = 0.0;
	pose_msg.orientation.y = 0.0;
	pose_msg.orientation.z = sin(th_pos * 0.5);
	pose_msg.orientation.w = cos(th_pos * 0.5);

	// odom_msg.pose.pose.position.x = x_pos;
	// odom_msg.pose.pose.position.y = y_pos;
	// odom_msg.pose.pose.position.z = 0.0;

	// odom_msg.pose.pose.orientation.x = 0.0;
	// odom_msg.pose.pose.orientation.y = 0.0;
	// odom_msg.pose.pose.orientation.z = sin(th_pos * 0.5);
	// odom_msg.pose.pose.orientation.w = cos(th_pos * 0.5);

	// odom_msg.twist.twist.linear.x = d_center/dt;
	// odom_msg.twist.twist.linear.y = 0.0;
	// odom_msg.twist.twist.linear.z = 0.0;

	// odom_msg.twist.twist.angular.y = 0.0;
	// odom_msg.twist.twist.angular.z = d_th/dt;

	// odom_msg.header.seq = seq_count++;
	// odom_msg.header.stamp = nh.now();
	// odom_msg.header.frame_id = odom_tf;
	// odom_msg.child_frame_id = base_link_tf;	

	pose_pub.publish(&pose_msg);
	nh.logerror(String(th_pos).c_str());

	old_raw_pos[0] = new_raw_pos[0];
	old_raw_pos[1] = new_raw_pos[1];
	old_micros = new_micros;
}
