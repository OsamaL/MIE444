#include <Arduino.h>
#include <L298N.h>
#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
// #include <std_msgs/String.h>
// #include <geometry_msgs/Vector3Stamped.h>
// #include <geometry_msgs/Twiscdt.h>
// #include <nav_msgs/Odometry.h>
// #include <ros/time.h>
#include <main.h>

//initializing all the variables
const int LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

const int PIN_ENCOD_A_MOTOR_LEFT = 18;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 19;               //B channel for encoder of left motor
const int PIN_ENCOD_A_MOTOR_RIGHT = 20;              //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 21;              //B channel for encoder of right motor 

const int ENCODER_TICKS_PER_REV = 32;
const int ENCODER_TICKS_TOLERANCE = 1;               //32 ticks if counting rising and falling for one encoder channel

unsigned long lastMilli = 0;
const double radius = 0.034;                   //Wheel radius, in m
const double wheelbase = 0.2;               //Wheelbase, in m

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 

const double max_speed = 255;                 //Max speed in m/s

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 

// PID Parameters
const double PID_left_param[] = { 1, 0.1, 0.1 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 1, 0.0, 0.1 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor

void setup() {

}



//Left motor encoder counter
void encoderLeftMotor() {
	if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
	else pos_left--;
}

//Right motor encoder counter
void encoderRightMotor() {
	if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right--;
	else pos_right++;
}
