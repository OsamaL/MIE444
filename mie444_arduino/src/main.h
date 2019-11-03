#ifndef main_h
#define main_h

void encoderLeftMotor();
void encoderRightMotor();
void update_PID();
void setup_PID();
void setup_encoders();
void update_motors();
void cmd_vel_cb(const geometry_msgs::Twist& twist_input);

#define L 0
#define R 1


#endif
