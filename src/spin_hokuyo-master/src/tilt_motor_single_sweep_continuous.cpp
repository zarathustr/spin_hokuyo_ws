#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<dynamixel_msgs/JointState.h>
#include<cmath>
#include<laser_assembler/AssembleScans.h>
#include<ros/time.h>
#include<std_msgs/Time.h>
#include<std_msgs/Empty.h>

//This code allows the motor to move back and forth to a maximum and minimum number of degrees defined in parameters.
//This iteration has the servo initilize and then move back and forth continuously.
//It also sends times to the combine_clouds_subscriber to allow point clouds to be compiled.

using namespace std;

//global variables
float error;
int max_angle;
int min_angle;
float pause_time;
#define MODE_SWEEP	0
#define MODE_VEL	1
int mode;
double vel;

//obtains error from dynamixel message
void obtainValues(const dynamixel_msgs::JointState &msg) {
    error = msg.error;
}

//creates all commands for the motor
class Odrive {
    private:
    ros::NodeHandle nh;
    ros::Publisher pub_1;
    ros::Publisher pub_2;
    ros::Publisher pub_3;
    ros::Subscriber sub;

    public:
    Odrive();
    void checkError();
    void moveMotor(double position);
    void startTime();
    void endTime();
    void publishVel();
};

//creates publishers and subscriber
Odrive::Odrive() {
    //publish motor commands
    pub_1 = nh.advertise<std_msgs::Float64>("/tilt_controller/command", 10);
    //publish start time of sweep
    pub_2 = nh.advertise<std_msgs::Time>("/time/start_time", 1);
    //publish end time of sweep
    pub_3 = nh.advertise<std_msgs::Time>("/time/end_time", 1);
    //subscribe to motor error messages
    sub   = nh.subscribe("/tilt_controller/state", 1, &obtainValues);
}

//creates message and publishes -> degree to radian to publish
void Odrive::moveMotor(double position) {
    double convert = (position * 3.14/180);
    
    std_msgs::Float64 aux;
    aux.data = convert;
    pub_1.publish(aux);
    ROS_INFO_STREAM(aux);
}

//ensures proper alignment
void Odrive::checkError() {
    ros::spinOnce(); //get starting value of motor position error

    while(mode == MODE_SWEEP && (abs (error))>0.05) { //keep waiting and checking error until < 0.05
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
}

//publishes start time for cloud compiler
void Odrive::startTime() {
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    pub_2.publish(msg);
}

//publishes end time for cloud compiler
void Odrive::endTime() {
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    pub_3.publish(msg);
}

void Odrive::publishVel() {
    std_msgs::Float64 aux;
    aux.data = vel;
    pub_1.publish(aux);
}

//initilazies motor to min angle
void initialize(){
    if(mode == MODE_SWEEP)
    {
    	Odrive motor_1;  //Creates class object only used in the single instance that this function is run

    	motor_1.moveMotor(min_angle);
    	ros::Duration(pause_time).sleep();
    	motor_1.checkError();
    	ros::Duration(pause_time).sleep();
    }
}

//performs one sweep min -> max -> min
void sweep()
{
    Odrive motor;

    
    motor.startTime();

    if(mode == MODE_SWEEP)
    {
    	motor.moveMotor(max_angle);
    	ros::Duration(pause_time).sleep();
    	motor.checkError();
    	ros::Duration(pause_time).sleep();

    	motor.moveMotor(min_angle);
    	ros::Duration(pause_time).sleep();
    	motor.checkError();
    	ros::Duration(pause_time).sleep();
    	ROS_INFO("Finished One Sweep!");
    }
    else
    {
	motor.publishVel();
	motor.checkError();
    }
    motor.endTime();
}

//main
int main(int argc, char **argv) {

    //initialize
    ros::init(argc, argv, "Motor_Tilt");
    ros::NodeHandle nh;

    //variables
    int max;
    int min;
    double pause;
    Odrive motor;

    //establish parameters
    nh.param("maximum", max, 180);
    nh.param("minimum", min, -180);
    nh.param("pause", pause, 1.0);
    nh.param("mode", mode, MODE_VEL);
    nh.param("vel", vel, 0.8);

    //transfer parameters to global variables
    max_angle = max;
    min_angle = min;
    pause_time = pause;

    if(mode == MODE_SWEEP)
    	ROS_INFO("Sweep Mode!");
    else
    {
	ROS_INFO("Velocity Mode!");
	ROS_INFO("Running with Speed: %f", vel);
    }
    //Wait for servo init by waiting for /state message
    ros::topic::waitForMessage<dynamixel_msgs::JointState>("/tilt_controller/state", ros::Duration(100));

    //pause to allow motor object to initialize and set to min_angle
    ros::Duration(1).sleep();
    initialize();    

    //continuously perform sweeps
    while(ros::ok()) {
        sweep();
    }
}
