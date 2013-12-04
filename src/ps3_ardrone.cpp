/*
PS3 control of the ardrone quadcopter
2013 Kenneth Bogert
University of Georgia

Adapted From: Parker Conroy
Algorithmic Robotics Lab @ University of Utah


*/

/* TODO:
use the current rotation of the quadcopter to provide a "start position relative" control scheme
	need 3 control schemes:
	local 3rd person
	relative 3rd person
	local 1st person

	Is the local 1st person control even worth it?

*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ardrone_autonomy/Navdata.h>

#define PS3_AXIS_SCALE 1

int x_axis, y_axis, z_axis1, z_axis2, theta_axis, but_takeoff, but_trim, but_switch_cam, but_speed_up, but_speed_down, but_ctrl_scheme=0;

int drone_state, control_scheme=0;
double drone_rotation,basis_rotation=0;

double joy_x,joy_y,joy_z,joy_theta, speed, deadzone=0;
int last_but_takeoff, last_but_trim, last_but_switch_cam, last_but_speed_up, last_but_speed_down, last_but_ctrl_scheme=0;

geometry_msgs::Twist twist_msg;
std_msgs::Empty emp_msg;
std_srvs::Empty emp_srv;

float map(float value, float in_min, float in_max, float out_min, float out_max) {
  return (float)((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}


inline int is_flying() {
	if (drone_state >= 3 && drone_state <= 8)
		return 1;

	return 0;
}

inline double to_rad(double degrees) {
	return degrees * 3.1415926 / 180.0;
}

ros::Publisher pub_twist;
ros::Publisher pub_takeoff;
ros::Publisher pub_land;
ros::Publisher pub_reset;
ros::ServiceClient srv_trim;
ros::ServiceClient srv_switch_cam;


void sendResetMsg() {
    pub_reset.publish(emp_msg);
}

void sendTakeoffOrLandMsg() {

    if (drone_state <= 1) {
	sendResetMsg();
	return;
    }

    if (is_flying()) {
	pub_land.publish(emp_msg); //lands the drone

    } else {
	pub_takeoff.publish(emp_msg); //launches the drone
    }

}

void sendTrimMsg() {

    if (drone_state <= 1) {
	sendResetMsg();
	return;
    }

    srv_trim.call(emp_srv);
    basis_rotation = drone_rotation;
}

void sendSwitchCamMsg() {
    srv_switch_cam.call(emp_srv);
}


void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
    //Take in joystick
    joy_x=map(joy_msg_in.axes[x_axis],-PS3_AXIS_SCALE,PS3_AXIS_SCALE,-1,1);
    if (fabs(joy_x) < deadzone)
	joy_x = 0;
    joy_y=map(joy_msg_in.axes[y_axis],-PS3_AXIS_SCALE,PS3_AXIS_SCALE,-1,1);
    if (fabs(joy_y) < deadzone)
	joy_y = 0;
    joy_z=map(joy_msg_in.axes[z_axis1],0,-PS3_AXIS_SCALE,0,1) - map(joy_msg_in.axes[z_axis2],0,-PS3_AXIS_SCALE,0,1);
    if (fabs(joy_z) < deadzone)
	joy_z = 0;
    joy_theta=map(joy_msg_in.axes[theta_axis],-PS3_AXIS_SCALE,PS3_AXIS_SCALE,-1,1);
    if (fabs(joy_theta) < deadzone)
	joy_theta = 0;


    if (joy_msg_in.buttons[but_takeoff] && ! last_but_takeoff)
	sendTakeoffOrLandMsg();

    if (joy_msg_in.buttons[but_trim] && ! last_but_trim && ! is_flying())
	sendTrimMsg();

    if (joy_msg_in.buttons[but_switch_cam] && ! last_but_switch_cam)
	sendSwitchCamMsg();

    if (joy_msg_in.buttons[but_ctrl_scheme] && ! last_but_ctrl_scheme)
	control_scheme = ! control_scheme;

    if (joy_msg_in.buttons[but_speed_up] && ! last_but_speed_up) {
	speed += .1f;
	if (speed > 1)
		speed = 1;
    }

    if (joy_msg_in.buttons[but_speed_down] && ! last_but_speed_down) {
	speed -= .1f;
	if (speed < 0.1)
		speed = 0.1;
    }


    if (is_flying()) {

	if (control_scheme == 1) {
	    // rotate the current control by the drone's rotation (relative to basis rotation)
	    double rot = (drone_rotation - basis_rotation);
	    double tmp_joy_y = joy_y * cos(rot) - joy_x * sin(rot);
	    double tmp_joy_x = joy_y * sin(rot) + joy_x * cos(rot);
	    joy_x = tmp_joy_x;
	    joy_y = tmp_joy_y;
	}
	
	twist_msg.linear.x=joy_x*speed;
	twist_msg.linear.y=joy_y*speed;
	twist_msg.linear.z=joy_z*speed;
	twist_msg.angular.z=joy_theta*speed;
	twist_msg.angular.x=0;
	twist_msg.angular.y=0;

	pub_twist.publish(twist_msg); //send message to the robot
    }


    last_but_takeoff = joy_msg_in.buttons[but_takeoff];
    last_but_trim = joy_msg_in.buttons[but_trim];
    last_but_switch_cam = joy_msg_in.buttons[but_switch_cam];
    last_but_speed_up = joy_msg_in.buttons[but_speed_up];
    last_but_speed_down = joy_msg_in.buttons[but_speed_down];
    last_but_ctrl_scheme = joy_msg_in.buttons[but_ctrl_scheme];

}


void navdata_callback(const ardrone_autonomy::Navdata &navdata_msg_in) {


    // save current drone status
    drone_state = navdata_msg_in.state;
	
    // save current drone rotation

    drone_rotation = to_rad(navdata_msg_in.rotZ);
}
	
int main(int argc, char** argv)
{
    speed = 0.5;

    ros::init(argc, argv,"PS3_Ardrone_Control");
    ros::NodeHandle node;
    ros::Subscriber joy_sub;
    ros::Subscriber navdata_sub;

    ros::NodeHandle ph("~");

    ph.param("deadzone", deadzone, 0.0);
    ph.param("x_axis", x_axis, 3);
    ph.param("y_axis", y_axis, 2);
    ph.param("z_axis1", z_axis1, 9);
    ph.param("z_axis2", z_axis2, 8);
    ph.param("theta_axis", theta_axis, 0);
    ph.param("but_takeoff", but_takeoff, 3);
    ph.param("but_trim", but_trim, 0);
    ph.param("but_ctrl_scheme", but_ctrl_scheme, 12);
    ph.param("but_switch_cam", but_switch_cam, 13);
    ph.param("but_speed_up", but_speed_up, 4);
    ph.param("but_speed_down", but_speed_down, 6);

    pub_twist = node.advertise<geometry_msgs::Twist>("cmd_vel", 1); //send robot input on /cmd_vel topic
    joy_sub = node.subscribe("/joy", 1, joy_callback); //suscribe to the joystick message
    navdata_sub = node.subscribe("ardrone/navdata", 1, navdata_callback); //suscribe to the joystick message

    pub_takeoff = node.advertise<std_msgs::Empty>("ardrone/takeoff", 1); //send robot input on /cmd_vel topic
    pub_land = node.advertise<std_msgs::Empty>("ardrone/land", 1); //send robot input on /cmd_vel topic
    pub_reset = node.advertise<std_msgs::Empty>("ardrone/reset", 1); //send robot input on /cmd_vel topic
    srv_trim = node.serviceClient<std_srvs::Empty>("ardrone/flattrim");
    srv_switch_cam = node.serviceClient<std_srvs::Empty>("ardrone/togglecam");


    ros::spin();

}//main


