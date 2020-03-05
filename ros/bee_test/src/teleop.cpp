#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include "std_msgs/Char.h"

// Init variables
float speed(0.25); // Linear velocity (m/s)
float turn(1.0); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
std_msgs::Char key;

void state_cb(const std_msgs::Char::ConstPtr& msg){
    key = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop");
    ros::NodeHandle nh;

    ros::Subscriber keyboard_state = nh.subscribe<std_msgs::Char>
            ("keyboard_publisher", 10, state_cb);
    ros::Publisher vel_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    mavros_msgs::PositionTarget pos_msg;
    pos_msg.coordinate_frame = 1;
    pos_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX
                        | mavros_msgs::PositionTarget::IGNORE_PY
                        | mavros_msgs::PositionTarget::IGNORE_PZ
                        | mavros_msgs::PositionTarget::IGNORE_YAW
                        | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    while(ros::ok()){

        if(key.data == 'o') {
            x = 0;
            y = 0;
            z = 1;
        } else if (key.data == 'k') {
            z = 0;
            x = 0;
            y = 0;
        } else if (key.data == 'u') {
            x = 0;
            y = 0;
            z = -1;
        } else if (key.data == 'i') {
            x = 1;
            y = 0;
            z = 0;
        } else if (key.data == ',') {
            x = -1;
            y = 0;
            z = 0;
        } else if (key.data == 'l') {
            x = 0;
            y = 1;
            z = 0;
        } else if (key.data == 'j') {
            x = 0;
            y = -1;
            z = 0;
        } 
        // Otherwise, set the robot to stop
        else
        {
            x = 0;
            y = 0;
            z = 0;
        }

        pos_msg.velocity.x = x * speed;
        pos_msg.velocity.y = y * speed;
        pos_msg.velocity.z = z * speed;
        
        vel_pub.publish(pos_msg);

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
