/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "actuate_arm");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher actuator_pub = nh.advertise<mavros_msgs::ActuatorControl>
            ("mavros/actuator_control", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(0.333);

    mavros_msgs::ActuatorControl pos_msg;
    pos_msg.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_MANUAL_PASSTHROUGH;

    ros::Time last_time = ros::Time::now();
    int count = 0;
    while(ros::ok()){
        // ros::Duration(1).sleep();
        // dont disarm for real code! ! ! ! ! !!  
        pos_msg.controls[5] = (pos_msg.controls[5] > -0.5) ? -1 : 0;
        pos_msg.header.stamp = ros::Time::now();
      
        actuator_pub.publish(pos_msg);
        ros::spinOnce();
        rate.sleep();
        count++;
    }

    return 0;
}
