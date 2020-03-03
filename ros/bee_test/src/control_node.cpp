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
#include <mavros_msgs/PositionTarget.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
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
    // pos_msg.velocity.x = 0.5;
    pos_msg.velocity.z = 0.1;

    ros::Time last_time = ros::Time::now();
    bool first_offboard = true;;

    while(ros::ok()){
        if (current_state.mode == "OFFBOARD") {
            if (first_offboard)
            {
                last_time = ros::Time::now();
                first_offboard =  false;
                pos_msg.velocity.z = 0.1;
                ROS_INFO("Offboard Enabled");
            }
        }
        else
        {
            first_offboard = true;
        }
        if (ros::Time::now() - last_time > ros::Duration(5.0))
        {
            pos_msg.velocity.z = 0.0;
        }
        vel_pub.publish(pos_msg);
        ROS_INFO("Vel Z: %f", pos_msg.velocity.z);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
