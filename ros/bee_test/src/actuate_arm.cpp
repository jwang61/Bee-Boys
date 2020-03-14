#include <ros/ros.h>
#include <mavros_msgs/ActuatorControl.h>
#include <std_msgs/Float32.h>

std_msgs::Float32 servo_angle;

void state_cb(const std_msgs::Float32::ConstPtr& msg){
    servo_angle = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "actuate_arm");
    ros::NodeHandle nh;

    ros::Subscriber servo_angle_sub = nh.subscribe<std_msgs::Float32>
            ("servo_angle_publisher", 10, state_cb);
    ros::Publisher actuator_pub = nh.advertise<mavros_msgs::ActuatorControl>
            ("mavros/actuator_control", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10); 

    float target = 0;
    float last_position = 0;

    mavros_msgs::ActuatorControl act_msg;
    act_msg.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_MANUAL_PASSTHROUGH;

    ros::Time last_time = ros::Time::now();
    int count = 0;
    while(ros::ok()){
        target = servo_angle.data;
        if (target != last_position){
            if (target - last_position > 0.1){
                target = last_position + 0.1;
            }
            else if (target - last_position < -0.1){
                target = last_position - 0.1;
            }
        }
        last_position = target;
        act_msg.controls[5] = target;
        act_msg.header.stamp = ros::Time::now();

        actuator_pub.publish(act_msg);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
