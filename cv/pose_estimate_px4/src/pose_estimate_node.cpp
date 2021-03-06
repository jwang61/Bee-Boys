#include <pose_estimate/detector.h>

#include <iostream>
#include <ros/ros.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include <std_msgs/Char.h>

#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace cv;

#define PUBLISH_MSG

float speed(0.3);

std_msgs::Char key;
bool teleop_active = true;

void keyboard_cb(const std_msgs::Char::ConstPtr& msg){
    key = *msg;
}

int main( int argc, char** argv )
{
    std::string cascade;
    int video_mode, camera_id, frame_rate;
    ros::init(argc, argv, "pose_estimate_node");
    ros::NodeHandle nh("~");
    nh.getParam("cascade", cascade);
    nh.param("video_mode", video_mode, 2);
    nh.param("frame_rate", frame_rate, 30);
    nh.param("camera_id", camera_id, 1);

#ifdef PUBLISH_MSG
    ros::Subscriber keyboard_state = nh.subscribe<std_msgs::Char>
            ("/keyboard_publisher", 10, keyboard_cb);
    ros::Publisher vel_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);
    ros::Publisher servo_angle_pub = nh.advertise<std_msgs::Float32>("/servo_angle_publisher", 10);
    std_msgs::Float32 servo_angle;
    mavros_msgs::PositionTarget pos_msg;
    pos_msg.coordinate_frame = 1;
    pos_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX
                        | mavros_msgs::PositionTarget::IGNORE_PY
                        | mavros_msgs::PositionTarget::IGNORE_PZ
                        | mavros_msgs::PositionTarget::IGNORE_YAW
                        | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
#endif

    ROS_INFO("CREATING DETECTOR...");
    Detector detector(camera_id, frame_rate, video_mode);

    ROS_INFO("LOADING CASCADE...");
    if (!detector.load_cascade(cascade))
        ROS_INFO("NO LOADED CASCADE");
    else
        ROS_INFO("LOADED CASCADE");

    int im_key;
    bool result;
    ros::Time watchdog;
    float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
    ros::Rate rate(frame_rate);

    //int nsecs = ros::Time::now().nsec;
    //int prev_time = nsecs;
    while (1)
    {
        ros::spinOnce();
        result = detector.load_frame();
        if (!result)
            break;

#ifdef PUBLISH_MSG
        if (teleop_active) {
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
            } else if (key.data == 'j') {
                x = 1;
                y = 0;
                z = 0;
            } else if (key.data == 'l') {
                x = -1;
                y = 0;
                z = 0;
            // Flipping l and j for ardrone
            } else if (key.data == 'i') {
                x = 0;
                y = -1;
                z = 0;
            } else if (key.data == ',') {
                x = 0;
                y = 1;
                z = 0;
            } 
            else if (key.data == 's')
            {
                teleop_active = false;
            }
            else if (key.data == 'a')
            {
                break;
            }
            // Otherwise, set the robot to stop
            else
            {
                x = 0;
                y = 0;
                z = 0;
            }
            ROS_INFO_STREAM("KEY " << key.data);
            detector.process();
            pos_msg.velocity.x = x* speed;
            pos_msg.velocity.y = y*speed;
            pos_msg.velocity.z = z*speed;
        } else {
            if (key.data == 'd') {
                teleop_active = true;
                pos_msg.velocity.x = 0;
                pos_msg.velocity.y = 0;
                pos_msg.velocity.z = 0;
            } else if (key.data == 'a') {
                break;
            } else {
                pos_msg.velocity = detector.process();
                pos_msg.velocity.x *= speed;
                pos_msg.velocity.y *= speed;
                pos_msg.velocity.z *= speed;
            }
            
        }
        
        vel_pub.publish(pos_msg);
        if (detector.locked)
        {
            if (ros::Time::now() - watchdog > ros::Duration(3.0))
                detector.locked = false;
        }
        else
        {
            if (detector.position_locked)
            {
                detector.locked = true;
                watchdog = ros::Time::now();
            }
        }
        servo_angle.data = (detector.locked) ? 0.3 : 1.0;
        servo_angle_pub.publish(servo_angle);
#else
        detector.process();
#endif

        im_key = waitKey(3);
        if (im_key == 27)
            break; // escape
        rate.sleep();
        //nsecs = ros::Time::now().nsec;
        //cout << "TIME: " << prev_time - nsecs << endl;
	//prev_time = nsecs;

    }

    return 0;
}
