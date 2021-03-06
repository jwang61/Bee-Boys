#include <pose_estimate/detector.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <iostream>
#include <ros/ros.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace cv;

#define PUBLISH_MSG

Mat frame;
void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

float speed(0.05);

std_msgs::Char key;
bool teleop_active = true;

void keyboard_cb(const std_msgs::Char::ConstPtr& msg){
    key = *msg;
}

int main( int argc, char** argv )
{
    std::string cascade;
    int video_mode, frame_rate;
    ros::init(argc, argv, "pose_estimate_node");
    ros::NodeHandle nh("~");
    nh.getParam("cascade", cascade);
    nh.param("video_mode", video_mode, 2);
    nh.param("frame_rate", frame_rate, 30);

#ifdef PUBLISH_MSG
    ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher land_pub = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
    std_msgs::Empty empty_msg;

    ros::Subscriber keyboard_state = nh.subscribe<std_msgs::Char>
            ("/keyboard_publisher", 10, keyboard_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/cmd_vel", 10);
    geometry_msgs::Twist vel_msg;
#endif

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/ardrone/front/image_raw", 1, image_cb);

    ROS_INFO("CREATING DETECTOR...");
    Detector detector(0, frame_rate, video_mode);

    ROS_INFO("LOADING CASCADE...");
    if (!detector.load_cascade(cascade))
        ROS_INFO("NO LOADED CASCADE");
    else
        ROS_INFO("LOADED CASCADE");

    int im_key;
    bool result;
    float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
    ros::Rate rate(frame_rate);

    //int nsecs = ros::Time::now().nsec;
    //int prev_time = nsecs;
    while (1)
    {
        ros::spinOnce();
        if (frame.empty())
        {
            ROS_INFO("NO IMAGE");
            rate.sleep();
            continue;
        }
        result = detector.load_frame(frame);
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
            } else if (key.data == 'i') {
                x = 1;
                y = 0;
                z = 0;
            } else if (key.data == ',') {
                x = -1;
                y = 0;
                z = 0;
            // Flipping l and j for ardrone
            } else if (key.data == 'l') {
                x = 0;
                y = -1;
                z = 0;
            } else if (key.data == 'j') {
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
            takeoff_pub.publish(empty_msg);
            vel_msg.linear.x = x*speed;
            vel_msg.linear.y = y*speed;
            vel_msg.linear.z = z*speed;
            detector.process();
        } else {
            if (key.data == 'd') {
                teleop_active = true;
                vel_msg.linear.x = 0;
                vel_msg.linear.y = 0;
                vel_msg.linear.z = 0;
            } else if (key.data == 'a') {
                break;
            } else {
                takeoff_pub.publish(empty_msg);
                vel_msg.linear = detector.process();
                vel_msg.linear.x *= speed;
                vel_msg.linear.y *= speed;
                vel_msg.linear.z *= speed;
            }
            
        }
        
        vel_pub.publish(vel_msg);
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
#ifdef PUBLISH_MSG
    for (int i = 0; i < 50; i++)
    {
        land_pub.publish(empty_msg);
        ros::spinOnce();
        rate.sleep();
    }
#endif

    return 0;
}
