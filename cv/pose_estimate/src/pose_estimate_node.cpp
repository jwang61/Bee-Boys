#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include <std_msgs/Float32.h>

#include <pose_estimate/detector.h>

using namespace std;
using namespace cv;

#define PUBLISH_MSG

#define CAMERA_ID 0 // Camera device number
#define FRAME_RATE 30

float speed(0.25);

int main( int argc, char** argv )
{
    std::string param;
    int mode;
    ros::init(argc, argv, "pose_estimate_node");
    ros::NodeHandle nh("~");
    nh.getParam("cascade", param);
    nh.param("mode", mode, 2);

#ifdef PUBLISH_MSG
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    ros::Publisher servo_angle_pub = nh.advertise<std_msgs::Float32>("/servo_angle_publisher", 10);
    geometry_msgs::Twist vel;
    std_msgs::Float32 servo_angle;
#endif

    Detector detector(CAMERA_ID, FRAME_RATE, mode);

    if (!detector.load_cascade(param))
        cout << "Failed to load cascade" << endl;

    int key;
    bool result;
    ros::Rate rate(FRAME_RATE);
    //int nsecs = ros::Time::now().nsec;
    //int prev_time = nsecs;
    while (1)
    {
        ros::spinOnce();

        result = detector.load_frame();
        if (!result)
            break;

#ifdef PUBLISH_MSG
        vel.linear = detector.process();
	    vel.linear.x *= speed;
	    vel.linear.y *= speed;
	    vel.linear.z *= speed;
        vel_pub.publish(vel);
        if (detector.position_locked)
            servo_angle.data = 45;
        else
            servo_angle.data = 0.0;
        servo_angle_pub.publish(servo_angle);
#else
        detector.process();
#endif

        key = waitKey(3);
        if (key == 27)
            break; // escape
        rate.sleep();
        //nsecs = ros::Time::now().nsec;
        //cout << "TIME: " << prev_time - nsecs << endl;
        //prev_time = nsecs;

    }

    return 0;
}
