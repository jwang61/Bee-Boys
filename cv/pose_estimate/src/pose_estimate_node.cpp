#include <pose_estimate/detector.h>
#ifdef USE_CAM_TOPIC
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#endif

#include <iostream>
#include <ros/ros.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"

#define ARDRONE
#ifdef ARDRONE
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#else
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Float32.h>
#endif


using namespace std;
using namespace cv;

#define PUBLISH_MSG

#ifdef USE_CAM_TOPIC
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
#endif

float speed(0.1);

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
#ifdef ARDRONE
    ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher land_pub = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/RANDOM", 1);
    geometry_msgs::Twist vel_msg;
    std_msgs::Empty empty_msg;
#else
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
#endif

#ifdef USE_CAM_TOPIC
    image_transport::ImageTransport it(nh);
#ifdef ARDRONE
    image_transport::Subscriber image_sub = it.subscribe("/ardrone/front/image_raw", 1, image_cb);
#else
    image_transport::Subscriber image_sub = it.subscribe("/iris/usb_cam/image_raw", 1, image_cb);
#endif
#endif

    ROS_INFO("CREATING DETECTOR...");
    Detector detector(0, frame_rate, video_mode);

    ROS_INFO("LOADING CASCADE...");
    if (!detector.load_cascade(cascade))
        ROS_INFO("NO LOADED CASCADE");
    else
        ROS_INFO("LOADED CASCADE");

    int key;
    bool result;
    ros::Rate rate(frame_rate);

    //int nsecs = ros::Time::now().nsec;
    //int prev_time = nsecs;
    while (1)
    {
        ros::spinOnce();
#ifdef USE_CAM_TOPIC
        if (frame.empty())
        {
            ROS_INFO("NO IMAGE");
            rate.sleep();
            continue;
        }
        result = detector.load_frame(frame);
#else
        result = detector.load_frame();
#endif
        if (!result)
            break;

#ifdef PUBLISH_MSG
#ifdef ARDRONE
        takeoff_pub.publish(empty_msg);
        vel_msg.linear = detector.process();
        vel_msg.linear.x *= speed;
        vel_msg.linear.y *= speed;
        vel_msg.linear.z *= speed;
        vel_pub.publish(vel_msg);
#else
        pos_msg.velocity = detector.process();
        pos_msg.velocity.x *= speed;
        pos_msg.velocity.y *= speed;
        pos_msg.velocity.z *= speed;
        vel_pub.publish(pos_msg);
        if (detector.position_locked)
            servo_angle.data = -0.5;
        else
            servo_angle.data = -1.0;
        servo_angle_pub.publish(servo_angle);
#endif
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
#ifdef PUBLISH_MSG
    for (int i = 0; i < 100; i++)
    {
        land_pub.publish(empty_msg);
        ros::spinOnce();
        rate.sleep();
    }
#endif

    return 0;
}
