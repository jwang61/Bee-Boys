#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"

#include <pose_estimate/detector.h>

using namespace std;
using namespace cv;

#define USE_CAMERA
#define PUBLISH_MSG

#ifdef USE_CAMERA
#define CAMERA_ID 0 // Camera device number
#endif
#define FRAME_RATE 30

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
    geometry_msgs::Twist vel;
#endif

#ifdef USE_CAMERA
    VideoCapture cap;
    Mat frame;
    cap.open(CAMERA_ID);

    if (! cap.isOpened())
    {
        cout << "--(!)Error opening video capture\n";
        return -1;
    }
#else
    Mat frame = imread("/home/justin/github/Bee-Boys/cv/data/28_5.jpg");
#endif

    Detector detector(mode, FRAME_RATE);

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

#ifdef USE_CAMERA
        cap >> frame;
#endif

        result = detector.load_frame(frame);
        if (!result)
            break;

#ifdef PUBLISH_MSG
        vel.linear = detector.process();
        vel_pub.publish(vel);
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

#ifdef USE_CAMERA
    cap.release();
#endif

    return 0;
}
