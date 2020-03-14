#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <pose_estimate/detector.h>

// For PX4
#define CENTER_X 320
#define CENTER_Y 240
#define CENTER_THRESHOLD 50
#define DETECT_SIZE 275
#define DETECT_THRESHOLD 50

Detector::Detector(int camera_id, int fps, int video_mode) :
    loaded(false),
    position_locked(false)
{
    display_raw    = video_mode & static_cast<int>(VideoModes::DISPLAY_RAW);
    display_detect = video_mode & static_cast<int>(VideoModes::DISPLAY_DETECT);
    save_raw       = video_mode & static_cast<int>(VideoModes::SAVE_RAW);
    save_detect    = video_mode & static_cast<int>(VideoModes::SAVE_DETECT);

    frame_width = 640;
    frame_height = 480;

    if (save_raw)
    {
        raw_video.open("raw.avi", cv::VideoWriter::fourcc('M','J','P','G'), fps, cv::Size(frame_width,frame_height));
    }

    if (save_detect)
    {
        detect_video.open("detect.avi", cv::VideoWriter::fourcc('M','J','P','G'), fps, cv::Size(frame_width,frame_height));
    }

    // Let's not use camera matrices??
    //float cam_tmp[9] = {78.34164567, 0, 319.5, 0, 78.34164567, 239.5, 0, 0, 1};
    ////float dis_tmp[5] = {-0.022528287754063452, 0.00061097745, 0., 0., -0.000005983169};
    //float dis_tmp[5] = {0,0,0,0,0};
    //camera_matrix = cv::Mat(3, 3, CV_32F, cam_tmp);
    //distortion = cv::Mat(1, 5, CV_32F, dis_tmp);
}

Detector::~Detector()
{
    cap.release();
    if (save_raw)
        raw_video.release();
    if (save_detect)
        detect_video.release();
}

bool Detector::load_cascade(cv::String cascade_file)
{
    if(cascade.load(cascade_file))
        loaded = true;
    return loaded;
}

bool Detector::load_frame(cv::Mat frame)
{
    raw_frame = frame;

    if(raw_frame.empty())
        return false;
    cv::cvtColor( raw_frame, detect_frame, cv::COLOR_BGR2GRAY );
    //cv::equalizeHist( detect_frame, detect_frame );
    //cv::flip(detect_frame, detect_frame, 0);
    //cv::undistort(raw_frame, trans_frame, camera_matrix, distortion);
    return true;
}


geometry_msgs::Vector3 Detector::process()
{
    geometry_msgs::Vector3 vel;
    if (!loaded)
    {
        std::cout << "ERROR: NO CASCADE FILE!!" << std::endl;
        return vel;
    }
    std::vector<cv::Rect> detections;
    cascade.detectMultiScale(detect_frame, detections);
    cv::Rect main_detection;
    bool detected = (detections.size() > 0);
    for (auto rect : detections)
    {
        if (rect.width > main_detection.width)
            main_detection = rect;
    }

    position_locked = false;
    if (detected)
    {
        int center_x = main_detection.x + main_detection.width/2;
        int center_y = main_detection.y + main_detection.height/2;

        if (center_x > CENTER_X + CENTER_THRESHOLD)
            vel.y = 1;  // Move right
        else if (center_x < CENTER_X - CENTER_THRESHOLD)
            vel.y = -1; // Move left

        if (center_y > CENTER_Y + CENTER_THRESHOLD)
            vel.z = -1;  // Move down
        else if (center_y < CENTER_Y - CENTER_THRESHOLD)
            vel.z = 1; // Move up

        if (main_detection.width > DETECT_SIZE + DETECT_THRESHOLD)
            vel.x = 2; // Move back
        else if (main_detection.width < DETECT_SIZE - DETECT_THRESHOLD)
            vel.x = -2;  // Move forward
        std::cout << "Detected at: " << center_x << ", " << center_y
                  << " with size of " << main_detection.width << std::endl;
        std::cout << vel << std::endl;
        if (vel.x == 0 && vel.y == 0 && vel.z == 0)
            position_locked = true;
    }
    else
    {
        std::cout << "NO DETECTIONS " << std::endl;
    }

    if (display_raw)
        cv::imshow("raw footage", raw_frame);
    if (display_detect)
        display_detection(main_detection, detected);
    if (save_raw)
        raw_video.write(raw_frame);
    if (save_detect)
        detect_video.write(detect_frame);

    return vel;
}

// TODO: Add more info to display
void Detector::display_detection(cv::Rect detection, bool detected)
{
    cv::cvtColor(detect_frame, detect_frame, CV_GRAY2RGB);
    if (detected)
    {
        cv::Point center(detection.x + detection.width/2,
                         detection.y + detection.height/2);
        cv::ellipse(detect_frame, center,
                    cv::Size(detection.width/2, detection.height/2),
                    0, 0, 360, cv::Scalar(255, 0, 255), 4 );
        // Show center lines
        cv::line(detect_frame, cv::Point(0, CENTER_Y), cv::Point(frame_width, CENTER_Y), cv::Scalar(0, 0, 255), 4);
        cv::line(detect_frame, cv::Point(CENTER_X, 0), cv::Point(CENTER_X, frame_height), cv::Scalar(0, 0, 255), 4);
    }
    else
    {
        cv::putText(detect_frame, "No Flowers Found", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 1,
                    cv::Scalar(0, 0, 255), 4);
    }
    cv::imshow("detection frame", detect_frame);
}
