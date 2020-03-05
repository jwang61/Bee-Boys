#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include <geometry_msgs/Vector3.h>
#define USE_CAM_TOPIC

namespace VideoModes
{
    enum VideoMode
    {
        NONE           = 0,
        DISPLAY_RAW    = 1,
        DISPLAY_DETECT = 2,
        SAVE_RAW       = 4,
        SAVE_DETECT    = 8
    };
}
typedef VideoModes::VideoMode VideoMode;

class Detector {
public:
    bool position_locked;
    // Constructor
    Detector(int camera_id, int fps, int video_mode = 2);

    // Destructor
    virtual ~Detector();

    bool load_cascade(cv::String);

#ifdef USE_CAM_TOPIC
    bool load_frame(cv::Mat frame);
#else
    bool load_frame();
#endif

    geometry_msgs::Vector3 process();

private:
    cv::CascadeClassifier cascade;

    cv::Mat raw_frame;
    cv::Mat detect_frame;

    cv::VideoCapture cap;

    cv::VideoWriter raw_video;
    cv::VideoWriter detect_video;

    bool loaded;
    bool display_raw;
    bool display_detect;
    bool save_raw;
    bool save_detect;

    void display_detection(cv::Rect, bool);

    int frame_width;
    int frame_height;
    int frame_rate;

    // fuck camera matrices
    //cv::Mat camera_matrix;
    //cv::Mat distortion;
};
