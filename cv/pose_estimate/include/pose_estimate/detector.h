#include "opencv2/objdetect.hpp"
#include <geometry_msgs/Vector3.h>

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
    // Constructor
    Detector(int video_mode = 2, int frame_rate = 30);

    // Destructor
    virtual ~Detector();

    bool load_cascade(cv::String);

    bool load_frame(cv::Mat);

    geometry_msgs::Vector3 process();

private:
    cv::CascadeClassifier cascade;

    cv::Mat raw_frame;
    cv::Mat detect_frame;

    bool loaded;
    bool display_raw;
    bool display_detect;
    bool save_raw;
    bool save_detect;

    void display_detection(cv::Mat, cv::Rect, bool);

    int frame_rate;

    // fuck camera matrices
    //cv::Mat camera_matrix;
    //cv::Mat distortion;
};
