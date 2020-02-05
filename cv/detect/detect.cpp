#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>

using namespace std;
using namespace cv;

Mat detectAndDisplay( Mat frame );
CascadeClassifier cascade;

int main( int argc, const char** argv )
{
    CommandLineParser parser(argc, argv,
                             "{help h||}"
                             "{cascade|cascade.xml|Path to cascade.}"
                             "{camera|-1|Camera device number.}"
                             "{video||Video to cascade.}");
    parser.about( "\nThis program demonstrates using the cv::CascadeClassifier class to detect objects (Face + eyes) in a video stream.\n"
                  "You can use Haar or LBP features.\n\n" );
    parser.printMessage();
    String cascade_name = samples::findFile( parser.get<String>("cascade") );
    //-- 1. Load the cascades
    if( !cascade.load( cascade_name ) )
    {
        cout << "--(!)Error loading cascade\n";
        return -1;
    };

    String video_file = parser.get<String>("video");
    int camera_device = parser.get<int>("camera");
    VideoCapture cap;
    if (!video_file.empty())
        cap.open(video_file);
    else if (camera_device != -1)
        cap.open( camera_device );

    double fps = cap.get(CV_CAP_PROP_FPS);

    // Default resolution of the frame is obtained.The default resolution is system dependent.
    int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
    VideoWriter video("out.avi", CV_FOURCC('M','J','P','G'), fps, Size(frame_width,frame_height));
    if ( ! cap.isOpened() )
    {
        cout << "--(!)Error opening video capture\n";
        return -1;
    }
    Mat frame;
    bool capture = true;
    int key = 0;
    while (1)
    {
        cap >> frame;
        if( frame.empty() )
        {
            cout << "--(!) No more frames -- Break!\n";
            break;
        }
        //-- 3. Apply the classifier to the frame
        Mat new_frame = detectAndDisplay( frame );
        if (capture)
            video.write(new_frame);
        key = waitKey(10);
        if (key == 27)
            break; // escape
        else if (key == 32)
            capture ^= true;
    }
    cap.release();
    video.release();
    return 0;
}

Mat detectAndDisplay( Mat frame )
{
    Mat frame_gray;
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    //-- Detect flower
    std::vector<Rect> flowers;
    cascade.detectMultiScale( frame_gray, flowers );
    for ( size_t i = 0; i < flowers.size(); i++ )
    {
        Point center( flowers[i].x + flowers[i].width/2, flowers[i].y + flowers[i].height/2 );
        ellipse( frame, center, Size( flowers[i].width/2, flowers[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4 );
    }
    //-- Show what you got
    imshow( "Capture - flower detection", frame );
    return frame;
}
