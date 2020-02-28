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
    cout << "OpenCV version : " << CV_VERSION << endl;
      cout << "Major version : " << CV_MAJOR_VERSION << endl;
        cout << "Minor version : " << CV_MINOR_VERSION << endl;
          cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;
    return 0;
}
