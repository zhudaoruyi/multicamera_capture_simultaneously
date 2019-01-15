#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    cout << "Built with OpenCV " << CV_VERSION << endl;
    VideoCapture cap_0("nvcamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)3840, height=(int)2160, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink");
    VideoCapture cap_1("nvcamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=(int)3840, height=(int)2160, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink");
    VideoCapture cap_2("nvcamerasrc sensor-id=2 ! video/x-raw(memory:NVMM), width=(int)3840, height=(int)2160, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink");
    VideoCapture cap_3("nvcamerasrc sensor-id=3 ! video/x-raw(memory:NVMM), width=(int)3840, height=(int)2160, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink");
    if(cap_0.isOpened())
    {
        cout << "Capture is opened" << endl;
        for(;;)
        {
            Mat image_0;
            Mat image_1;
            Mat image_2;
            Mat image_3;
            cap_0 >> image_0;
            cap_1 >> image_1;
            cap_2 >> image_2;
            cap_3 >> image_3;

            imshow("camera video 0", image_0);
            imshow("camera video 1", image_1);
            imshow("camera video 2", image_2);
            imshow("camera video 3", image_3);
            if(waitKey(10) >= 0)
                break;
        }
    }
    else
    {
        cout << "No capture" << endl;
        cap_0.release();
        cap_1.release();
        cap_2.release();
        cap_3.release();
        destroyAllWindows();
    }
    return 0;
}


