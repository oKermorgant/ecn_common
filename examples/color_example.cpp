#include <ecn_common/color_detector.h>
#include <sstream>
#include <sensor_msgs/Image.h>
#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
//using ecn::ColorDetector;

cv::Mat im;
bool im_ok;

void readImage(const sensor_msgs::ImageConstPtr msg)
{
    im_ok = true;
    im = cv_bridge::toCvCopy(msg, "bgr8")->image;
}

int main(int argc, char** argv)
{
    // subscribe to images
    ros::init(argc, argv, "color_detector");

    im_ok = false;

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imsub = it.subscribe("/image", 1, &readImage);

    // init color detector
    int r = 255, g = 0, b = 0;
    if(argc == 4)
    {
        r = atoi(argv[1]);
        g = atoi(argv[2]);
        b = atoi(argv[3]);
    }
    ecn::ColorDetector cd(r, g, b);
    cd.showSegmentation();  // also gives trackbars for saturation / value
    cd.showOutput();
    cd.fitCircle();

    ros::Rate loop(10);

    while(ros::ok())
    {
        if(im_ok)
        {
            cd.process(im);
        }

        loop.sleep();
        ros::spinOnce();
    }
}
