#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <sensor_msgs/Image.h>
#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

using namespace cv;
cv::Mat src;
bool im_ok;

void readImage(const sensor_msgs::ImageConstPtr msg)
{
    im_ok = true;
    src = cv_bridge::toCvCopy(msg, "bgr8")->image;
}


/**
 * @function main
 */
int main( int argc, char** argv )
{
    ros::init(argc, argv, "color_detector");

    im_ok = false;

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imsub = it.subscribe("/image", 1, &readImage);

    // build inRange bounds for hue
    vector<int> hue = {0,10, 170,179};

    cv::Mat dst, mask, mask2;

    ros::Rate loop(10);

    while(ros::ok())
    {
        if(im_ok)
        {
            cv::cvtColor(src, src, cv::COLOR_BGR2HSV);
            // segment for detection of given RGB (from Hue)
            cv::inRange(src, cv::Scalar(hue[0], 0,0),
                    cv::Scalar(hue[1], 255, 255), mask);
            cv::inRange(src, cv::Scalar(hue[2], 0, 0),
                    cv::Scalar(hue[3], 255, 255), mask2);
            mask += mask2;

            imshow("Mask", mask);

            /// Separate the image in 3 places ( B, G and R )
            vector<Mat> bgr_planes;
            split( src, bgr_planes );

            /// Establish the number of bins
            int histSize = 256;

            /// Set the ranges ( for B,G,R) )
            float range[] = { 0, 256 } ;
            const float* histRange = { range };

            bool uniform = true; bool accumulate = false;

            Mat b_hist, g_hist, r_hist;

            /// Compute the histograms:
            calcHist( &bgr_planes[1], 1, 0, mask, g_hist, 1, &histSize, &histRange, uniform, accumulate );
            calcHist( &bgr_planes[2], 1, 0, mask, r_hist, 1, &histSize, &histRange, uniform, accumulate );

            // Draw the histograms for B, G and R
            int hist_w = 512; int hist_h = 400;
            int bin_w = cvRound( (double) hist_w/histSize );

            Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

            /// Normalize the result to [ 0, histImage.rows ]
            normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
            normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

            /// Draw for each channel
            for( int i = 1; i < histSize; i++ )
            {
                line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                      Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                      Scalar( 0, 255, 0), 2, 8, 0  );
                line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                      Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                      Scalar( 0, 0, 255), 2, 8, 0  );
            }

            /// Display
            namedWindow("calcHist Demo", cv::CV_WINDOW_AUTOSIZE );
            imshow("calcHist Demo", histImage );

            waitKey(1);

        }

        loop.sleep();
        ros::spinOnce();
    }


    return 0;
}
