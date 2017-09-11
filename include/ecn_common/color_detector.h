#ifndef COLORDETECTOR_H
#define COLORDETECTOR_H

#include <opencv2/core/core.hpp>

namespace ecn
{


struct CamParam
{
    // init from classical parameters
    CamParam(double _px, double _py, double _u0, double _v0):
        u0(_u0), v0(_v0), ipx(1./_px), ipy(1./_py) {}

    // init from resolution and view angle
    CamParam(int width, int height, double field_of_view)
    {
        const double t = tan(field_of_view*M_PI/90);
        u0 = width/2.;
        v0 = height/2.;
        ipx = t/u0;
        ipy = t/v0;
    }

    double ipx, ipy, u0, v0;
};

class ColorDetector
{
public:
    ColorDetector(): cam_(640,480,30)
    {
        // default values
        setContourDisplay(255,255,255);
        setSaturationValue(130,95);
    }

    ColorDetector(int r, int g, int b) : cam_(640,480,30)
    {
        setContourDisplay(255,255,255);
        setSaturationValue(130,95);
        detectColor(r, g, b);
    }

    // tuning
    void setSaturationValue(int sat, int value) {sat_ = sat; val_ = value;}
    void detectColor(int r, int g, int b);
    void showSegmentation();
    inline void showOutput() {show_output_ = true;}
    inline void fitCircle() {fit_circle_ = true;}
    inline void setContourDisplay(int r, int g, int b)
    {
        ccolor = cv::Scalar(b,g,r);
    }
    void setCamera(double px, double py, double u0, double v0)
    {
        cam_ = CamParam(px, py, u0, v0);
    }
    void setCamera(int width, int height, double field_of_view)
    {
        cam_ = CamParam(width, height, field_of_view);
    }

    // processing functions
    std::vector<cv::Point> findMainContour(const cv::Mat &_im);
    void process(const cv::Mat &_im, cv::Mat &_im_processed, bool write_output = true);
    void process(const cv::Mat &_im);

    // get resulting info
    inline double x() {return (x_-cam_.u0)*cam_.ipx;}
    inline double y() {return (y_-cam_.v0)*cam_.ipy;}
    inline double area() {return area_*cam_.ipx*cam_.ipy;}

protected:    
    double x_=0, y_=0, area_=0;
    CamParam cam_;
    std::vector<int> hue_;
    int sat_, val_;
    cv::Scalar ccolor;
    cv::Mat img_, seg1_, seg2_;
    bool show_segment_ = false, show_output_ = false, fit_circle_ = false;
};
}

#endif // COLORDETECTOR_H
