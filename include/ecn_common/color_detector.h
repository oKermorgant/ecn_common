#ifndef COLORDETECTOR_H
#define COLORDETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>

namespace ecn
{


struct CamParam
{
    CamParam() {}
    // init from classical parameters
    CamParam(double _px, double _py, double _u0, double _v0):
        u0(_u0), v0(_v0), px(_px), py(_py), ipx(1./_px), ipy(1./_py)
    {}

    // init from resolution and view angle
    CamParam(int width, int height, double field_of_view)
    {
        const double t = tan(field_of_view*M_PI/360.);   // tan(fov/2) in rad
        u0 = width/2.;
        v0 = height/2.;
        ipx = t/u0;
        ipy = t/v0;
        px = 1./ipx;
        py = 1./ipy;
    }

    double ipx, ipy, u0, v0, px, py;
};

class ColorDetector
{
public:
    ColorDetector()
    {
        // default values
        setContourDisplay(255,255,255);
        setSaturationValue(130,95);

        setCamera(640,480,60);
    }

    ColorDetector(int r, int g, int b)
    {
        setCamera(640,480,60);
        setContourDisplay(255,255,255);
        setSaturationValue(130,95);
        detectColor(r, g, b);
    }

    // tuning
    void setSaturationValue(int sat, int value)
    {
        if(show_segment_)
        {
             cv::setTrackbarPos("Saturation", "Color detector - range", sat);
             cv::setTrackbarPos("Value", "Color detector - range", value);
        }
        sat_ = sat; val_ = value;
    }
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
        cam = CamParam(px, py, u0, v0);
        // default values for x,y
        x_ = cam.u0/2;
        y_ = cam.v0/2;
    }
    void setCamera(int width, int height, double field_of_view)
    {
        cam = CamParam(width, height, field_of_view);
    }
    inline double xLim() {return cam.u0*cam.ipx;}
    inline double yLim() {return cam.v0*cam.ipy;}

    // processing functions
    std::vector<cv::Point> findMainContour(const cv::Mat &_im);
    bool process(const cv::Mat &_im, cv::Mat &_im_processed, bool write_output = true);
    bool process(const cv::Mat &_im);

    // get resulting info
    inline double x() {return (x_-cam.u0)*cam.ipx;}
    inline double y() {return (y_-cam.v0)*cam.ipy;}
    inline double area() {return area_*cam.ipx*cam.ipy;}
    inline double x_p() {return x_;}
    inline double y_p() {return y_;}
    inline double area_p() {return area_;}
    CamParam cam;

protected:    
    double x_=0, y_=0, area_=0;

    std::vector<int> hue_;
    int sat_, val_;
    cv::Scalar ccolor;
    cv::Mat img_, seg1_, seg2_;
    bool show_segment_ = false, show_output_ = false, fit_circle_ = false;
};
}

#endif // COLORDETECTOR_H
