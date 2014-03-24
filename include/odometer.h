#ifndef _ODOMETER_H
#define _ODOMETER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <ardrone_autonomy/Navdata.h>

class ArdroneDROdometer
{
private:
    cv::Point2d p;
    double t;
    bool init;
public:
    ArdroneDROdometer(): p(0,0), t(0), init(false) {};
    void update(const ardrone_autonomy::Navdata & navdata, double yaw);
    cv::Point2d drone2ground(cv::Point2d p, double yaw);
    cv::Point2d ground2drone(cv::Point2d p, double yaw);
    inline cv::Point2d pos(cv::Point2d offset = cv::Point2d(0, 0)) const { return p - offset; }
};

#endif
