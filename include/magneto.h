#ifndef _MAGNETO_H
#define _MAGNETO_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <ardrone_autonomy/Navdata.h>

class ArdroneMagneto
{
private:
    double filtered_angle;
    cv::Point3f offset, ratio;
    //Mat rotationMatrix, orthogonalMatrix;
    cv::KalmanFilter KF;
public:
    void initKF(double angle);
    void loadCalibration(const std::string & filename);
    void alignWithIMUData(Point3f & mag, const ardrone_autonomy::Navdata & navdata);
    void update(const ardrone_autonomy::Navdata & navdata);
    double angle() { return filtered_angle; }
}; 

#endif