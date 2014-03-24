#include "odometer.h"

using namespace std;
using namespace cv;

void ArdroneDROdometer::update(const ardrone_autonomy::Navdata &navdata, double yaw)
{
    if (init)
    {
        double dt = navdata.header.stamp.toSec() - t;
        Point2d speed(navdata.vx*0.001, navdata.vy*0.001);
        p = drone2ground(speed * dt, yaw);
        t = navdata.header.stamp.toSec();
    } else {
        p = Point2d(0, 0);
        t = navdata.header.stamp.toSec();
        init = true;
    }
}

Point2d ArdroneDROdometer::drone2ground(Point2d pt, double yaw)
{
    return Point2d(pt.x*cos(yaw) - pt.y*sin(yaw), pt.x*sin(yaw) + pt.y*cos(yaw)) + pos();
}

Point2d ArdroneDROdometer::ground2drone(Point2d pt, double yaw)
{
    pt -= pos();
    return Point2d(pt.x*cos(-yaw) - pt.y*sin(-yaw), pt.x*sin(-yaw) + pt.y*cos(-yaw));
}
