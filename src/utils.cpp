#include "utils.h"

double correctAngle(double new_angle, double old_angle)
{
    double a = atan2(sin(old_angle), cos(old_angle));
    if (abs(new_angle - a) > CV_PI)
    {
        if (new_angle > 0 && a < 0)
            return old_angle - ((a + CV_PI*2) - new_angle);
        else if (new_angle < 0 && a > 0)
            return old_angle + ((new_angle + CV_PI*2) - a);
        else
            return old_angle + (new_angle - a);
    } else
        return old_angle + (new_angle - a);
}

Point2f computeIntersect(Vec4i a, Vec4i b)
{
    int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3];
    int x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];

    if (float d = ((float)(x1-x2) * (y3-y4)) - ((y1-y2) * (x3-x4)))
    {
        cv::Point2f pt;
        pt.x = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)) / d;
        pt.y = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)) / d;
        return pt;
    }
    else
        return Point2f(-1, -1);
}
