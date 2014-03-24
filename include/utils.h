#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>
#include <opencv2/core/core.hpp>
using namespace cv;
using namespace std;

double correctAngle(double new_angle, double old_angle);
Point2f computeIntersect(Vec4i a, Vec4i b);
template<typename T> double sign(const T & value) { return value > 0 ? 1 : -1; }
template<typename T> string to_string(const T & val)
{
    ostringstream strs;
    strs << val;
    return strs.str();
}

#endif // UTILS_H
