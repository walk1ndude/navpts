#ifndef _BIRDSEYE_H
#define _BIRDSEYE_H

#include <opencv2/opencv.hpp>

cv::Mat getBirdsEyeTransform(const cv::Mat & K, double pitch, double roll, double altd, cv::Size view_size, double view_scale);

inline cv::Point2f RemapPoint(const cv::Mat & H, cv::Point2f pos)
{
#define _H ((const double*)H.data)
    float x = _H[0] * pos.x + _H[1] * pos.y + _H[2];
    float y = _H[3] * pos.x + _H[4] * pos.y + _H[5];
    float z = _H[6] * pos.x + _H[7] * pos.y + _H[8];
    return cv::Point2f(x / z, y / z);
#undef _H
}

#endif // _BIRDSEYE_H
