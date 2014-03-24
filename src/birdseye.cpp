#include "birdseye.h"

cv::Mat getBirdsEyeTransform(const cv::Mat & K, double pitch, double roll, double altd, cv::Size view_size, double view_scale)
{
    assert(!K.empty());

    double alpha = pitch - CV_PI/2;
    double beta = -roll;
    double gamma = 0;

    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    double dist = altd * fy * view_scale;

    // Projection 2D -> 3D matrix
    cv::Mat A1 = (cv::Mat_<double>(4,3) <<
        1, 0, -view_size.width*0.5, // -cx
        0, 1, -view_size.height, // -6404
        0, 0,   0,
        0, 0,   1); // dist2

    // Rotation matrices around the X,Y,Z axis
    cv::Mat RX = (cv::Mat_<double>(4, 4) <<
        1,          0,           0, 0,
        0, cos(alpha), -sin(alpha), 0,
        0, sin(alpha),  cos(alpha), 0,
        0,          0,           0, 1);

    cv::Mat RY = (cv::Mat_<double>(4, 4) <<
        cos(beta), 0, -sin(beta), 0,
                0, 1,          0, 0,
        sin(beta), 0,  cos(beta), 0,
                0, 0,          0, 1);

    cv::Mat RZ = (cv::Mat_<double>(4, 4) <<
        cos(gamma), -sin(gamma), 0, 0,
        sin(gamma),  cos(gamma), 0, 0,
        0,          0,           1, 0,
        0,          0,           0, 1);

    // Composed rotation matrix with (RX,RY,RZ)
    cv::Mat R = RX * RY * RZ;

    // Translation matrix on the Z axis change dist will change the height
    cv::Mat T = (cv::Mat_<double>(4, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, dist,
        0, 0, 0, 1);

    // Camera Intrisecs matrix 3D -> 2D
    cv::Mat A2 = (cv::Mat_<double>(3,4) <<
        fx, 0, cx, 0,
        0, fy, cy, 0,
        0, 0,   1, 0);

    // Finally
    cv::Mat H = A2 * (R * (T * A1)); // T * R maybe
    return H;
}
