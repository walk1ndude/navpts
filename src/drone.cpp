#include "drone.h"
#include "stdio.h"

double Drone::Navdata::Angle::varAccelerationError_yaw;
double Drone::Navdata::Angle::varSpeedError_rp;
double Drone::Navdata::Velocity::varAccelerationError_xy;
double Drone::Navdata::Height::varAccelerationError_z;

// correct new angle to avoid angle jumps from 2pi to 0 (?)
double keepAngle(double new_angle, double old_angle)
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

ostream & Drone::dbgStream(int level)
{
    printf("\n[%.4f] ", getTimestamp());
    return cout;
}

bool Drone::controllable(double ts)
{
    int s = state(ts);
    return s == Hovering || s == Flying;
}

Matx33d Drone::Pose::rot() const {
    Matx33d RX(
        1,          0,           0,
        0, cos(roll), -sin(roll),
        0, sin(roll),  cos(roll)
    );
    Matx33d RY(
        cos(pitch), 0,  sin(pitch),
                0, 1,          0,
        -sin(pitch), 0, cos(pitch)
    );
    Matx33d RZ(
        cos(yaw[0]), -sin(yaw[0]), 0,
        sin(yaw[0]),  cos(yaw[0]), 0,
        0,          0,           1
    );
    return RX*RY*RZ;
}
