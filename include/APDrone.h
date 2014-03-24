#ifndef _AP_DRONE
#define _AP_DRONE

#include "drone.h"

class APDrone: public virtual Drone
{
private:

    struct
    {
        double trim_xy = 0.6; // 0.4
        double trim_z = 0.7;
        double trim_yaw = 1.2; //1.0
    } params;

    struct {  double i, d, p;  }
           rp_k =  {0, 0.35, 0.5},
           gaz_k = {0.001, 0.1, 0.6},
           yaw_k = {0.000, 0.1, 0.6}; // 0, 0, 0.05

    bool _targetValid;
    uint _flags;
    Vec4d _target;
    Vec2d _tolerance;

    Vec4d last_err;
    Vec4d i_term;
    bool reached_axe[4];
    double last_ts;

    static void i_term_increase(double& i_term, double new_err, double cap);

public:

    APDrone();

    inline bool targetValid() { return _targetValid; }
    inline Vec4d target() { return _target; }
    inline Point3d targetPos() { return Point3d(_target[0], _target[1], _target[2]); }
    inline Point2d targetPos2d() { return Point2d(_target[0], _target[1]); }
    inline double targetYaw() { return _target[3]; }

    void clearTarget();
    void moveTo(const Vec4d & target, uint flags = AutoXY | AutoZ | AutoYaw);
    void moveTo(const Point3d & pos, double yaw);
    void moveTo(const Point3d & pos);
    void moveTo(const Point2d & pos2d, double z, double yaw);
    void moveTo(const Point2d & pos2d, double z);
    void moveTo(const Point2d & pos2d);
    void moveBy(const Point2d & vec2d, double ts = -1);

    enum {
        AutoXY = 0x1,
        AutoZ = 0x2,
        AutoYaw = 0x4,
        AutoPos = AutoXY | AutoZ,
        AutoAll = AutoPos | AutoYaw
    };

    int update();
};

/*
        rp_k.i = 0;
        rp_k.d = 0.35;
        rp_k.p = 0.5;

        gaz_k.i = 0.001;
        gaz_k.d = 0.1;
        gaz_k.p = 0.6;

        yaw_k.i = 0;
        yaw_k.d = 0;
        yaw_k.p = 0.05;
*/

#endif
