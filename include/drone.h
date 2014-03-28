#ifndef _DRONE_H
#define _DRONE_H

//#define _BAGPLAY

#include <opencv2/opencv.hpp>

#include "PVFilter.h"
#include <limits>

#define grad (CV_PI/180.)
#define INF (numeric_limits<double>::infinity())

using namespace std;
using namespace cv;

double keepAngle(double new_angle, double old_angle);

// Drone
class Drone
{
public:

    struct Stamped {
        double ts;
        Stamped(double _ts): ts(_ts) {}
    };

    class Event
    {
    private:
        int _type;
        Ptr<Stamped> _obj;
    public:
        bool empty() { return _obj.empty(); }
        int type() const { return _type; }
        inline double ts() const { return _obj->ts; }
        template<typename T> inline const T & as() const { return *(const T*)(const Stamped*)_obj; }
        Event(int __type, Stamped * __obj): _type(__type), _obj(__obj) {}
    };

    class Navdata {
    public:
        class Acceleration : public Stamped, public Point3d {
            Acceleration(double ts) : Stamped(ts) {}
        };
        struct Velocity : public Stamped {
            Point2d dir, var; // varSpeedObservation_xy
            bool local;
            static double varAccelerationError_xy;
            Velocity(double ts) : Stamped(ts), var(INF, INF) {}
        };
        struct Height : Stamped {
            double z, var; // varPoseObservation_z
            static double varAccelerationError_z;
            Height(double ts) : Stamped(ts), var(INF) {}
        };
        struct Angle : Stamped
        {
            Vec3d rpy, var; // varPoseObservation_rp, varSpeedObservation_yaw
            bool imu;
            static double varSpeedError_rp, varAccelerationError_yaw;
            Angle(double ts) : Stamped(ts), var(INF, INF, INF) {}
        };
        struct Position : Stamped
        {
            Point3d pos;
            Point3d var;
            Position(double ts) : Stamped(ts), var(INF) {}
        };
        struct State : Stamped
        {
            int state;
            State(double ts) : Stamped(ts), state(Unknown) {}
        };
    };
    enum {NoEvent = 0x0000, ControlEvent = 0x0200, AccelerationEvent = 0x0100, VelocityEvent, HeightEvent, AngleEvent, PositionEvent, StateEvent};

    enum NavState {
        Unknown = 0x0,
        Inited = 0x1,
        Idle = 0x2,
        Landed = 0x3,
        Flying = 0x4,
        Hovering = 0x5,
        TakingOff = 0x6,
        Landing = 0x7
    };

    struct Pose : Stamped
    {
        PVFilter x, y, z;
        PFilter roll, pitch;
        PVFilter yaw;
        int state = Unknown;
        double z0 = 0; // ground level

        Pose(): Stamped(-1) {}
        inline Vec4d vec(int component) const { return Vec4d(x[component], y[component], z[component], yaw[component]); }
        inline Point3d pos() const { return Point3d(x[0], y[0], z[0]); }
        inline Point2d pos2d() const { return Point2d(x[0], y[0]); }
        inline Point3d vel() const { return Point3d(x[1], y[1], z[1]); }
        inline Point2d vel2d() const { return Point2d(x[1], y[1]); }
        //inline Matx22d rot2d() const { return Matx22d(sin(yaw[0]), cos(yaw[0]), cos(yaw[0]), -sin(yaw[0])); } // tum CS
        inline Matx22d rot2d() const { return Matx22d(cos(yaw[0]), -sin(yaw[0]), sin(yaw[0]), cos(yaw[0])); } // local to global
        Matx33d rot() const;
    };

    struct VelocityControl : Stamped // timestamp of apply
    {
        Point3d linear;
        Point3d angular;
        VelocityControl(double ts) : Stamped(ts) {}
        VelocityControl() : Stamped(-1), linear(0,0,0), angular(0,0,0) {}
    };

protected:
    string _name;
    virtual void onState(int st, double ts) {}
    virtual double getTimestamp() const { return 0; }
    virtual ostream & dbgStream(int level = 0);

public: 

    Drone() {}

    virtual Pose pose(double ts = -1) { return Pose(); }
    virtual int state(double ts = -1) { return pose(ts).state; }

    virtual bool controllable(double ts = -1);

    virtual double battery() { return 0; }
    virtual bool reset() { return false; }
    virtual bool flattrim() { return false; }
    virtual bool takeoff() { return false; }
    virtual bool land() { return false; }
    virtual bool hover() { return false; }
    virtual bool cmdvel(const VelocityControl & cmd_vel) { return false; }

};

#endif // DRONE_H
