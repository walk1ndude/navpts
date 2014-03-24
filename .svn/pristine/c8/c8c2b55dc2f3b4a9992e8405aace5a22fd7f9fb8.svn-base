#ifndef _SE_DRONE_H
#define _SE_DRONE_H

#include "drone.h"

class EKFDrone: public virtual Drone
{
public:

    class PoseEKF : public Pose
    {
    private:
        double z_raw = 0;
    public:
        PoseEKF(): Pose() {}
        void predictTo(double dt, const VelocityControl *ctrl);

        void observeVelocity(const Navdata::Velocity & v);
        void observeHeight(const Navdata::Height & h);
        void observeAngle(const Navdata::Angle & a);
        void observePosition(const Navdata::Position & p);
    };

private:
    struct
    {
        double historyLength = 1.0;
        double smallDt = 0.01;
        double maxObsAge = 0.1;
        double maxCtrlAge = 0.2;
        double keypointInterval = 0.05;
    } params;

    typedef vector<Event> Events;
    typedef vector<PoseEKF> History;
    History history;    // pose cache
    Events events;     // observations and velocity controls on timeline

    void checkAges();

protected:
    const Event eventBefore(double ts, int _type = NoEvent) const;
    void pushEvent(const Event & evt);
    void pushEvent(int type, Stamped * obj) {  pushEvent(Event(type, obj));  }

public:
    EKFDrone();

    Pose pose(double ts = -1);
    int state(double ts = -1) const;
};

#endif
