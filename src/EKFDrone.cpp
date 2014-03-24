#include "EKFDrone.h"

/// ======================== PoseEKF ===========================

void EKFDrone::PoseEKF::observeVelocity(const Navdata::Velocity & v)
{
    Point2d _dir = v.local ? rot2d() * v.dir : v.dir;
    x.observeSpeed(_dir.x, v.var.x);
    y.observeSpeed(_dir.y, v.var.y);
}

void EKFDrone::PoseEKF::observeHeight(const Navdata::Height & h)
{
    //double dz_raw = h.z - z_raw;
    //if (abs(dz_raw) > 0.11) // TODO: check acceleration
    //    z0 -= dz_raw;
    z.observePose(h.z /*+ z0*/, h.var);
    z_raw = h.z;
}

void EKFDrone::PoseEKF::observeAngle(const Navdata::Angle & a)
{
    roll.observe(a.rpy[0], a.var[0]);
    pitch.observe(a.rpy[1], a.var[1]);
    yaw.observePose(a.rpy[2], a.var[2]);
}

void EKFDrone::PoseEKF::observePosition(const Navdata::Position & p)
{
    x.observePose(p.pos.x, p.var.x);
    y.observePose(p.pos.y, p.var.y);
    z.observePose(p.pos.z, p.var.z);
}

void EKFDrone::PoseEKF::predictTo(double ts, const VelocityControl* ctrl = NULL)
{
    if (ts <= this->ts)
        return;

    double dt = ts - this->ts;

    double vx_gain, vy_gain, vz_gain;
    double rollControlGain, pitchControlGain, yawSpeedControlGain; // control gains
    if (ctrl && false) // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    {
        const double
            c1 = 0.58,
            c2 = 17.8,
            c3 = 10,
            c4 = 35 * grad,
            c5 = 10,
            c6 = 25 * grad,
            c7 = 1.4,
            c8 = 1.0;

        rollControlGain = dt*c3*(c4 * max(-0.5, min(0.5, ctrl->linear.y)) + roll.state); // -roll.state ?
        pitchControlGain = dt*c3*(c4 * max(-0.5, min(0.5, ctrl->linear.x)) - pitch.state);
        yawSpeedControlGain = dt*c5*(c6 * ctrl->angular.z - yaw.state[1]);

        //double forceX = cos(yaw[0]) * sin(roll) * cos(pitch) - sin(yaw[0]) * sin(pitch);   // tum CS
        //double forceY = - sin(yaw[0]) * sin(roll) * cos(pitch) - cos(yaw[0]) * sin(pitch);

        Point2d forceLoc(sin(pitch), sin(-roll)*cos(pitch)); // ardrone CS
        Point2d forceGlob = rot2d() * forceLoc;

        vx_gain = dt * c1 * (c2*forceGlob.x - x.state[1]);
        vy_gain = dt * c1 * (c2*forceGlob.y - y.state[1]);
        vz_gain = dt * c7 * (c8*ctrl->linear.z*(ctrl->linear.z < 0 ? 2 : 1) - z.state[1]);
    }
    else
    {
        vx_gain = vy_gain = vz_gain = 0;
        rollControlGain = pitchControlGain = yawSpeedControlGain = 0;
    }

    roll.predict(dt, Navdata::Angle::varSpeedError_rp, rollControlGain);
    pitch.predict(dt, Navdata::Angle::varSpeedError_rp, pitchControlGain);
    yaw.predict(dt, Navdata::Angle::varAccelerationError_yaw, Vec2d(dt*yawSpeedControlGain/2, yawSpeedControlGain), 1, 5*5);

    x.predict(dt, Navdata::Velocity::varAccelerationError_xy, Vec2d(dt*vx_gain/2, vx_gain), 0.0001);
    y.predict(dt, Navdata::Velocity::varAccelerationError_xy, Vec2d(dt*vy_gain/2, vy_gain), 0.0001);
    z.predict(dt, Vec3d(dt*dt*dt*dt, 9*dt, dt*dt*dt*3), Vec2d(dt*vz_gain/2, vz_gain));

    this->ts = ts;
}

/// ====================== EKFDrone =======================

EKFDrone::EKFDrone()
{
    history.push_back(PoseEKF());
}

void EKFDrone::checkAges()
{
    if (!events.empty())
    {
        double thresh_ts = events.back().ts() - params.historyLength; // check age from the last event

        History::iterator history_begin = history.begin()+1;
        while (history_begin != history.end() && history_begin->ts < thresh_ts) ++history_begin;
        history.erase(history.begin()+1, history_begin);

        thresh_ts -= params.maxCtrlAge;
        Events::iterator events_begin = events.begin();
        while (events_begin != events.end() && events_begin->ts() < thresh_ts) ++events_begin;
        events.erase(events.begin(), events_begin);
    }
}

void EKFDrone::pushEvent(const Event & evt)
{
    // push event
    Events::reverse_iterator push_after = events.rbegin();
    while (push_after != events.rend() && push_after->ts() > evt.ts())
        ++push_after;
    bool predict = push_after == events.rbegin();
    events.insert(push_after.base(), evt);

    //truncate history
    History::reverse_iterator erase_after = history.rbegin();
    while (erase_after->ts >= evt.ts())
        ++erase_after;
    history.erase(erase_after.base(), history.end());

    // predict only if event is the newest
    // checkAges counts time from the newest event, so its still guaranted than prediction will take place before history is cleared
    if (predict)
        pose(evt.ts());        
}

const Drone::Event EKFDrone::eventBefore(double ts, int _type) const
{
    Events::const_reverse_iterator evt_it = events.rbegin();
    while (evt_it != events.rend() && (_type == NoEvent || evt_it->type() == _type) && evt_it->ts() > ts) ++evt_it;
    return evt_it != events.rend() ? (*evt_it) : Event(NoEvent, NULL);
}

Drone::Pose EKFDrone::pose(double ts)
{
    if (ts < 0)
        ts = getTimestamp();

    // find preceeding to timestamp (initial) pose
    // first history element has ts=-1
    History::iterator pose_it = history.end()-1;
    while (pose_it->ts > ts)
        --pose_it;
    PoseEKF pose = *pose_it;

    // if no events, nothing to calculate from
    if (events.size() > 0)
    {
        // find first event number after desired ts (en doesn't get < 0)
        Events::iterator evt_it = events.end();
        while (evt_it != events.begin() && (evt_it-1)->ts() > pose.ts)
            --evt_it;

        // initialize pose
        if (pose.ts < 0) pose.ts = evt_it->ts() - params.smallDt;

        // find first active control (first control before desired ts)
        Events::reverse_iterator ctrl_it(evt_it);
        while (ctrl_it != events.rend() && ctrl_it->ts() >= pose.ts - params.maxCtrlAge && ctrl_it->type() != ControlEvent)
            ++ctrl_it;
        const VelocityControl * active_ctrl = (ctrl_it != events.rend() ? &ctrl_it->as<VelocityControl>() : NULL);

        // track pose using prediction and observations
        while (pose.ts < ts)
        {
            Event * evt = evt_it != events.end() ? &(*evt_it) : NULL;

            double predictTo = pose.ts + params.smallDt;
            // check if any event is earlier that prediction time
            if (ts < predictTo) predictTo = ts;
            if (evt && evt->ts() <= predictTo) predictTo = evt->ts();

            // check if active velocity control is obsolete
            if (active_ctrl && active_ctrl->ts < pose.ts - params.maxCtrlAge)
                active_ctrl = NULL;
            // predict (pose.ts is set to predictTo here)
            pose.predictTo(predictTo, active_ctrl);

            // make observations if pending on timestamp
            if (evt && evt->ts() == predictTo)
            {
                switch(evt->type())
                {
                case VelocityEvent:
                    pose.observeVelocity(evt->as<Navdata::Velocity>());
                    break;
                case HeightEvent:
                    pose.observeHeight(evt->as<Navdata::Height>());
                    break;
                case AngleEvent:
                    pose.observeAngle(evt->as<Navdata::Angle>());
                    break;
                case PositionEvent:
                    pose.observePosition(evt->as<Navdata::Position>());
                    break;
                case ControlEvent:
                    active_ctrl = &evt->as<VelocityControl>();
                    break;
                case StateEvent:
                    pose.state = evt->as<Navdata::State>().state;
                }
                ++evt_it;
            }

            // append history
            if (pose.ts - history.back().ts > params.keypointInterval)
                history.push_back(pose);
        }
        checkAges();
    }
    return pose;
}

int EKFDrone::state(double ts) const
{
    if (ts < 0)
        ts = getTimestamp();

    History::const_iterator pose_it = history.end()-1;
    while (pose_it->ts > ts)
        --pose_it;
    return pose_it->state;
}
