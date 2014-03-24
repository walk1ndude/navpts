#include "APDrone.h"
#include <iomanip>

/// ======================== Controller =========================

APDrone::APDrone() {
    clearTarget();
}

// helper function
void APDrone::i_term_increase(double& i_term, double new_err, double cap)
{
    if(new_err < 0 && i_term > 0)
        i_term = std::max(0.0, i_term + 2.5 * new_err);
    else if(new_err > 0 && i_term < 0)
        i_term = std::min(0.0, i_term + 2.5 * new_err);
    else
        i_term += new_err;

    if(i_term > cap) i_term =  cap;
    if(i_term < -cap) i_term =  -cap;
}

#define abs_trim(val, border) (min(max((val), -(border)), (border)))
#define sqr(x) ((x)*(x))
int APDrone::update()
{
    double ts = getTimestamp();
    if (_targetValid)
    {
        double dt = ts - last_ts;
        Pose p = pose(ts);
        Vec4d new_err = _target - p.vec(0); // TODO: correct yaw!!!
        Vec4d d_error = -p.vec(1);
        Matx22d r = p.rot2d().t(); // global to local - inv matrix

        // here was complete event

        Vec4d d_term, p_term;

        d_term[0] = r(0,0)*d_error[0] + r(0,1)*d_error[1];
        d_term[1] = r(1,0)*d_error[0] + r(1,1)*d_error[1]; // in tum CS - invert pitch (* (-1))
        d_term[2] = d_error[2];
        d_term[3] = d_error[3];

        p_term[0] = r(0,0)*new_err[0] + r(0,1)*new_err[1];
        p_term[1] = r(1,0)*new_err[0] + r(1,1)*new_err[1]; // in tum CS - invert pitch (* (-1))
        p_term[2] = new_err[2];
        p_term[3] = new_err[3];

        i_term_increase(i_term[0], new_err[0] * dt, 0.1 / (rp_k.i+(1e-10)));
        i_term_increase(i_term[1], new_err[1] * dt, 0.1 / (rp_k.i+(1e-10)));
        i_term_increase(i_term[2], new_err[2] * dt, 0.2 / (gaz_k.i+(1e-10)));
        i_term_increase(i_term[3], new_err[3] * dt, 0.1 / (yaw_k.i+(1e-10)));

        for (int i = 0; i < 4; i++)
            if (!reached_axe[i] && last_err[i] * new_err[i] < 0) // !!!!!!!!!! check time?
                reached_axe[i] = true, i_term[i] = 0;

        double cmd_pitch = rp_k.p  * p_term[0] + rp_k.d  * d_term[0] + rp_k.i * i_term[0]; //
        double cmd_roll  = rp_k.p  * p_term[1] + rp_k.d  * d_term[1] + rp_k.i * i_term[1]; // in tum CS - roll is x, pitch is (-y)
        double cmd_gaz   = gaz_k.p * p_term[2] + gaz_k.d * d_term[2] + gaz_k.i * i_term[2];
        double cmd_yaw   = yaw_k.p * p_term[3] + yaw_k.d * d_term[3] + yaw_k.i * i_term[3];

        VelocityControl cmd_vel(ts); // set to 0

        if (_flags & AutoXY)  cmd_vel.linear.x = abs_trim(cmd_pitch, params.trim_xy);
        if (_flags & AutoXY)  cmd_vel.linear.y = abs_trim(cmd_roll, params.trim_xy);
        if (_flags & AutoZ)   cmd_vel.linear.z = abs_trim(cmd_gaz, params.trim_z);
        if (_flags & AutoYaw) cmd_vel.angular.z = abs_trim(cmd_yaw, params.trim_yaw);

        cmdvel(cmd_vel);

        /*
        Vec4d cmd(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z, cmd_vel.angular.z);
        dbgStream() << fixed << setprecision(3)
          << "pos:" << p.vec(0)
          << "; vel:" << p.vec(1)
          << "; err:" << new_err
          << "; cmd:" << cmd << endl;
        */
    }
    last_ts = ts;
}
#undef sqr
#undef abs_trim

void APDrone::moveTo(const Vec4d & target, uint flags)
{
    //dbgStream() << std::setprecision(4) << "Set target to " << target << endl;

    _target = target;
    _targetValid = true;
    _flags = flags;
    last_ts = getTimestamp();

    reached_axe[0] = reached_axe[1] = reached_axe[2] = false;
    i_term[0] = i_term[1] = i_term[2] = i_term[3] = 0;
    //complete_ts = -1;
    //_complete = false;
}

void APDrone::moveTo(const Point3d & pos, double yaw)
{
    moveTo(Vec4d(pos.x, pos.y, pos.z, yaw), AutoAll);
}

void APDrone::moveTo(const Point3d & pos)
{
    moveTo(Vec4d(pos.x, pos.y, pos.z), AutoPos);
}

void APDrone::moveTo(const Point2d & pos2d, double z, double yaw)
{
    moveTo(Vec4d(pos2d.x, pos2d.y, z, yaw), AutoAll);
}

void APDrone::moveTo(const Point2d & pos2d, double z)
{
    moveTo(Vec4d(pos2d.x, pos2d.y, z), AutoPos);
}

void APDrone::moveTo(const Point2d & pos2d)
{
    moveTo(Vec4d(pos2d.x, pos2d.y), AutoXY);
}

void APDrone::moveBy(const Point2d & vec2d, double ts)
{
    Pose p = pose();
    moveTo(p.pos2d() + p.rot2d() * vec2d, p.z[0], p.yaw[0]);
}

void APDrone::clearTarget()
{
    _targetValid = false;
}
