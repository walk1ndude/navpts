#ifndef _PVFILTER_H
#define _PVFILTER_H

#include "opencv2/opencv.hpp"

// dt in seconds

class PVFilter
{
public:
    cv::Vec2d state;
    cv::Matx22d var; // 2x2

    // constructors
    inline PVFilter(cv::Vec2d state, cv::Matx22d var)
        : state(state), var(var) {}

    inline PVFilter(double pose, double speed)
        : state(cv::Vec2d(pose,speed)), var(cv::Matx22d(0,0,0,0)) {}

    inline PVFilter(double pose)
        : state(cv::Vec2d(pose,0)), var(cv::Matx22d(0,0,0,1e10)) {}

    inline PVFilter()
        : state(cv::Vec2d(0,0)), var(cv::Matx22d(1e10,0,0,1e10)) {}

    inline PVFilter(const PVFilter & right)
        : state(right.state), var(right.var) {}

    inline const double & operator[](int i) const { return state[i]; }

    // observe
    inline void observePose(double obs, double obsVar)
    {
        /* MATLAB:
        H = [1 0];
        K = (uncertainty * H') / ((H * uncertainty * H') + obsVar);
        state = state + K * (obs - H*state);
        var = (eye(2)-K*H) * var;
        */

        cv::Vec2d K = cv::Vec2d(var(0,0), var(0,1)) * (1 / (obsVar + var(0,0)));	//K is first col = first row of var.
        state = state + K * (obs - state[0]);
        cv::Matx22d tmp(1-K[0], 0, -K[1], 1);
        var = tmp * var;
    }

    inline void observeSpeed(double obs, double obsVar)
    {
        /* MATLAB:
        H = [0 1];
        K = (uncertainty * H') / ((H * uncertainty * H') + obsVar);
        state = state + K * (observation - H*state);
        uncertainty = (eye(2)-K*H) * uncertainty;
        */

        cv::Vec2d K = cv::Vec2d(var(1,0), var(1,1)) * (1 / (obsVar + var(1,1)));	//K is second col = second row of var.
        state = state + K * (obs - state[1]);
        cv::Matx22d tmp(1, -K[0], 0, 1-K[1]);
        var = tmp * var;
    }

    // predict
    // calculates prediction variance matrix based on gaussian acceleration as error.
    inline void predict(double dt, double accelerationVar, cv::Vec2d controlGains = cv::Vec2d(0,0), double coVarFac = 1, double speedVarFac = 1)
    {
        /* MATLAB:
        G = [1 ms/1000; 0 1];
        E = [((ms/1000)^2)/2; ms/1000]; % assume normal distributed constant ACCELERATION.
        state = G*state;
        var = G * var * G' + accelerationVarPerS*(E*E');
        */

        cv::Matx22d G(1,dt,0,1);

        state = G * state + controlGains;
        var  = G * var * G.t();
        var(0,0) += accelerationVar * 0.25 * dt*dt*dt*dt;
        var(1,0) += coVarFac * accelerationVar * 0.5 * dt*dt*dt * 4;
        var(0,1) += coVarFac * accelerationVar * 0.5 * dt*dt*dt * 4;
        var(1,1) += speedVarFac * accelerationVar * 1 * dt*dt * 4 * 4;
    }

    // predict
    // calculates prediction using the given uncertainty matrix
    // vars is var(0) var(1) covar(0,1)
    inline void predict(double dt, cv::Vec3d vars, cv::Vec2d controlGains = cv::Vec2d(0,0))
    {
        /* MATLAB:
        G = [1 ms/1000; 0 1];
        E = [((ms/1000)^2)/2; ms/1000]; % assume normal distributed constant ACCELERATION.
        state = G*state;
        var = G * var * G' + accelerationVarPerS*(E*E');
        */

        cv::Matx22d G(1,dt,0,1);

        state = G * state + controlGains;
        var  = G * var * G.t();
        var(0,0) += vars[0];
        var(1,0) += vars[2];
        var(0,1) += vars[2];
        var(1,1) += vars[1];
    }
};


// KalmanFilter with only one component (pose, is observed directly)
class PFilter
{
public:
    double state;
    double var;

    inline PFilter()
        : state(0), var(1e10) {}

    inline PFilter(double initState)
        : state(initState), var(0) {}

    inline PFilter(const PFilter & right)
        : state(right.state), var(right.var) {}

    inline operator double() const { return state; }

    inline void predict(double dt, double speedVar, double controlGains = 0)
    {
        /* MATLAB:
        state = state;
        var = var + speedVar*((ms/1000)^2);
        */
        state += controlGains;
        var += speedVar * dt * dt;
    }

    inline void observe(double obs, double obsVar)
    {
        /* MATLAB:
        obs_w = var / (var + obsVar);
        state = state * (1-obs_w) + obs * obs_w;
        var = var*obsVar / (var + obsVar);
        */
        double w = var / (var + obsVar);
        state = (1-w) * state + w * obs;
        var = var * obsVar / (var + obsVar);
    }
};

#endif // _PVFILTER_H
