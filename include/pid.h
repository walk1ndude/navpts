#ifndef PID_H
#define PID_H

class PID
{
private:
    const double kp, ki, kd;
    double last_err, sum_err;
    bool init;
public:
    inline PID(double _kp, double _ki, double _kd): kp(_kp), ki(_ki), kd(_kd), init(false) {}
    inline double control(double err, double dt)
    {
        sum_err += err*dt;
        double cnt = kp*err + ki*sum_err + (init ? kd*(err - last_err) : 0.0);
        last_err = err;
        init = true;
        return cnt;
    }
};

#endif // PID_H
