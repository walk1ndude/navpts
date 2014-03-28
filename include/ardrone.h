#ifndef _ARDRONE_H
#define _ARDRONE_H

#include "EKFDrone.h"
#include "APDrone.h"

//#define _BAGPLAY

#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

typedef Matx<double, 1, 5> Matx15d;

#define CAM_SWITCH_DELAY 0.4
class ArDrone : public EKFDrone, public APDrone
{
protected:
    ros::NodeHandle handle;
    ros::Subscriber s_navdata, s_vel2;
    ros::Publisher p_vel, p_vel2, p_takeoff, p_land, p_reset;
    ros::ServiceClient srv_togglecamera, srv_flattrim;
    image_transport::ImageTransport it;
    image_transport::Subscriber s_camera_front, s_camera_bottom;

    //vector<ardrone_autonomy::Navdata> nds;
    //void _onNavdata2(const ardrone_autonomy::Navdata & navdata);

    double batteryPercent = 0;
    int64 navdata_seq;
    int camIndex = -1, desiredCamIndex = -1;
    double camSwitchTs = -1;

    int lastState = -1;
    double yaw0, last_yaw = 0, last_pitch = 0, last_roll = 0;
    double last_ts = -1;
    int last_altd = -1;
    int last_altd_seq = -9999;
    
    double timeCorr = 0;
    std::string vid_topic;

    //double must_land;
    //double must_takeoff;

    void _onNavdata(const ardrone_autonomy::Navdata & navdata);
    void _onCmdVel(const geometry_msgs::TwistStamped & cmd_vel);
    void _onImageFront(const sensor_msgs::ImageConstPtr & msg);
    void _onImageBottom(const sensor_msgs::ImageConstPtr & msg);

protected:
    static const int smap[9]; // status mapping from ardrone to abstract drone

protected:
    // for successors
    enum { CAM_FRONT = 0, CAM_BOTTOM = 1 };
    virtual void onNavdata(const ardrone_autonomy::Navdata & navdata) {}
    virtual void onImage(const Mat & image, double ts, int camera) {}

    // implement functions declared in Drone
    virtual double getTimestamp() const;
    virtual double battery();
    virtual bool reset();
    virtual bool flattrim();
    virtual bool takeoff();
    virtual bool land();
    virtual bool hover();
    virtual bool cmdvel(const VelocityControl & cmd_vel);

public:
    ArDrone(const string & name);
    void update();
    inline int activeCamera() { return camIndex; }
    bool setActiveCamera(int index);
    bool getCameraCalibration(Matx33d &M, Matx15d &D, Matx33d &R, Matx31d &T, int camIndex = -1);
};

#endif

