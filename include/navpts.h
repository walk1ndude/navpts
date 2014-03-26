#ifndef _NAVPTS_H
#define _NAVPTS_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <ardrone_autonomy/Navdata.h>
#include <fstream>
#include <deque>
#include "utils.h"

#include <visualization_msgs/Marker.h>

using namespace std;
using namespace cv;

/// =========================== Definitions ===========================

const bool SEARCH_MARKER = true;


#define GRAD (CV_PI/180.)

#define CAMERA_PARAMS_FILE "/home/ardrone/cameraParams.yml"
#define MAGNETO_PARAMS_FILE "magnetoParams.yaml"
#define RAW_VIDEO_FPS 25
#define VIEW_M2P 550.0 // 600
#define VIEW_P2M (1./VIEW_M2P)
#define VIEW_SCALE 0.1 // 0.03

// marker
#define MAX_DIST_TO_MARKER 5.0
#define MARKER_CONFIDENCE 3 // 3
#define MARKER_SIZE 1.5 // m
#define MARKER_SIZE_TOL 0.25 // %
#define MARKER_BORDER_LOW_THRESH 500
#define COLOR_DIFF_THRESH 20
#define ELLIPSE_MORPH_THRESH 0.98 // 0.95
#define ELLIPSE_MIN_PTS 40 // 20


// correct
#define CAM_HORIZON_CORR (4.5*GRAD) // 4.5
#define GYRO_X_CORRECT (0.0*GRAD) // flat trim must fix it

#define ROTZ_CORR_SPEED 0.05
#define GYRO_COEFF 1.75
#define VIDEO2NAVDATA_DELAY 0.07

// keep distances and angles
#define HEIGHT 0.8
#define POS_TOL 0.3
#define DY_TOL Vec2f(5*GRAD, 15*GRAD)
#define WALL_DIST 1.0
#define WALLS_SPEED_TOL 0.2


// sensors
#define WAHEAD 0
#define WLEFT 1
#define WBACK 2
#define WRIGHT 3
#define MAX_DIST 4.0
#define MAX_US_DELAY 0.5

// speed
#define SPEED_COEFF 0.3
#define ROT_SPEED 0.4
#define KE 0.5
#define KD 0.5

//#define YAW_CORRECT_CONST
//#define YAW_CORRECT_MAGNETO


/// =============================== PID =================================
class PID
{
private:
    const double kp, ki, kd;
    double last_err, sum_err;
    bool init;
public:
    PID(double _kp, double _ki, double _kd): kp(_kp), ki(_ki), kd(_kd), init(false) {}
    double control(double err, double dt)
    {
        sum_err += err*dt;
        double cnt = kp*err + ki*sum_err + (init ? kd*(err - last_err) : 0.0);
        last_err = err;
        init = true;
        return cnt;
    }
};

/// =========================== Navigator ===============================
class Navigator
{
public:
    explicit Navigator();

private:
    ros::NodeHandle handle;
    ros::Subscriber navdata_sub, vel_sub, serial_sub;
    ros::Publisher vel_pub, takeoff_pub, land_pub, reset_pub, flattrim_pub;
    image_transport::ImageTransport it;
    image_transport::Subscriber camera_image;
    cv_bridge::CvImagePtr cv_ptr;

    // frame
    Mat K, D;
    Mat frame;
    void loadCameraCalibration(const string & filename);

    // birds eye
    Size view_size;
    Mat view;
    Mat view_H;
    Mat getBirdsEyeTransform(double yaw, double pitch, double roll, double altd) const;
    Point2d image2drone(Point2d p, double scale);
    Point2d drone2view(Point2d p);
    static double color_diff(const Mat &src, const Mat &mask);
    void findMarker();

    // sensors
    bool navdata_init;
    ArdroneMagneto mag;
    ArdroneDROdometer odm;
    double rotZ_raw, rotZ_corr, d_rotZ_corr; // correct yaw
    double yaw, pitch, roll;
    Point2f speed;
    double altd; double ac;
    int drone_state;
    int battery;
    double drone_time, prev_time, takeoff_time, start_time;

    // image navdata sync
    deque<ardrone_autonomy::Navdata> navdataQueue;
    deque<cv_bridge::CvImagePtr> imageQueue;
    ardrone_autonomy::Navdata imgNavdata;
    ardrone_autonomy::Navdata navdataInterpolate(const ardrone_autonomy::Navdata & nd0, ardrone_autonomy::Navdata & nd1, ros::Time & middle_stamp);
    void tryImageIMSync();

    // walls
    double dist[4], prev_dist[4], new_dist[4], dist_time, prev_dist_time;
    bool dist_ok;
    int dist_cnt, dist_cnt2;

    // events
    void onNavdata(const ardrone_autonomy::Navdata & navdata);
    void onSerial(const std_msgs::String & str);
    void onImage(const sensor_msgs::ImageConstPtr & msg);

    // navigation
    enum {
        NS_MUSTINIT,
        NS_READY,
        NS_YAW_WALLS_ALIGN,
        NS_FLYBACK,
        NS_FLYSIDE,
        NS_FLYFRONT
    } nav_state; char * nav_state_str;
    int task2;

    Point2d marker_pos; bool marker_found;
    Point2d tmp_pos;
    int marker_found_times;
    double yaw0;
    double dist2;

    void userInput(geometry_msgs::Twist & cmdT);
    bool droneFlying() { return drone_state == 3 || drone_state == 4 || drone_state == 7; }
    bool droneTakingOff() { return drone_state == 6; }
    bool droneLanding() { return drone_state == 8; }
    bool droneIdle() { return drone_state == 2 || drone_state == 0 || drone_state == -1; }

    void keepHeight(geometry_msgs::Twist & cmdT, double min_height, double max_height);
    bool keepYaw(geometry_msgs::Twist & cmdT, double yaw_0, Vec2f tol = DY_TOL);
    bool keepMinDist(geometry_msgs::Twist & cmdT, int side, double min_dist, double error_dist);
    bool keepMaxDist(geometry_msgs::Twist & cmdT, int side, double max_dist, double error_dist);
    bool keepPos(geometry_msgs::Twist & cmdT, Point2f pos, double tol_dist, double err_dist);
    void flySide(geometry_msgs::Twist & cmdT, int side, double speed);

    // demo
    void demo();

public:
    void manual();
    void navigate();
};


#endif // NAVIGATION_NODE_H
