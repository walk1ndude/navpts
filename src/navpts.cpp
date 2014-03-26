#include "PVFilter.h"
#include "ardrone.h"
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"

#include <XmlRpcValue.h>

#include <geometry_msgs/PoseArray.h>

using namespace cv;
using namespace aruco;

#define FIX_SPOT_UPDATES 3

#define GET_POSES_INFO_TOPIC "get_poses_info"

#define SET_POSES_INFO_TOPIC "set_poses_info"

#define sqr(x) ((x)*(x))

class NPDrone : public ArDrone
{
private:
    class Spot
    {
    private:
        PFilter x, y, z;
        int _id = -1;
        double last_ts = -1;
        int upd_count = 0;

    public:
        Spot(int id): _id(id) {}
        Spot() {}

        inline void predict(double ts)
        {
            if (last_ts >= 0)
            {
                double dt = ts - last_ts;
                x.predict(dt, sqr(1));
                y.predict(dt, sqr(1));
                z.predict(dt, sqr(1));
            }
            last_ts = ts;
        }
        inline void update(const Point3d & pos, const Point3d & var)
        {
            x.observe(pos.x, var.x);
            y.observe(pos.y, var.y);
            z.observe(pos.z, var.z);
            upd_count++;
        }

        inline int id() { return _id; }
        inline Point3d pos() { return Point3d(x, y, z); }
        inline Point3d var() { return Point3d(x.var, y.var, z.var); }
        inline Point2d pos2d() { return Point2d(x, y); }
        inline Point2d var2d() { return Point2d(x.var, y.var); }

        inline bool fixed() { return FIX_SPOT_UPDATES >= 0 && upd_count >= FIX_SPOT_UPDATES; }
    };

    struct Params
    {
        //double arucoThresh[2] = {500, 1500};
        //MarkerDetector::ThresholdMethods arucoThreshMethod = MarkerDetector::CANNY;
        double arucoThresh[2] = {12, 12};
        MarkerDetector::ThresholdMethods arucoThreshMethod = MarkerDetector::ADPT_THRES;
        double markerSize = 0.12; // in meters
        double cubeSize = 0.16;
        double markerXYVarK = 30;
        double markerZVarK = 15;

        bool fixedSpots = true;

        int maxSearchAttempts = 8;
        double attemptYawTolerance = 5*grad;
    };

private:
    Params params;
    MarkerDetector MDetector;
    
    ros::NodeHandle paramHandle;

    ros::Publisher poses_pub;
    
    ros::Subscriber poses_sub;
    
    double yawRotateHeight;
    
    double hitTargetDist;
    double nearToTargetDist;
    
    double targetHeightHeading;
    
    double timeBetweenAttempts;
    
    Matx33d M;
    Matx15d D;
    Matx33d R;
    Matx31d T;

    vector<Spot> spots;
    vector<int> order;
    int searchAttempts = 0;
    bool operating = false;
    double missTs = -1;
    Spot * targetSpot = NULL;

    //enum { ST_TARGET, ST_SEARCH, ST_LAND } taskstate;

    Mat demo;
    Mat plan;

    Spot * getSpot(int id)
    {
        for (int i = 0; i < spots.size(); i++)
            if (spots[i].id() == id)
                return &spots[i];
        return NULL;
    }

    Spot * addSpot(int id)
    {
        spots.push_back(Spot(id));
        return &spots.back();
    }

    void getSpotPos(const Marker & marker, Vec3d & pos, Vec3d & var, double ts) // variance sucks (covariance needed)
    {
        Vec3d rvec = (Vec3d)marker.Rvec;
        Vec3d tvec = (Vec3d)marker.Tvec;
        Matx33d RCam;
        Rodrigues(rvec, RCam);

        double nCamLen = params.cubeSize/2;
        Vec3d nCam(0, 0, -nCamLen);
        Vec3d posCam = tvec + RCam * nCam;
        double _var = params.markerXYVarK * posCam(2) / ((M(0,0)+M(1,1))*0.5);
        Vec3d varCam(_var, _var, _var);
                    //sqr(params.markerXYVarK * posCam(2) / M(0,0)),
                    //sqr(params.markerXYVarK * posCam(2) / M(1,1)),
                    //sqr(params.markerZVarK * sqr(tvec(2)) / ((M(0,0)+M(1,1))*0.5 * params.markerSize);

        Pose p = pose(ts);

        //cout << R*Vec3d(posCam(1), -posCam(0), posCam(2)) << endl;
        pos = (Vec3d)(Mat)(p.rot() * (R*Vec3d(posCam(1), -posCam(0), posCam(2)) + T) + (Vec3d)p.pos());
        var = (Vec3d)(Mat)(p.rot() * (R*Vec3d(varCam(1), -varCam(0), varCam(2))));
        for (int i = 0; i < 3; i++) var(i) = abs(var(i));
    }

    virtual void onState(int st, double ts)
    {
        dbgStream() << "state = " << st << endl;
        if (st == Hovering)
        {
            operating = true;
        }
        else if (st == Landing)
        {
            operating = false;
        }
    }
    
    virtual void publishPoses(const std::vector<cv::Vec3d> & poses) {
      geometry_msgs::PoseArray poseArray;
      
      geometry_msgs::Pose pose;
      
      for (size_t i = 0; i != poses.size(); ++ i) {
	pose.position.x = poses[i][0];
	pose.position.y = poses[i][1];
	pose.position.z = poses[i][2];
	
	poseArray.poses.push_back(pose);
      }
      
      poseArray.header.stamp = ros::Time::now();
      
      poses_pub.publish<geometry_msgs::PoseArray>(poseArray);
    }
    
    
    void updatePoses(const geometry_msgs::PoseArray & posesInfo) {
      
    }

    virtual void onImage(const Mat & image, double ts, int camera)
    {
        // manage active camera calibration
      
      ROS_INFO("onImage");
        getCameraCalibration(M, D, R, T, camera);
        CameraParameters CP;
        CP.setParams(Mat(M), Mat(D), image.size());
        assert(CP.isValid());

        Mat tmp;
        image.copyTo(tmp);
        undistort(image, tmp, M, D);
        tmp.copyTo(demo);
        GaussianBlur(tmp, image, Size(3,3), 0);

        // detect markers
        vector<Marker> detectedMarkers;
        MDetector.setThresholdMethod(params.arucoThreshMethod);
        MDetector.setThresholdParams(params.arucoThresh[0], params.arucoThresh[1]);
        MDetector.setCornerRefinementMethod(MarkerDetector::SUBPIX);
        try {
            MDetector.detect(image, detectedMarkers, CP, params.markerSize, false);
        } catch (cv::Exception & e) {
            cout << e.what() << endl;
        }

        imshow("bin", MDetector.getThresholdedImage());
	
	std::vector<cv::Vec3d>poses;

        // update spots' positions
        for (size_t i = 0; i < detectedMarkers.size(); i++) {
            Marker &marker = detectedMarkers[i];

            // get and update spot from marker
            Vec3d spotPos, spotVar;
            getSpotPos(marker, spotPos, spotVar, ts);
            Spot * spot = getSpot(marker.id/100);
            if (!spot)
                spot = addSpot(marker.id/100);

            // update spot's ot self position
            if (!spot->fixed()) // spot fixes self position after several measurements
            {
                spot->update(spotPos, spotVar);
            }
            else // else update self position, if it had not been just added
            {
                Navdata::Position * selfPos = new Navdata::Position(ts);
                selfPos->pos = pose(ts).pos() + (spot->pos() - (Point3d)spotPos);
                selfPos->var = spotVar;
                pushEvent(PositionEvent, selfPos);
                // TODO: Angle event
            }
            
            poses.push_back(spotPos);

            // draw markers
            detectedMarkers[i].draw(demo, Scalar(0,0,255), 1);
            if (CP.isValid())
                CvDrawingUtils::draw3dAxis(demo, marker, CP);
        }
        
        publishPoses(poses);
        
        namedWindow("demo", 0);
        imshow("demo", demo);
    }

    void showPlan(double ts)
    {
        if (plan.empty())
            plan = Mat(600, 600, CV_8UC3);

        Pose p = pose(ts);

        plan.setTo(Scalar::all(255));

        Point2d cp(plan.cols/2, plan.rows/2);
        double scale = plan.cols/6.;
        Matx22d g2i(0, -scale, -scale, 0);

        // axes
        line(plan, Point(0, cp.y), Point(plan.cols, cp.y), Scalar::all(200));
        line(plan, Point(cp.x, 0), Point(cp.x, plan.rows), Scalar::all(200));

        // self position
        Point2d pos = cp + g2i*p.pos2d();
        line(plan, pos, pos + g2i*p.rot2d()*Point2d(1,0), Scalar(255,0,0));
        circle(plan, pos, 5, Scalar(255,0,0), -1);

        // spots
        for (int i = 0; i < spots.size(); i++)
        {
            Spot & spot = spots[i];
            Point2d sp = cp + g2i*Point2d(spot.pos().x, spot.pos().y);
            Point2d sv = g2i*Point2d(sqrt(spot.var().x), sqrt(spot.var().y));
            ellipse(plan, RotatedRect(sp, Size(abs(sv.x*2), abs(sv.y*2)), 0), Scalar(0,255,0));
            circle(plan, sp, 2, Scalar(0,0,255), -1);
            char str[32];
            sprintf(str, "%d", spot.id());
            putText(plan, str, sp, FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(0,0,255));
            //cout << "spot: " << spot.pos() << endl;
        }

        // target
        if (targetValid())
        {
            Point2d tp = cp + g2i*targetPos2d();
            line(plan, tp, tp + g2i*Point2d(cos(targetYaw()), sin(targetYaw()))*0.25, Scalar(0,0,255));
            circle(plan, tp, 5, Scalar(0,0,255), -1);
            putText(plan, "target", tp, FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(0,0,255));
        }

        imshow("plan", plan);
	waitKey(1);
    }

    void navigate(double ts)
    {
        int desiredCamera = CAM_FRONT;
        //int desiredCamera = CAM_BOTTOM;
        if (operating)
        {
            Pose p = pose(ts);
            if (!order.empty())
            {
                targetSpot = getSpot(order[0]);
                if (targetSpot)
                {
                    if (searchAttempts)
                        missTs = ts;
                    searchAttempts = 0;

                    double targetDist = norm(targetSpot->pos2d() - p.pos2d());
                    bool nearToTarget = targetDist < nearToTargetDist; // 1
                    bool hitTarget = targetDist < hitTargetDist; //0.2

                    Spot * nextSpot = order.size() >= 2 ? getSpot(order[1]) : NULL;
                    double headingYaw;
                    if (nearToTarget && nextSpot)
                    {
                        Point2d headingVec = nextSpot->pos2d() - targetSpot->pos2d();
                        headingYaw = atan2(headingVec.y, headingVec.x);
                    } else if (!nearToTarget && targetSpot)
                    {
                        Point2d headingVec = targetSpot->pos2d() - p.pos2d();
                        headingYaw = atan2(headingVec.y, headingVec.x);
                    } else
                        headingYaw = p.yaw[0];

                    if (nearToTarget)
                        desiredCamera = CAM_BOTTOM;

                    if (hitTarget)
                    {
                        if (ts - missTs > 1.2) // 1.5
                        {
                            dbgStream() << "Target #" << order[0] << "reached!" << endl;
                            order.erase(order.begin());
                            spots.clear(); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! clears all spots
                        }
                    }
                    else missTs = ts;
                    dbgStream() << "targetDist: " << targetDist << " (" << ts - missTs << " s)" << endl;

                    moveTo(targetSpot->pos2d(), targetHeightHeading, headingYaw); // 1.5 !!!

                }
                else // search
                {
		   
                    if (searchAttempts < params.maxSearchAttempts)
                    { 
                        if (searchAttempts == 0 || ts - missTs > timeBetweenAttempts) //0.8
                        {
                            dbgStream() << "Searching for spot #" << order[0] << "... (" << searchAttempts << ")" << endl;
                            moveTo(p.pos2d(), yawRotateHeight, p.yaw[0] + (searchAttempts > 0 ? CV_PI/5 : 0)); //0.6
                            if (searchAttempts == 0) missTs = ts;
                            searchAttempts++;
                        }
                        if (abs(target()[3] - p.yaw[0]) > params.attemptYawTolerance)
                        {
                            missTs = ts;
                            //desiredCamera = CAM_BOTTOM;
                        }
                    }
                    else // skip
                    {
                        dbgStream() << "Skip spot #" << order[0] << endl;
                        order.erase(order.begin());
                    }
                }
            }
            else
            {
                //land accurately
                desiredCamera = CAM_BOTTOM;
                if (targetSpot && p.z[0] - targetSpot->pos().z > 0.25) // - p.z0, 0.25
                    moveTo(targetSpot->pos()); // p.z0
                else
                    land();
            }
        }
        else
            missTs = ts;

        // switch camera if needed
        if (activeCamera() != desiredCamera)
            setActiveCamera(desiredCamera);
    }

    void checkControl()
    {
        // check control
        uchar c = waitKey(1);
        switch (tolower(c))
        {
        case 'l':
            land();
            break;
        case 't':
            takeoff();
            break;
        case 'r':
            reset();
            break;
        case 'f':
            flattrim();
            break;
        }
    }

    void predictSpotsPositions(double ts)
    {
        for (int i = 0; i < spots.size(); i++)
            if (!spots[i].fixed())
                spots[i].predict(ts);
    }
    
    void getFlightTask() {
      //XmlRpc::XmlRpcValue flightTask;
      
      paramHandle.param("yawRotateHeight", yawRotateHeight, 0.6);
      paramHandle.param("hitTargetDist", hitTargetDist, 0.2);
      paramHandle.param("nearToTargetDist", nearToTargetDist, 1.0);
      paramHandle.param("targetHeightHeading", targetHeightHeading, 1.35);
      paramHandle.param("timeBetweenAttempts", timeBetweenAttempts, 0.8);
      
     /* paramHandle.getParam("flightTask", flightTask);
      ROS_ASSERT(flightTask.getType() == XmlRpc::XmlRpcValue::TypeArray);
      */
      for (int i = 0; i < 3/*flightTask.size()*/; i ++) {
            order.push_back(i/*static_cast<int>(flightTask[i])*/);
      }
      
    }

public:
    NPDrone(): ArDrone("drone"), paramHandle("~")
    {
      getFlightTask();
      
      poses_pub = handle.advertise<geometry_msgs::PoseArray>(GET_POSES_INFO_TOPIC, 1);
      poses_sub = handle.subscribe(SET_POSES_INFO_TOPIC, 1, &NPDrone::updatePoses, this);
    }

    void update()
    {
        ArDrone::update();

        double ts = getTimestamp();
        //cout << pose(ts).rot()*Vec3d(0,0,1) << endl;
        predictSpotsPositions(ts);
      /*  navigate(ts);
       showPlan(ts);*/
        checkControl();
    }
};

/// ============== MAIN ======================
int main(int argc, char ** argv)
{    
    ros::init(argc, argv, "navpts");

    NPDrone drone;

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
//#ifndef _BAGPLAY
        drone.update();
//#endif
        loop_rate.sleep();
    }

    return 0;
}
