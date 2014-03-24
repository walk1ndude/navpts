/// =========================== Navigator ===============================
class Navigator
{
public:
    explicit Navigator();

private:
    ros::NodeHandle handle;
    ros::NodeHandle handleParams("~");
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