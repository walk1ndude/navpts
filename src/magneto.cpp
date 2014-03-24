#include "magneto.h"

using namespace std;
using namespace cv;

void ArdroneMagneto::initKF(double angle)
{
    KF.init(2,1,0);
    KF.transitionMatrix = *(cv::Mat_<float>(2,2) <<
           1,1,
           0,1);

    KF.statePre.at<float>(0) = angle;
    KF.statePre.at<float>(1) = 0;

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, cv::Scalar::all(1e+1));
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e+1));
    setIdentity(KF.errorCovPost, cv::Scalar::all(1e+6));
}

void ArdroneMagneto::loadCalibration(const string &filename)
{
    FileStorage fs(filename, FileStorage::READ);
    Mat mat;

    fs["offset"] >> mat;
    assert(!mat.empty());
    offset.x = mat.at<double>(0);
    offset.y = mat.at<double>(1);
    offset.z = mat.at<double>(2);

    fs["ratio"] >> mat;
    assert(!mat.empty());
    ratio.x = mat.at<double>(0);
    ratio.y = mat.at<double>(1);
    ratio.z = mat.at<double>(2);

    //fs["rotationMatrix"] >> rotationMatrix;
    //fs["orthogonalMatrix"] >> orthogonalMatrix;
}

void ArdroneMagneto::alignWithIMUData(Point3f & mag, const ardrone_autonomy::Navdata &navdata)
{
    double angleX = navdata.rotY * M_PI/180. * GYRO_COEFF;//atan2(accel.y,accel.z);
    double angleY = -navdata.rotX * M_PI/180. * GYRO_COEFF;//-atan2(accel.x,accel.z);

    double cosX = cos(angleX), sinX = sin(angleX);
    double cosY = cos(angleY), sinY = sin(angleY);

    Mat matX = (Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, cosX, -sinX, 0.0, sinX, cosX);
    Mat matY = (Mat_<double>(3, 3) << cosY, 0.0, sinY, 0.0, 1.0, 0.0, -sinY, 0.0, cosY);
    Mat matZ = (Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    Mat rot = matY *  matX * (Mat_<double>(3,1) << mag.x, mag.y, mag.z);

    mag.x = rot.at<double>(0,0);
    mag.y = rot.at<double>(0,1);
    mag.z = rot.at<double>(0,2);
}

void ArdroneMagneto::update(const ardrone_autonomy::Navdata & navdata)
{
    Point3f mag(
            (navdata.magX - offset.x) * ratio.x,
            (navdata.magY - offset.y) * ratio.y,
            (navdata.magZ - offset.z) * ratio.z
    );
    mag *= 1 / sqrt(mag.x * mag.x + mag.y * mag.y);
    alignWithIMUData(mag, navdata);
    double raw_angle = atan2(mag.y, mag.x);
    filtered_angle = raw_angle;
    if (KF.statePre.empty())
        initKF(raw_angle);
    float abs_angle = correctAngle(raw_angle, KF.predict().at<float>(0,0));
    filtered_angle = KF.correct(Mat(1, 1, CV_32F, &abs_angle)).at<float>(0, 0);
}
