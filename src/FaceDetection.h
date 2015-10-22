#ifndef FACEDETECTION_H_
#define FACEDETECTION_H_

#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32MultiArray.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pthread.h>
#include <QThread>
#include <QTimer>
#include <QDateTime>

using namespace std_msgs;
using namespace std;
using namespace cv;

class FaceDetection : public QObject{
Q_OBJECT

public:

    FaceDetection(ros::NodeHandle* nh, QObject *parent = 0);

    void prepareFaceDetection(std::string mouthROIMethod, double thresholdKeepFaceROI = 0.90,  bool useCam = true);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    void sendFaceMouthROI();
private:
    ros::NodeHandle* nh;
    ros::Publisher faceROIPublisher;
    ros::Publisher mouthROIPublisher;
    CvHaarClassifierCascade *cascade_face;
    CvMemStorage *storage;
    sensor_msgs::RegionOfInterest lastFaceROI;

    QString cascade_name_f;

    std::string mouthROIMethod;
    double thresholdKeepFaceROI;
    bool useCam;

    QTimer* noUseCamTimer;

    sensor_msgs::RegionOfInterest removeFaceROIShaking(sensor_msgs::RegionOfInterest &faceROI);
public slots:

signals:

};




#endif /* FACEDETECTION_H_ */
