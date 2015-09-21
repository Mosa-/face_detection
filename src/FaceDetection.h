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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pthread.h>
#include <QThread>
#include <QTimer>

using namespace std_msgs;
using namespace std;
using namespace cv;

class FaceDetection : public QObject{
Q_OBJECT

public:

    FaceDetection(ros::NodeHandle* nh, QObject *parent = 0);

    void prepareFaceMouthROISender();
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

private:
    ros::NodeHandle* nh;
    ros::Publisher faceROIPublisher;
    ros::Publisher mouthROIPublisher;
    CvHaarClassifierCascade *cascade_face;
    CvHaarClassifierCascade *cascade_nose;
    CvMemStorage *storage;
    sensor_msgs::RegionOfInterest lastFaceROI;


    sensor_msgs::RegionOfInterest removeFaceROIShaking(sensor_msgs::RegionOfInterest &faceROI);
private slots:
    void sendFaceMouthROI();

};




#endif /* FACEDETECTION_H_ */
