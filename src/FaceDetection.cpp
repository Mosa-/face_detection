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


using namespace std_msgs;
using namespace std;

ros::Subscriber camImage;
ros::Publisher faceROIPublisher;
ros::Publisher mouthROIPublisher;
CvHaarClassifierCascade *cascade_face = 0;
CvHaarClassifierCascade *cascade_nose = 0;
CvMemStorage *storage = 0;
const char* cascade_name_f = "src/face_detection/src/haarcascade/haarcascade_frontalface_alt.xml";
const char* cascade_name_n = "src/face_detection/src/haarcascade/haarcascade_mcs_nose.xml";

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv::Mat img;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	img = cv_ptr->image;
	IplImage iplImg = img;

	CvSeq *faces = cvHaarDetectObjects(
			&iplImg, cascade_face, storage,
			2.0, 3, 0, cvSize( 50,50 ) );

	sensor_msgs::RegionOfInterest faceROI;
	sensor_msgs::RegionOfInterest mouthROI;

	if (faces->total > 0){

		for (int face = 0; face < faces->total; ++face) {
			CvRect *e = (CvRect*)cvGetSeqElem(faces, face);
			cvRectangle(&iplImg,
						cvPoint(e->x, e->y),
						cvPoint(e->x + e->width, e->y + e->height),
						CV_RGB(255, 0, 0), 2, 8, 0);

			int mouthHeightDifference = e->height/3;
			int mouthWidthDifference = e->width/5;

			int mouthHeight = mouthHeightDifference;
			int mouthWidth = mouthWidthDifference * 3;

			int mouthC1X = e->x + mouthWidthDifference;
			int mouthC1Y = e->y + mouthHeightDifference*2;

			int mouthC2X = mouthC1X + mouthWidth;
			int mouthC2Y = mouthC1Y + mouthHeight;

			cvRectangle(&iplImg,
							cvPoint(mouthC1X, mouthC1Y),
							cvPoint(mouthC2X, mouthC2Y),
							CV_RGB(255, 0, 0), 2, 8, 0);

			faceROI.height = e->height;
			faceROI.width = e->width;
			faceROI.x_offset = e->x;
			faceROI.y_offset = e->y;

			mouthROI.height = mouthHeight;
			mouthROI.width = mouthWidth;
			mouthROI.x_offset = mouthC1X;
			mouthROI.y_offset = mouthC1Y;
			faceROIPublisher.publish(faceROI);
			mouthROIPublisher.publish(mouthROI);

		}

	}
	cvClearMemStorage(storage);

//	CvSeq *noses = cvHaarDetectObjects(
//			&iplImg, cascade_nose, storage,
//			2.0, 3, 0, cvSize( 25,15 ) );
//
//	if (noses->total > 0){
//
//		for (int nose = 0; nose < faces->total; ++nose) {
//			CvRect *e = (CvRect*)cvGetSeqElem(noses, nose);
//			cvRectangle(&iplImg,
//						cvPoint(e->x, e->y),
//						cvPoint(e->x + e->width, e->y + e->height),
//						CV_RGB(255, 0, 0), 2, 8, 0);
//
//		}
//	}



//		int fl, fw, ft, fh;
//
//		fl = e->x;
//		fw = e->width;
//		ft = e->y;
//		fh = e->height;
//
//		int ml, mw, mt, mh;

//		ml = fl + (fw - fl)/4;
//		mw = fw - (fw - fl)/4;
//		mt = ft + (2*(fh - ft))/3;
//		mh = fh - (fh - ft)/15;

//		ml = fl - (fw - fl)/4;
//		mw = fw - (fw + fl)/4;
//		mt = ft - (2*(fh - ft))/3;
//		mh = fh - (fh + ft)/5;
//
//		ROS_INFO("Face: Corner1: x=%d y=%d; Corner2: x=%d y=%d; faceHeight=%d, faceWidth=%d",fl, ft, fl+fw, ft+fh,fh,fw);
//		ROS_INFO("Mouth: Corner1: x=%d y=%d; Corner2: x=%d y=%d; moutheight=%d, mouthWidth=%d\n",ml, mt, ml+mw, mt+mh, mh, mw);
//
//
//		cvRectangle(&iplImg,
//						cvPoint(ml, mt),
//						cvPoint(ml + mw, mt + mh),
//						CV_RGB(255, 255, 255), 1, 8, 0);




	cvShowImage("face",&iplImg);
	cvClearMemStorage(storage);
	cv::waitKey(3);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "face_detection");

  ros::NodeHandle n;

  camImage = n.subscribe("/liprecKinect/rgb/image_raw", 10, imageCallback);
  faceROIPublisher = n.advertise<sensor_msgs::RegionOfInterest>("/face_detection/faceROI", 10);
  mouthROIPublisher = n.advertise<sensor_msgs::RegionOfInterest>("/face_detection/mouthROI", 10);

  cv::namedWindow("face");

  cascade_face = (CvHaarClassifierCascade*)cvLoad(cascade_name_f, 0, 0, 0);
  cascade_nose = (CvHaarClassifierCascade*)cvLoad(cascade_name_n, 0, 0, 0);

  storage = cvCreateMemStorage(0);

  ros::spin();

  return 0;
}
