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
CvHaarClassifierCascade *cascade_face;
CvMemStorage			*storage;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv::Mat img;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	img = cv_ptr->image;
	IplImage iplImg = img;

	CvSeq *faces = cvHaarDetectObjects(  //using the face classifier, first detect the face.
			&iplImg, cascade_face, storage,
			2.0, 3, 0, cvSize( 50,50 ) );

	//return if not found
	if (faces->total > 0){

		// draw a rectangle on each face
		CvRect *e = (CvRect*)cvGetSeqElem(faces, 0);
		cvRectangle(&iplImg,
					cvPoint(e->x, e->y),
					cvPoint(e->x + e->width, e->y + e->height),
					CV_RGB(255, 0, 0), 2, 8, 0);

		int ml, mw, mt, mh;

		ml = e->x + (e->width - e->x)/4;
		mw = e->width - (e->width - e->x) /4;
		mt = e->y + (2*(e->height-e->y))/3;
		mh = e->height - (e->height - e->y) / 15;

		cvRectangle(&iplImg,
						cvPoint(ml, mt),
						cvPoint(ml + mw, mt + mh),
						CV_RGB(255, 255, 255), 1, 8, 0);

	}


	cvShowImage("face",&iplImg);
	cvClearMemStorage(storage);
	cv::waitKey(3);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "face_detection");


  ros::NodeHandle n;

  camImage = n.subscribe("/liprecKinect/rgb/image_raw", 10, imageCallback);
  cv::namedWindow("face");
  cascade_face = (CvHaarClassifierCascade*)cvLoad("/home/felix/catkin_ws/src/face_detection/src/haarcascade/haarcascade_frontalface_alt2.xml", 0, 0, 0);
  storage = cvCreateMemStorage(0);


  ros::spin();

  return 0;
}
