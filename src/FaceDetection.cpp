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
using namespace cv;

ros::Subscriber camImage;
ros::Publisher faceROIPublisher;
ros::Publisher mouthROIPublisher;
CvHaarClassifierCascade *cascade_face = 0;
CvHaarClassifierCascade *cascade_nose = 0;
CvMemStorage *storage = 0;
const char* cascade_name_f = "src/face_detection/src/haarcascade/haarcascade_frontalface_default.xml";
//const char* cascade_name_f = "src/face_detection/src/haarcascade/haarcascade_frontalface_alt.xml";
//const char* cascade_name_f = "src/face_detection/src/haarcascade/haarcascade_frontalface_alt2.xml";
//const char* cascade_name_f = "src/face_detection/src/haarcascade/haarcascade_frontalface_alt_tree.xml";
//const char* cascade_name_f = "src/face_detection/src/haarcascade/haarcascade_profileface.xml";

std::string mouthROIMethod;
sensor_msgs::RegionOfInterest lastFaceROI;
double thresholdKeepFaceROI = 0.95;


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
	
	sensor_msgs::RegionOfInterest intersectROI;
	bool keepFaceROI = false;


	if (faces->total > 0){

		for (int face = 0; face < faces->total; ++face) {
			CvRect *e = (CvRect*)cvGetSeqElem(faces, face);

			faceROI.height = e->height;
			faceROI.width = e->width;
			faceROI.x_offset = e->x;
			faceROI.y_offset = e->y;

			if(lastFaceROI.height == 0){
				lastFaceROI = faceROI;
			}else{
				//removeFaceROIShaking();
														
				//overlapRoi gucken
				int xMax = max(faceROI.x_offset, lastFaceROI.x_offset);
				int yMax = max(faceROI.y_offset, lastFaceROI.y_offset);
				int widthMin = min(faceROI.x_offset + faceROI.width, lastFaceROI.x_offset + lastFaceROI.width);
				int heightMin = min(faceROI.y_offset + faceROI.height, lastFaceROI.y_offset + lastFaceROI.height); 
				
				if(xMax < widthMin && yMax < heightMin){
					intersectROI.x_offset = xMax;
					intersectROI.y_offset = yMax;
					intersectROI.width = widthMin - xMax;
					intersectROI.height = heightMin - yMax;
					
					int areaIntersect = intersectROI.width * intersectROI.height;
					int areaTotal = faceROI.width * faceROI.height;
					
					double covers = areaIntersect / (double) areaTotal;
					
					if(covers > thresholdKeepFaceROI){
						faceROI = lastFaceROI;
					}
					
					lastFaceROI = faceROI;
				}
								
				// if(lastFaceROI.x_offset < faceROI.x_offset && lastFaceROI.y_offset < faceROI.y_offset){

					// Point P1(lastFaceROI.x_offset + lastFaceROI.width, lastFaceROI.y_offset+lastFaceROI.height);
					// Point P2(faceROI.x_offset, faceROI.y_offset);

				// }else if(faceROI.x_offset < lastFaceROI.x_offset && faceROI.y_offset < lastFaceROI.y_offset){

					// Point P1(faceROI.x_offset + faceROI.width, faceROI.y_offset+faceROI.height);
					// Point P2(lastFaceROI.x_offset, lastFaceROI.y_offset);

				// }else if(faceROI.x_offset < lastFaceROI.x_offset && faceROI.y_offset > lastFaceROI.y_offset){

					// Point P1(faceROI.x_offset + faceROI.width, faceROI.y_offset);
					// Point P2(lastFaceROI.x_offset, lastFaceROI.y_offset+lastFaceROI.height);

				// }else if(lastFaceROI.x_offset < faceROI.x_offset && lastFaceROI.y_offset > faceROI.y_offset){

					// Point P1(lastFaceROI.x_offset + lastFaceROI.width, lastFaceROI.y_offset);
					// Point P2(faceROI.x_offset, faceROI.y_offset + faceROI.height);
				// }
				// int intersectWidth = P1.x - P2.x;
				// int intersectHeight = P1.y - P2.y;
				// int aIntersect = intersectWidth * intersectHeight;
				// int aTotal = faceROI.width * faceROI.height;

				// double covers = aIntersect / aTotal;

			}



			if(mouthROIMethod.compare("one") == 0){
				int mouthHeightDifference = e->height/3;
				int mouthWidthDifference = e->width/5;

				int mouthHeight = mouthHeightDifference;
				int mouthWidth = mouthWidthDifference * 3;

				int mouthC1X = e->x + mouthWidthDifference;
				int mouthC1Y = e->y + mouthHeightDifference*2;

				mouthROI.height = mouthHeight;
				mouthROI.width = mouthWidth;
				mouthROI.x_offset = mouthC1X;
				mouthROI.y_offset = mouthC1Y;

			}else if(mouthROIMethod.compare("two") == 0){
				int fl, fw, ft, fh;
				fl = e->x;
				fw = e->x + e->width;
				ft = e->y;
				fh = e->y+e->height;

				int ml, mw, mt, mh;
				ml = fl + (fw - fl)/4;
				mw = fw - (fw - fl)/4;
				mt = ft + (fh - ft)/1.5;
				mh = fh - (fh - ft)/15;

				mouthROI.height = mh-mt;
				mouthROI.width = mw-ml;
				mouthROI.x_offset = ml;
				mouthROI.y_offset = mt;
			}else{
				int mouthHeightDifference = e->height/3;
				int mouthHeight = mouthHeightDifference;
				int mouthC1Y = e->y + mouthHeightDifference*2;
				int mouthC2Y = mouthC1Y + mouthHeight;

				int fl, fw;
				fl = e->x;
				fw = e->x + e->width;

				int ml, mw;
				ml = fl + (fw - fl)/4;
				mw = fw - (fw - fl)/4;

				mouthROI.height = mouthHeight;
				mouthROI.width = mw-ml;
				mouthROI.x_offset = ml;
				mouthROI.y_offset = mouthC1Y;
			}

			faceROIPublisher.publish(faceROI);
			mouthROIPublisher.publish(mouthROI);
		}

	}
	cvClearMemStorage(storage);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "face_detection");

  ros::NodeHandle n("~");

  camImage = n.subscribe("/liprecKinect/rgb/image_raw", 10, imageCallback);
  faceROIPublisher = n.advertise<sensor_msgs::RegionOfInterest>("/face_detection/faceROI", 10);
  mouthROIPublisher = n.advertise<sensor_msgs::RegionOfInterest>("/face_detection/mouthROI", 10);

  n.getParam("mouthROI", mouthROIMethod);

  cascade_face = (CvHaarClassifierCascade*)cvLoad(cascade_name_f, 0, 0, 0);

  storage = cvCreateMemStorage(0);

  ros::spin();

  return 0;
}
