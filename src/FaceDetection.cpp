#include "FaceDetection.h"


FaceDetection::FaceDetection(ros::NodeHandle *nh, QObject *parent): QObject(parent)
{

    cascade_name_f = "src/face_detection/src/haarcascade/haarcascade_frontalface_default.xml";
    //cascade_name_f = "src/face_detection/src/haarcascade/haarcascade_frontalface_alt.xml";
    //cascade_name_f = "src/face_detection/src/haarcascade/haarcascade_frontalface_alt2.xml";
    //cascade_name_f = "src/face_detection/src/haarcascade/haarcascade_frontalface_alt_tree.xml";
    //cascade_name_f = "src/face_detection/src/haarcascade/haarcascade_profileface.xml";


    this->nh = nh;
    faceROIPublisher = nh->advertise<sensor_msgs::RegionOfInterest>("/face_detection/faceROI", 10);
    mouthROIPublisher = nh->advertise<sensor_msgs::RegionOfInterest>("/face_detection/mouthROI", 10);
    cascade_face = NULL;
    storage = NULL;
}

void FaceDetection::prepareFaceDetection(string mouthROIMethod, double thresholdKeepFaceROI, bool useCam)
{
    this->mouthROIMethod = mouthROIMethod;
    this->thresholdKeepFaceROI = thresholdKeepFaceROI;
    this->useCam = useCam;

    if(useCam){
        ROS_INFO("TODES");
        storage = cvCreateMemStorage(0);
        cascade_face = (CvHaarClassifierCascade*)cvLoad(cascade_name_f.toStdString().c_str(), 0, 0, 0);
    }
}

void FaceDetection::sendFaceMouthROI()
{
    int i = 0;
    qint64 timestamp = QDateTime::currentMSecsSinceEpoch();
    while(true){
        // hier schicken
        while(QDateTime::currentMSecsSinceEpoch() < timestamp+3000){
        }
        timestamp = QDateTime::currentMSecsSinceEpoch();
    }
}

sensor_msgs::RegionOfInterest FaceDetection::removeFaceROIShaking(sensor_msgs::RegionOfInterest& faceROI){
	sensor_msgs::RegionOfInterest intersectROI;
	sensor_msgs::RegionOfInterest currentFaceROI = faceROI;

	if(lastFaceROI.height == 0){
		lastFaceROI = faceROI;
	}else{
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
			}else{
				lastFaceROI = currentFaceROI;
			}
		}
	}
	return faceROI;
}

void FaceDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv::Mat img;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
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

			faceROI.height = e->height;
			faceROI.width = e->width;
			faceROI.x_offset = e->x;
			faceROI.y_offset = e->y;

			faceROI = removeFaceROIShaking(faceROI);

			if(mouthROIMethod.compare("one") == 0){
				int mouthHeightDifference = faceROI.height/3;
				int mouthWidthDifference = faceROI.width/5;

				int mouthHeight = mouthHeightDifference;
				int mouthWidth = mouthWidthDifference * 3;

				int mouthC1X = faceROI.x_offset + mouthWidthDifference;
				int mouthC1Y = faceROI.y_offset + mouthHeightDifference*2;

				mouthROI.height = mouthHeight;
				mouthROI.width = mouthWidth;
				mouthROI.x_offset = mouthC1X;
				mouthROI.y_offset = mouthC1Y;

			}else if(mouthROIMethod.compare("two") == 0){
				int fl, fw, ft, fh;
				fl = faceROI.x_offset;
				fw = faceROI.x_offset + faceROI.width;
				ft = faceROI.y_offset;
				fh = faceROI.y_offset + faceROI.height;

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
				int mouthHeightDifference = faceROI.height/3;
				int mouthHeight = mouthHeightDifference;
				int mouthC1Y = faceROI.y_offset + mouthHeightDifference*2;
				int mouthC2Y = mouthC1Y + mouthHeight;

				int fl, fw;
				fl = faceROI.x_offset;
				fw = faceROI.x_offset + faceROI.width;

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

