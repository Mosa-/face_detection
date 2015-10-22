#include "FaceDetection.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "face_detection");

  ros::NodeHandle n("~");

  std::string mouthROIMethod;
  double thresholdKeepFaceROI = 0.90;
  bool useCam = true;

  FaceDetection* faceDetection = new FaceDetection(&n);

  n.getParam("mouthROI", mouthROIMethod);
  n.getParam("THkeepROI", thresholdKeepFaceROI);
  n.getParam("cam", useCam);

  faceDetection->prepareFaceDetection(mouthROIMethod, thresholdKeepFaceROI, useCam);

  ros::Subscriber camImage;
  if(useCam){
      camImage = n.subscribe("/kinect2/qhd/image_mono", 100, &FaceDetection::imageCallback, faceDetection);
  }else{
      faceDetection->sendFaceMouthROI();
  }

  ros::spin();

  return 0;
}
