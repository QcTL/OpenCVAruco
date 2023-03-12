#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/dictionary.hpp" 
#include "aruco_samples_utility.hpp"
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

const int NDICT = 20;
const string DN[] = {"DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250","DICT_4X4_1000","DICT_5X5_50","DICT_5X5_100","DICT_5X5_250","DICT_5X5_1000","DICT_6X6_50","DICT_6X6_100","DICT_6X6_250","DICT_6X6_1000","DICT_7X7_50","DICT_7X7_100","DICT_7X7_250","DICT_7X7_1000","DICT_ARUCO_ORIGINAL","DICT_APRILTAG_16h5","DICT_APRILTAG_25h9","DICT_APRILTAG_36h10","DICT_APRILTAG_36h11"};
  

int main(int argc, char* argv[]) {
  if (argc < 5){
    cerr << "Falten paràmetres" << endl;
    return -1;
  }
  
  cv::VideoCapture webCam(std::atoi(argv[1]));       // VideoCapture object declaration. Usually 0 is the integrated, 2 is the first external USB one

  if (webCam.isOpened() == false){   // Check if the VideoCapture object has been correctly associated to the webcam
    std::cerr << "error: Webcam could not be connected." << std::endl;
    return -1;
  }

  string cameraConfig = argv[2];
  string dictSelected = argv[3]; //Nom del diccionari.
  int id = stoi(argv[4]);
	float sideLong = stof(argv[5]); //Longitud del costat d’una marca ArUco sola, en metres (numèric)
	

 	
  char charCheckForESCKey = 0;
  int at = 0;
  bool found = false;
  while(at<NDICT && !found){
  	if(dictSelected == DN[at]) found = true;
  	else at++;
  }
  
  if(!found){
		cerr << "Dictionary " << dictSelected << " not found" << endl;
		return -1;
  } 
 	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(at);
 	cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

 	//FileStorage fs(detectorPara, FileStorage::READ); //Carreguem els paràmetres del detector.
  /*
  bool readOk = (*detectorParams)->readDetectorParameters(fs.root());
  
  if(!readOk){
  	cout << "Fitxer de paràmetres del detector incorrecte" << endl;
  	return -1;
  }
  */
  cv::Mat cameraMatrix, distCoeffs;
  readCameraParameters(cameraConfig, cameraMatrix, distCoeffs);
  
  cv::Mat objPoints(4, 1, CV_32FC3);
	objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-sideLong/2.f, sideLong/2.f, 0);
	objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(sideLong/2.f, sideLong/2.f, 0);
	objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(sideLong/2.f, -sideLong/2.f, 0);
	objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-sideLong/2.f, -sideLong/2.f, 0);
  
  
  
  

  std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	cv::Mat inputImage, outputImage;
	
	//Variables per colectar la informacio de les imatges de calibració.

 
	
	while(charCheckForESCKey != 27 && webCam.isOpened()){
		webCam.read(inputImage);
		inputImage.copyTo(outputImage);
		cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
		if (markerIds.size() > 0){
		
			cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
			int nMarkers = markerCorners.size();
		  std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
		  // Calculate pose for each marker
		  for (int i = 0; i < nMarkers; i++) {
		      solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
		  }
		  // Draw axis for each marker
		  for(unsigned int i = 0; i < markerIds.size(); i++) {
		  	if(markerIds[i] == id) cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
		    else break;
		  }
		} 
		
    cv::imshow("out", outputImage);
		
		charCheckForESCKey = cv::waitKey(1);  // gets the key pressed
	}
	
  return 0;
}
