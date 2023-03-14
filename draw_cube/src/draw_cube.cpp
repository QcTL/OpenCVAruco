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
  

void drawLines(){
	
}


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
  
  
  //Creem tots els 8 punts que ha de tenir el nostre cub:
  
  std::vector<cv::Point3f> cubeVertice;
  cubeVertice.push_back(cv::Point3f(sideLong/2.f+sideLong/2.f, sideLong/2.f+sideLong/2.f, -sideLong));
  cubeVertice.push_back(cv::Point3f(sideLong/2.f+sideLong/2.f, -sideLong/2.f+sideLong/2.f, -sideLong));
  cubeVertice.push_back(cv::Point3f(-sideLong/2.f+sideLong/2.f, -sideLong/2.f+sideLong/2.f, -sideLong));
  cubeVertice.push_back(cv::Point3f(-sideLong/2.f+sideLong/2.f, sideLong/2.f+sideLong/2.f, -sideLong));
  cubeVertice.push_back(cv::Point3f(sideLong/2.f+sideLong/2.f, sideLong/2.f+sideLong/2.f, 0));
  cubeVertice.push_back(cv::Point3f(sideLong/2.f+sideLong/2.f, -sideLong/2.f+sideLong/2.f, 0));
  cubeVertice.push_back(cv::Point3f(-sideLong/2.f+sideLong/2.f, -sideLong/2.f+sideLong/2.f, 0));
  cubeVertice.push_back(cv::Point3f(-sideLong/2.f+sideLong/2.f, sideLong/2.f+sideLong/2.f, 0));
  
  //Projectem els punts

  std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	cv::Mat inputImage, outputImage;
	
	//Variables per colectar la informacio de les imatges de calibració.

 
	
	while(charCheckForESCKey != 27 && webCam.isOpened()){
		webCam.read(inputImage);
		inputImage.copyTo(outputImage);
		cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
		if (markerIds.size() > 0){
		  // Calculate pose for each marker
		  std::vector<cv::Vec3d> rvecs, tvecs;
		  cv::aruco::estimatePoseSingleMarkers(markerCorners, sideLong, cameraMatrix, distCoeffs, rvecs, tvecs);
		  // Draw axis for each marker
		  for(unsigned int i = 0; i < markerIds.size(); i++) {
		  	if(markerIds[i] == id){
		  		//Dibuixem una linia entre tots els punts per formar un cub
		  		std::vector<cv::Point2f> imagePoints;
  				projectPoints(cubeVertice, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints);
		  		cv::line(outputImage, imagePoints[0], imagePoints[1], cv::Scalar(255,0,0), 3);
		  		cv::line(outputImage, imagePoints[0], imagePoints[3], cv::Scalar(255,0,0), 3);
		  		cv::line(outputImage, imagePoints[0], imagePoints[4], cv::Scalar(255,0,0), 3);
		  		cv::line(outputImage, imagePoints[1], imagePoints[2], cv::Scalar(255,0,0), 3);
		  		cv::line(outputImage, imagePoints[1], imagePoints[5], cv::Scalar(255,0,0), 3);
		  		cv::line(outputImage, imagePoints[2], imagePoints[3], cv::Scalar(255,0,0), 3);
		  		cv::line(outputImage, imagePoints[2], imagePoints[6], cv::Scalar(255,0,0), 3);
		  		cv::line(outputImage, imagePoints[3], imagePoints[7], cv::Scalar(255,0,0), 3);
		  		cv::line(outputImage, imagePoints[4], imagePoints[5], cv::Scalar(255,0,0), 3);
		  		cv::line(outputImage, imagePoints[4], imagePoints[7], cv::Scalar(255,0,0), 3);
		  		cv::line(outputImage, imagePoints[5], imagePoints[6], cv::Scalar(255,0,0), 3);
		  		cv::line(outputImage, imagePoints[6], imagePoints[7], cv::Scalar(255,0,0), 3);
		  	} 
		    else break;
		  }
		} 
		
    cv::imshow("out", outputImage);
		
		charCheckForESCKey = cv::waitKey(1);  // gets the key pressed
	}
	
  return 0;
}
