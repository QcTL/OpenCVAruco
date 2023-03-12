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
  if (argc < 7){
    cerr << "Falten paràmetres" << endl;
    return -1;
  }
  
  cv::VideoCapture webCam(std::atoi(argv[1]));       // VideoCapture object declaration. Usually 0 is the integrated, 2 is the first external USB one

  if (webCam.isOpened() == false){   // Check if the VideoCapture object has been correctly associated to the webcam
    std::cerr << "error: Webcam could not be connected." << std::endl;
    return -1;
  }

  cout << "llegim parametres" << endl;
  string dictSelected = argv[1]; //Nom del diccionari.
  string detectorPara = argv[2]; //Nom del fitxer .yml on està la configuracio del detector aruco. 
  int nRows = stoi(argv[3]); //Files de la matriu de marques.
	int nColumns = stoi(argv[4]); //Columnes de la matriu de marques.
	float sideLong = stof(argv[5]); //Longitud del costat d’una marca ArUco sola, en metres (numèric)
	float marckerDist = stof(argv[6]); //Distància entre marques ArUco, en metres (numèric)
	string settingsOut = argv[7]; //Nom del fitxer que conté la calibració de la càmera.
 	
  char charCheckForESCKey = 0;
  char charCheckForCKey = 0;
  int at = 0;
  bool found = false;
  cout << "mirem diccionari" << endl;
  while(at<NDICT && !found){
  	if(dictSelected == DN[at]) found = true;
  	else at++;
  }
  
  if(!found){
		cerr << "Dictionary " << dictSelected << " not found" << endl;
		return -1;
  } 
  cout << "caca 1" << endl;
 	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(at);
 	cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
 	cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(nColumns, nRows, sideLong, marckerDist, dictionary);
 	//FileStorage fs(detectorPara, FileStorage::READ); //Carreguem els paràmetres del detector.
  /*
  bool readOk = (*detectorParams)->readDetectorParameters(fs.root());
  
  if(!readOk){
  	cout << "Fitxer de paràmetres del detector incorrecte" << endl;
  	return -1;
  }
  */
  cout << "caca 2" << endl;
  std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	cv::Mat inputImage, outputImage;
	
	//Variables per colectar la informacio de les imatges de calibració.
	vector<vector<vector<Point2f>>> allCorners;
  vector<vector<int>> allIds;
  Size imgSize;
	int nCapture = 0;
	cout << "fem bucle" << endl;
	while(charCheckForESCKey != 27 && webCam.isOpened()){
		webCam.read(inputImage);
		inputImage.copyTo(outputImage);
		cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
		if (markerIds.size() > 0) cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
		putText(outputImage, "Captures taken ( " + to_string(nCapture) + "/20)", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
		
		
		
		
    cv::imshow("out", outputImage);
    
    charCheckForCKey = cv::waitKey(1);  // gets the key pressed
    cout << "id trobats: " << markerIds.size() << endl;
    cout << "files*columnes" << nRows*nColumns << endl;
    cout << "tecla apretada" << charCheckForCKey << endl;
		if(charCheckForCKey == 'c' && markerIds.size() == nRows*nColumns){ //Si cliques la tecla 'c' i s'han detectat totes les maqrques aruco
			cout << "amongus" << endl;
			allCorners.push_back(markerCorners);
			allIds.push_back(markerIds);	
			imgSize = inputImage.size();
			cv::imwrite("Captura" + to_string(nCapture) + ".png", inputImage);
			nCapture++;
		}
		
		charCheckForESCKey = cv::waitKey(1);  // gets the key pressed
	}
	
	if(nCapture<4){
		cout << "captures insuficients" << endl;
		return -1;
	}
	
	Mat cameraMatrix, distCoeffs;
  vector< Mat > rvecs, tvecs;
  double repError;
  
  vector<vector<Point2f>> allCornersConcatenated;
  vector<int> allIdsConcatenated;
  vector<int> markerCounterPerFrame;
  markerCounterPerFrame.reserve(allCorners.size());
  for(unsigned int i = 0; i < allCorners.size(); i++) {
      markerCounterPerFrame.push_back((int)allCorners[i].size());
      for(unsigned int j = 0; j < allCorners[i].size(); j++) {
          allCornersConcatenated.push_back(allCorners[i][j]);
          allIdsConcatenated.push_back(allIds[i][j]);
      }
  }
  
  // calibrate camera
  repError = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                         markerCounterPerFrame, board, imgSize, cameraMatrix,
                                         distCoeffs, rvecs, tvecs, 0);

  bool saveOk = saveCameraParams(settingsOut, imgSize, 1, 0, cameraMatrix,
                                 distCoeffs, repError);

  if(!saveOk) {
      cerr << "Cannot save output file" << endl;
      return 0;
  }

  cout << "Rep Error: " << repError << endl;
  cout << "Calibration saved to " << settingsOut << endl;
	
  return 0;
}
