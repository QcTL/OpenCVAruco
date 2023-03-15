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
  
  cv::VideoCapture webCam(std::atoi(argv[1]));       // Creaciò d'indentificació de camera

  if (webCam.isOpened() == false){   //Comprovar si podem accedir a la camera, sinò, mostrem un missatge d'error
    std::cerr << "error: Webcam could not be connected." << std::endl;
    return -1;
  }

  string cameraConfig = argv[2]; //Nom del fitxer de configuraciò de la camera
  string dictSelected = argv[3]; //Nom del diccionari.
  int id = stoi(argv[4]);
	float sideLong = stof(argv[5]); //Longitud del costat d’una marca ArUco sola, en metres (numèric)
	

 	
  char charCheckForESCKey = 0;
	
	//Cerca del diccionari amb el nom del mateix
  int at = 0;
  bool found = false;
  while(at<NDICT && !found){
  	if(dictSelected == DN[at]) found = true;
  	else at++;
  }
  
	//Error si no es trobat
  if(!found){
		cerr << "Dictionary " << dictSelected << " not found" << endl;
		return -1;
  }

	//Obtenir el diccionari amb la id
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(at);
	//Creaciò dels parametres de detecció de marques
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

//Creem les matrius, la que serà per la camera, i la que guardarà les transformacions de la calibraciò
  cv::Mat cameraMatrix, distCoeffs;
	//Apliquem les calibracions del fitxer a la matriu de la camera i la matriu de distorisio
  readCameraParameters(cameraConfig, cameraMatrix, distCoeffs);
  
	//Apliquem els punts que representaran els costats de les marques que detectarem, esta centrat en el superior esquerra i el voldrem en el 
	//Centre, aixì que utilitzarem la distancia real de la marca amb la mida "sideLong"
  cv::Mat objPoints(4, 1, CV_32FC3);
	objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-sideLong/2.f, sideLong/2.f, 0);
	objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(sideLong/2.f, sideLong/2.f, 0);
	objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(sideLong/2.f, -sideLong/2.f, 0);
	objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-sideLong/2.f, -sideLong/2.f, 0);
  
  
  //Vectors de descripció de la posició en la pantalla de les marques i quines son les seves indetificacions
  std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	
	//Matrius de la camera i el que es mostrarà per pantalla
	cv::Mat inputImage, outputImage;
	
	while(charCheckForESCKey != 27 && webCam.isOpened()){
		
		//Leguim de la camera
		webCam.read(inputImage);
		//Copiem la matriu al que es mostrara per pantalla
		inputImage.copyTo(outputImage);
		//Detectem totes les marques ha cada cicle que es mostren per pantalla
		cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
		if (markerIds.size() > 0){
     			//Si hi ha més de una marca, mostrem els seus contorns
			cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
			int nMarkers = markerCorners.size();
		  std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
		  // Aracalcularem amb els vectors creats, la posició en el mon real de cada marca, on rvecs representarà una llista de totes les rotacions que
			//tenen les marques i tvecs la seva posicio
		  for (int i = 0; i < nMarkers; i++) {
			  //Això ho fem per totes les marques
		      solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
		  }
		  for(unsigned int i = 0; i < markerIds.size(); i++) {
			  //Nomes mostrarem els eixos de la marca que l'usuari ha introduit
		  	if(markerIds[i] == id){
		  	//Calculem les coordenades de l'origen de la marca aruco respecte la camara. Amb el vector 3 de la posició d'aquella marca
				putText(outputImage, "Marker coordinate ( " + to_string(tvecs[i][0]) + "," + to_string(tvecs[i][1]) + "," + to_string(tvecs[i][2]) + ")", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
     		cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
		  	} 
		    else break;
		  }
		} 
		
    cv::imshow("out", outputImage);
		
		charCheckForESCKey = cv::waitKey(1); 
	}
	
  return 0;
}
