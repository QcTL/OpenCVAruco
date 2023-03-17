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

  string cameraConfig = argv[2]; //Nom del fitxer on està la calibració de la imatge
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

  cv::Mat cameraMatrix, distCoeffs;
	//Si volem configurar les caracteristiques de la camera perque utilitzi les configracuións de la calibraciò que hem 
	//Obtingut cal utilitzar la seguent funció, on del fitxer es tornarà a aconseguir la matriu 3x3 cameraMatrix i la matriu distCoeffs
	//Que vam generar en la calibració.
  readCameraParameters(cameraConfig, cameraMatrix, distCoeffs);
  
  
  //Creem tots els 8 punts que ha de tenir el nostre cub:
  //Aquests per ara no representaran res, no tindràn cap cordenada del mon real ni de la pantalla.
	//Per ara seran valors relatius que representen a quina figura es voldrà generar.
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
		//Serà aqui doncs quant estimarem la posició de les marques que veiem en pantalla amb les distoricions
		//De la camera aplicades, donant com a reultat interssant el rvecs i tvecs, que seràn la rotació i 
		//translació en el mon real de les mateix matques
	  cv::aruco::estimatePoseSingleMarkers(markerCorners, sideLong, cameraMatrix, distCoeffs, rvecs, tvecs);
	  
	  for(unsigned int i = 0; i < markerIds.size(); i++) {
		  //Ara de la marca que hem escollit en l'inici només d'ella voldrem mostrar el cub.
		if(markerIds[i] == id){
			//Dibuixem una linia entre tots els punts per formar un cub
			//Per fer això tornarem a fer la projeció del món real a la camera amb la funció project points.
			//Per això necessitarem els punts abtractes de abans, que ara tindràn la posició del mon de la pantalla, 
			//Un cop havent repesentat la seva posciò real gracies a tvecs[i] i rvecs[i]. 
			std::vector<cv::Point2f> imagePoints;
			projectPoints(cubeVertice, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints);
			//Això serà a causa que no podrem mostrar linies amb cordenades del mon real, aquestes
			//han de ser amb cordenades de la pantalla.
			//Aixì doncs, la funció el que farà serà generar una linia amb el color marcat per "Scalar"
			//Amb el gruix "3", del punt (X,Y), a (Xi,Yi), en la pantalla de sortida outputImage.
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
    cv::imshow("out", outputImage); //Mostrem per pantalla la matriu de output, que contindrà el cub.
    charCheckForESCKey = cv::waitKey(1);  // Obtenir la tecla actaul
	}
	
  return 0;
}
