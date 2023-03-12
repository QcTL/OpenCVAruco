#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include <iostream>
#include <vector>

using namespace std;

const int NDICT = 20;
const string DN[] = {"DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250","DICT_4X4_1000","DICT_5X5_50","DICT_5X5_100","DICT_5X5_250","DICT_5X5_1000","DICT_6X6_50","DICT_6X6_100","DICT_6X6_250","DICT_6X6_1000","DICT_7X7_50","DICT_7X7_100","DICT_7X7_250","DICT_7X7_1000","DICT_ARUCO_ORIGINAL","DICT_APRILTAG_16h5","DICT_APRILTAG_25h9","DICT_APRILTAG_36h10","DICT_APRILTAG_36h11"};
  

int main(int argc, char* argv[]) {
  if (argc < 2){
    cerr << "Falten paràmetres" << endl;
    return -1;
  }
  
  // A causa que haurem de detectar els aruco amb la camera, la generarema amb la seguent linia.
  //Amb la id que ha entrat l'usuari
  cv::VideoCapture webCam(std::atoi(argv[1]));      

//Comprobarem si hi podem accedir a la camera, sinò mostrarem un error.
  if (webCam.isOpened() == false){ 
    std::cerr << "error: Webcam could not be connected." << std::endl;
    return -1;
  }

  //Guardem el nom del diccionari el qual voldrem detectar els aruco capturats per la camera.
  string dictSelected = argv[2];
 	
  char charCheckForESCKey = 0;
  
  //Tornem a fer el procces de obtenir el id del dicconari que l'usuari ha introduit
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
	
	//Generarem els Parametres basic per el detector de Arucos:
 	cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
	
	//Crearem el vector on es guardaran les Ids del arucos que seràn capturats per la camera, aquesta llista anirà canviant en cada frame d'execució.
  	std::vector<int> markerIds;
	
	//Guardarem un vector els 4 punts on la camera ha detectat Aruco i un altre de on la camera ha cregut que eren arucos pero han resultat que no eren
	//Aquesta funció només la crearem per la funció de detecció i en aquest cas no li farem res.
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	
	//Creem la matriu que la camera hi projectarà i la altre matriu que serà la que es mostarà per pantalla que serà la camera més informació i posició
	//sobre els arucos detectats
	cv::Mat inputImage, outputImage;
	
	
	while(charCheckForESCKey != 27 && webCam.isOpened()){
		//Lleguim el frame de la camera i el guardem en la matriu.
		webCam.read(inputImage);
		//Copiem la matriu de la camera a la que es mostrarà per pantalla
		inputImage.copyTo(outputImage);
		//Detectem les marques aruco que hi ha en la camera que formin part del diccionari entrat
		cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
		
		//Si hem detectat algun aruco, mostrarem la seva posició i la seva id en la matriu de outputImage.
		if (markerIds.size() > 0) cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    	
		//generem una finestra amb el nom de "out" i mostrarem la matriu que conté la informació important.
		cv::imshow("out", outputImage);
		charCheckForESCKey = cv::waitKey(1);  // gets the key pressed
	}
	
  return 0;
}
