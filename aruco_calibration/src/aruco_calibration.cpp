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
  
  //Com en l'exercici anterior tornem a capturar la camera amb la id que ens ha entrat l'usuari
  cv::VideoCapture webCam(std::atoi(argv[1]));

	//I comprobem que funciona i que és valida:
  if (webCam.isOpened() == false){
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
  
  //Comprobem que es fa prem la tecla C per fer captures
  char charCheckForCKey = 0;
  
  //Tornem a conseguir la id diccionari de la paraula entrada per l'usuari, mostrant un missatge si no surt.
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
  //----
  
  //Creem el diccionari
 	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(at);
	//Creem la taula de Aruco que haurem de detectar amb les caracteristiques entrades per l'usuari
	cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(nColumns, nRows, sideLong, marckerDist, dictionary);
 	
 
	cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
 
	//FileStorage fs(detectorPara, FileStorage::READ); //Carreguem els paràmetres del detector.
  /*
  bool readOk = (*detectorParams)->readDetectorParameters(fs.root());
  
  if(!readOk){
  	cout << "Fitxer de paràmetres del detector incorrecte" << endl;
  	return -1;
  }
  */
  
	//Com que tornem a fer detecció tornem a tenir un vector que contindrà les id dels arucos que surten en pantalla.
	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	
	//I tornem a tenir dos materials, el primer serà el que farem la detecció i el segon en el qual li posarem els grafics que indiquen
	//Els contorns de les deteccions i quina id ha detectat.
	cv::Mat inputImage, outputImage;
	
	//Variables per colectar la informacio de les imatges de calibració.
	vector<vector<vector<Point2f>>> allCorners;
	vector<vector<int>> allIds;
	Size imgSize;
	
	//Nombre de captures
	int nCapture = 0;
	cout << "fem bucle" << endl;
	while(charCheckForESCKey != 27 && webCam.isOpened()){
		webCam.read(inputImage);
		inputImage.copyTo(outputImage);
		
		//Tornem a detectat i mostrat els arucos detectats i la seva informació per pantalla.
		cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
		if (markerIds.size() > 0) cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
		//També mostrem en la matriu de outputImage quantes captues s'han get:
		putText(outputImage, "Fotos: ( " + to_string(nCapture) + "/20)", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
		
		cv::imshow("out", outputImage);
		
		charCheckForCKey = cv::waitKey(1);  // aconseguim quina tecla s'ha pres.
	
	
		if(charCheckForCKey == 'c' && markerIds.size() == nRows*nColumns){ //Si cliques la tecla 'c' i s'han detectat totes les maqrques aruco
			
			//Afeguim els punts del contorn de les marques aruco en un vector.
			allCorners.push_back(markerCorners);
			//També les seves id.
			allIds.push_back(markerIds);	
			imgSize = inputImage.size();
			// I guardem la imatge que hem fotograficat.
			cv::imwrite("Captura" + to_string(nCapture) + ".png", inputImage);
			nCapture++; //Agumentem el nombre de capturesa fetes
		}
		
		charCheckForESCKey = cv::waitKey(1);
	}
	
	
	//Un cop processades totes: i pres la tecla ESC:
	if(nCapture<4){ //Comprobem que hi haguin captures suficients
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
  
  //i calibrem la camera amb tots els punts i les marques fetes, això donerà una quantitat de distorsió que se li ha
  //d'aplicar a la camera perque representi lo més proxim la realitat
  repError = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                         markerCounterPerFrame, board, imgSize, cameraMatrix,
                                         distCoeffs, rvecs, tvecs, 0);


	//Tot seguit guaradrem aquesta informació trobada en un fitxer el qual s'ha entrat en els parametres finals
  bool saveOk = saveCameraParams(settingsOut, imgSize, 1, 0, cameraMatrix,
                                 distCoeffs, repError);
  if(!saveOk) { //Mostrem si no s'ha pogut guradar el fitxer:
      cerr << "Cannot save output file" << endl;
      return -1;
  }
  
  return 0;
}
