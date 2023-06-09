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

 
  string dictSelected = argv[1]; //Nom del diccionari.
  string detectorPara = argv[2]; //Nom del fitxer .yml on està la configuracio del detector aruco. 
  int nRows = stoi(argv[3]); //Files de la matriu de marques.
  int nColumns = stoi(argv[4]); //Columnes de la matriu de marques.
  float sideLong = stof(argv[5]); //Longitud del costat d’una marca ArUco sola, en metres (numèric)
  float marckerDist = stof(argv[6]); //Distància entre marques ArUco, en metres (numèric)
  string settingsOut = argv[7]; //Nom del fitxer que conté la calibració de la càmera.
 	
  char charCheckForKey = 0;
  int at = 0;
  bool found = false;

	//Tecnica utilitzada en altres exercicis per aconseguir el diccionari basat en una entrada string del mateix.
  while(at<NDICT && !found){
  	if(dictSelected == DN[at]) found = true;
  	else at++;
  }
  
  if(!found){
	cerr << "Dictionary " << dictSelected << " not found" << endl;
	return -1;
  } 
	
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(at);
	//Inicializador del detector d'arucos.
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
	
	//Creació de la taula de les quals hem impres i farem fotografies, per el correcte funcionament ha de ser exactament aquesta.
  cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(nColumns, nRows, sideLong, marckerDist, dictionary);
  FileStorage fs(detectorPara, FileStorage::READ); //Carreguem els paràmetres del detector.

  bool readOk = detectorParams->readDetectorParameters(fs.root());
  if(!readOk){
  	cout << "Fitxer de paràmetres del detector incorrecte" << endl;
  	return -1;
  }

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Mat inputImage, outputImage;
	
  //Variables per colectar la informacio de les imatges de calibració.
  vector<vector<vector<Point2f>>> allCorners;
  vector<vector<int>> allIds;
  Size imgSize;
  int nCapture = 0;
  while(charCheckForKey != 27 && webCam.isOpened()){
	  
	  //Com en l'exercici de detecció de aruco, tindrem dos matrius, la primera serà simplement la que rep la camara i que utilitzarem
	  //Per detectar arucos, la segona, una que es mostrarà a l'usuari i que tindrà informaciò sobre el que està veient la camera.
	webCam.read(inputImage);
	inputImage.copyTo(outputImage);
	cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
	
	  //Si trobem alguna aruco per pantalla, la mostrem en la matriu de sortida amb les seves qualtitats.
	if (markerIds.size() > 0) cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
	
	  //Mostrem el text per pantalla, per això necessitarem definir sobre quina matriu es mostrarà i les caracteristiques del text.
	  	//Cal dir que el Scalar en aquest context és Blue, Green, Red
	putText(outputImage, "Captures taken ( " + to_string(nCapture) + "/20)", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
		
    	cv::imshow("out", outputImage);
    
    	charCheckForKey = cv::waitKey(1);  // Detectem la tecla pressa abans del if per poder fer una fotografia en el mateix cicle
	  					//que estem veien les marques en la imatge.

	if(charCheckForKey == 'c' && markerIds.size() == nRows*nColumns && nCapture<=20){ //Si cliques la tecla 'c' i s'han detectat totes les maqrques aruco
		allCorners.push_back(markerCorners);
		allIds.push_back(markerIds);	
		imgSize = inputImage.size();
		cv::imwrite("Captura" + to_string(nCapture) + ".png", inputImage);
		nCapture++;
	}
}
	
  if(nCapture<19){
	cout << "captures insuficients" << endl;
	return -1;
  }

  Mat cameraMatrix, distCoeffs;
  vector< Mat > rvecs, tvecs; //Representaran els diferents valors de rotació i translació que tenen les marques en les imatges realitzades.
  double repError;
  
  vector<vector<Point2f>> allCornersConcatenated;
  vector<int> allIdsConcatenated;
  vector<int> markerCounterPerFrame;
  markerCounterPerFrame.reserve(allCorners.size());
	
	//Voldrem tenir un sol vector de vectors de punts que representaran el seguit de marques totes juntes, també voldrem tenir una
	//Llista de totes les identificacions de les marques entrades, com també quines marques apareixen per cada foto.
  for(unsigned int i = 0; i < allCorners.size(); i++) {
      markerCounterPerFrame.push_back((int)allCorners[i].size());
      for(unsigned int j = 0; j < allCorners[i].size(); j++) {
          allCornersConcatenated.push_back(allCorners[i][j]);
          allIdsConcatenated.push_back(allIds[i][j]);
      }
  }
  
  // Calibrate camera
  repError = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                         markerCounterPerFrame, board, imgSize, cameraMatrix,
                                         distCoeffs, rvecs, tvecs, 0);
   /*Parametres:
	- És una llista de tots els costats detectats en totes les fotos
	- Una llista de tots els identificadors que haurian de representar els mateixos valors i ordres de la llista anterior
	- Quants marcadors hi ha en cada fotografia
	- De quina "Board" s'han fet les captures
	- La mida de les marques en metres de les quals s'han fet la fotografia
	- Se li entrarà una matriu i generarà una matriu 3x3 amb la transformació
	- S'emplena una matriu i es retorneran els coficients de distorció de la camera.
	- rvecs representarà un vector que contindrà la rotació de cada una de les marques que s'han entrat
	- tvecs representarà un vector que contindrà la translació de cada una de les marques que s'han entrat
	- l'ultim canvia el valor entrat per unvector dels errors estimats que s'han donat a cada marca, però introduim un 0
		a causa que no utilitzarem el valor.
  */	
	
	
  //Guardem el fitxer .yml amb la configuracio
  bool saveS = saveCameraParams(settingsOut, imgSize, 1, 0, cameraMatrix,
                                 distCoeffs, repError);
  /*Parametres:
	- Nom del fitxer de calibracio que es guardara
	- Mida de la imatge que s'ha guardat, en el nostre cas sera el la mida de les fotos que afagem, 
		de aquesta manera ens assegurem que es la mida de la camera
	- El especte de la camera, en aquest cas es 1:1, aixi que sera 1
	- El seguent camp sera per si es volen introduir flag, en el nostre cas no s'introduiran
	- El proxim sera la matriu que portara la calibracio 3x3 en el seus valors
	- Aquest sera els propis valors que distorisio que s'han aplicat a els pixels per aproximar-los
	- El valor que ens ha retornat la calibracio que intenta aproximar els errors que s'han aplicat en
		realitzar la transformacio
  */	
  //Comprovacio si no s'ha pogut guardarelfitxer
  if(!saveS) {
      cerr << "Cannot save output file" << endl;
      return 0;
  }
	
  //Mostrar Informacio sobre com ha anat el calibratge
  cout << "Rep Error: " << repError << endl;
  cout << "Calibration saved to " << settingsOut << endl;
	
  return 0;
}
