#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include <iostream>

using namespace std;

const int NDICT = 20;
const string DN[] = {"DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250","DICT_4X4_1000","DICT_5X5_50","DICT_5X5_100","DICT_5X5_250","DICT_5X5_1000","DICT_6X6_50","DICT_6X6_100","DICT_6X6_250","DICT_6X6_1000","DICT_7X7_50","DICT_7X7_100","DICT_7X7_250","DICT_7X7_1000","DICT_ARUCO_ORIGINAL","DICT_APRILTAG_16h5","DICT_APRILTAG_25h9","DICT_APRILTAG_36h10","DICT_APRILTAG_36h11"};
  

int main(int argc, char* argv[]) {
  if (argc < 6){
    cerr << "Falten paràmetres" << endl;
    return -1;
  }
  char charCheckForESCKey = 0;
  
  cv::Mat imgAruBoard; // Matrix format image of the specified ID and size
	int nRows = stoi(argv[1]);
	int nColumns = stoi(argv[2]);


  string dictSelected = argv[3];
 	
  int sizeAruco = stoi(argv[4]) * 0.0002645833; 
	int separationAruco = stoi(argv[5]) * 0.0002645833;
  string imageName = argv[6];
  
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
  cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(nColumns, nRows, sizeAruco, separationAruco, dictionary);
  
  board->draw( cv::Size(600, 500), imgAruBoard, 10, 1 );
  cv::imwrite(imageName, imgAruBoard);
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  return 0;
}
