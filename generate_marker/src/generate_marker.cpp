#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include <iostream>

using namespace std;
using namespace cv;
const int NDICT = 20;
const string DN[] = {"DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250","DICT_4X4_1000","DICT_5X5_50","DICT_5X5_100","DICT_5X5_250","DICT_5X5_1000","DICT_6X6_50","DICT_6X6_100","DICT_6X6_250","DICT_6X6_1000","DICT_7X7_50","DICT_7X7_100","DICT_7X7_250","DICT_7X7_1000","DICT_ARUCO_ORIGINAL","DICT_APRILTAG_16h5","DICT_APRILTAG_25h9","DICT_APRILTAG_36h10","DICT_APRILTAG_36h11"};
  

int main(int argc, char* argv[]) {
  if (argc < 4){
    cerr << "Falten paràmetres" << endl;
    return -1;
  }

  cv::Mat imgAru; // Matrix format image of the specified ID and size


  char charCheckForESCKey = 0;
  string dictSelected = argv[1];
  int idAruco = stoi(argv[2]);
  int sizeAruco = stoi(argv[3]);

  string imageName = argv[4];
  
  int i = 0;
  bool found = false;
  
  while(i<NDICT && !found){
  	if(dictSelected == DN[i]) found = true;
  	else i++;
  }
  
  if(!found){
  cerr << "Dictionary " << dictSelected << " not found" << endl;
  return -1;
  } 
 
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(i);
  
  cv::aruco::drawMarker(dictionary, idAruco, sizeAruco, imgAru);
  cv::imwrite(imageName, imgAru);
  
  return 0;
}
