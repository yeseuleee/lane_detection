#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/opencv.hpp"

// Complile : g++ camera_check.cpp -o camera_check `pkg-config --cflags --libs opencv` -std=c++11
// ESC : 종료
// ENTER : 다음 카메라
// SPACE : 사진 촬영. ../sample_image에 저장


#define MAX_CAM 10

using namespace std;
using namespace cv;

void mouseEvent( int event, int x, int y, int flags, void* userdata );

int main(int argc, char** argv){
  VideoCapture cam[MAX_CAM];
  char status[MAX_CAM];
  Mat frame[MAX_CAM];
  Mat frameX2[MAX_CAM];
  Mat frameWNumber[MAX_CAM];
  int showNum = 0;
  int imgNum = 0;
  memset(status, 0, sizeof(status));

  for(int i=0; i<MAX_CAM; i++){
    cam[i].open(i);
    if(!cam[i].isOpened()){
      cout<<"Webcam number "<<i<<" open failed"<<endl;
      status[i] = -1;
      cam[i].release();
    }
    else{
      status[i] = 1;
    }
  }



  while(1){
      String cam_num;
      char textBuf[5];
      cam[showNum]>>frame[showNum];
      resize(frame[showNum], frameX2[showNum], Size(frame[showNum].cols * 2, frame[showNum].rows * 2), 0, 0, CV_INTER_LINEAR);

      cam_num = to_string(showNum);
      frame[showNum].copyTo(frameWNumber[showNum]);
      putText(frameWNumber[showNum], cam_num, Point(10, 150), 2, 3, Scalar::all(255));

      imshow("cam", frameWNumber[showNum]);

      setMouseCallback("cam", mouseEvent, &frame[showNum]);

      int key = waitKey(30);
      if(key == 27) {
          break;
      }
      else if (key == 13) {
        cam[showNum].release();
        destroyAllWindows();
        for(int i = showNum+1; i<MAX_CAM; i++){
          cout<<"i :" << i<<endl;
          if(status[i] == 1){
            showNum = i;
            cout<<"shownum : "<<showNum<<endl;
            break;
          }
          if(i == MAX_CAM-1){
            showNum = 0;
            break;
          }
        }
        cam[showNum].open(showNum);
      }
      else if(key == 32){
        cout<<"Save vision/camera_image/sample_image/origin_"<<imgNum<<".jpg"<<endl;
        cout<<"Save vision/camera_image/sample_image/sizeX2_"<<imgNum<<".jpg"<<endl;
        imwrite("../sample_image/origin_"+to_string(imgNum)+".jpg", frame[showNum]);
        imwrite("../sample_image/sizeX2_"+to_string(imgNum)+".jpg", frameX2[showNum]);
        imgNum++;

      }

  }

  return 0;

}

void mouseEvent( int event, int x, int y, int flags, void* userdata ){
  if(event == EVENT_LBUTTONDOWN){
    cout<<"x : "<<x <<endl;
    cout<<"y : "<<y<<endl;
  }
}
