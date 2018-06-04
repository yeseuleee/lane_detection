#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include <vector>

// Complile : g++ camera_image_test.cpp -o camera_image_test `pkg-config --cflags --libs opencv` -std=c++11

#define HORIZONTAL_LINE_NUMBER 11
#define VERTICAL_LINE_NUMBER 11

using namespace std;
using namespace cv;

static const bool CAPTURE = false;
static const bool VIDEO = false;
static const bool IMAGE = true;

int verticalMinOffset = 92;
int verticalMaxOffset = 317;
int lineNum = 11;
int verticalYMin = 246;
int verticalYMax = 828;
int verticalXCenter = 695;
int yCenter = 828;
double horizontalRatio = 0.8;
vector< vector< vector<int> > > calTable;
vector<double> gradientVec;
vector<double> yInterceptVec;

// int originXCenter = src.cols/2;

void mouseEvent( int event, int x, int y, int flags, void* userdata );

void verticalLine(Mat& src, int xcenter, int ymax, int ymin, int maxOff, int minOff);

void horizontalLine(Mat& src, Point2f origin, int lineNum, double ratio);

Mat rotateMat(Mat& src, Point2f origin, double degree);

// void moveImage(Mat& src, int xCenter, int originXCenter);

vector< vector< vector<int> > > makeCalTable(double xcenter, double ycenter, int maxOff, int minOff, Mat& src);

void calDistance( vector< vector< vector<int> > >& calTable, int x, int y , int& xDist, int& yDist);

int main(int argc, char** argv){

    int camera_number = 1;
    int frequency = 30;
    VideoCapture cap(0);
    int key;
    int num = 1;
    Mat src;
    Mat src2;
    Mat calib1;
    Mat calib2;
    Mat cameraMatrix= Mat::eye(3, 3, CV_64FC1);
    Mat distCoeffs = Mat::zeros(1, 5, CV_64FC1);

    cameraMatrix=(Mat1d(3, 3) << 708.611809, 0, 320.331083, 0, 703.012319, 260.343059, 0, 0, 1);
    distCoeffs=(Mat1d(1, 5) << 0.09776299999999999, -0.235306, 0.00463, -0.001884, 0);

    if(VIDEO){        setMouseCallback("Calibration 2", mouseEvent, &calib2);

      while(1){
          cap >> src;
          undistort(src, calib1, cameraMatrix, distCoeffs);

          imshow("origin",src);
          // cout<<"src x : "<<src.cols << " / src y : "<<src.rows<<endl;

          resize(src, src2, Size(src.cols * 2, src.rows * 2), 0, 0, CV_INTER_LINEAR);
          imshow("Size up",src2);
          // cout<<"src2 x : "<<src2.cols << " / src_up y : "<<src2.rows<<endl;


          //undistort(src2, calib2, cameraMatrix, distCoeffs);
          resize(calib1, calib2, Size(calib1.cols * 2, calib1.rows * 2), 0, 0, CV_INTER_LINEAR);

          imshow("Calibration 1",calib1);

          imshow("Calibration 2",calib2);

          key = waitKey(30);
          if(key == 27) {
              break;
          }
          else if (key == 32 && CAPTURE == true) {

              cout<<"Image write "<<to_string(num)<<endl;
              imwrite("../sample_image/src_"+to_string(num)+".jpg", src);
              imwrite("../sample_image/src2_"+to_string(num)+".jpg", src2);
              imwrite("../sample_image/calib1_"+to_string(num)+".jpg", calib1);
              imwrite("../sample_image/calib2_"+to_string(num)+".jpg", calib2);

              num++;

          }
      }
    }

    if(IMAGE){
      Point2f origin(377, 837);

      while(1){
        cout<<"!!"<<endl;
        src = imread("../sample_image/src_8.jpg", CV_LOAD_IMAGE_COLOR);
        resize(src, src2, Size(src.cols * 2, src.rows * 2), 0, 0, CV_INTER_LINEAR);
        undistort(src, calib1, cameraMatrix, distCoeffs);
        resize(calib1, calib2, Size(calib1.cols * 2, calib1.rows * 2), 0, 0, CV_INTER_LINEAR);

        double degree = -1;
        calib2 = rotateMat(calib2,origin, degree);

        imshow("origin",src);
        //imshow("Size up",src2);
        //imshow("Calibration 1",calib1);

        horizontalLine(calib2, origin, lineNum, horizontalRatio);
        verticalLine(calib2, verticalXCenter, verticalYMax, verticalYMin, verticalMaxOffset, verticalMinOffset);
        // moveImage(calib2, verticalXCenter, originXCenter);

        calTable = makeCalTable((double)verticalXCenter, (double)yCenter, verticalMaxOffset, verticalMinOffset, calib2);

        imshow("Calibration 2",calib2);

        setMouseCallback("Calibration 2", mouseEvent, &calib2);

        key = waitKey();
        if(key == 27) break;
      }
    }
    return 0;
}

void mouseEvent( int event, int x, int y, int flags, void* userdata ){
  if(event == EVENT_LBUTTONDOWN){
    cout<<"x : "<<x <<endl;
    cout<<"y : "<<y<<endl;

    int xDist, yDist;
    calDistance(calTable, x, y, xDist, yDist);
  }
}

void horizontalLine(Mat& src, Point2f origin, int lineNum, double ratio){

  // //2.5
  // height = 830;
  // line(src, Point(0,height), Point(src.cols,height), Scalar(255,0,0));
  // //3
  // height = 672;
  // line(src, Point(0,height), Point(src.cols,height), Scalar(255,0,0));
  // //3.5
  // height = 568;
  // line(src, Point(0,height), Point(src.cols,height), Scalar(255,0,0));
  // //4
  // height =487;
  // line(src, Point(0,height), Point(src.cols,height), Scalar(255,0,0));
  // //4.5
  // height = 428;
  // line(src, Point(0,height), Point(src.cols,height), Scalar(255,0,0));
  // //5
  // height = 379;
  // line(src, Point(0,height), Point(src.cols,height), Scalar(255,0,0));
  // //5.5
  // height = 339;
  // line(src, Point(0,height), Point(src.cols,height), Scalar(255,0,0));
  // //6
  // height = 308;
  // line(src, Point(0,height), Point(src.cols,height), Scalar(255,0,0));
  // //6.5
  // height = 282;
  // line(src, Point(0,height), Point(src.cols,height), Scalar(255,0,0));
  // //7
  // height = 258;
  // line(src, Point(0,height), Point(src.cols,height), Scalar(255,0,0));
  // //7.5
  // height = 239;
  // line(src, Point(0,height), Point(src.cols,height), Scalar(255,0,0));

  int yPoint = origin.y - 10;
  //int yArray[] = { 830, 672, 563, 487, 428, 379, 339, 308, 282, 258, 239};
  int yArray[] = { 828, 672, 563, 487, 429, 381, 344, 313, 286, 265, 246};
  for(int i=0; i<lineNum; i++){
    //cout<<yPoint<<endl;

    line(src, Point(0, yArray[i]), Point(src.cols,yArray[i]), Scalar(255,0,0));

    yPoint *=ratio;
  }




}

void verticalLine(Mat& src, int xcenter, int ymax, int ymin, int maxOff, int minOff){
  // horizontal



  // // 0
  line(src, Point(xcenter,ymin), Point(695,ymax), Scalar(255,0,0));

  for(int i=1; i<=5; i++){
    line(src, Point(xcenter-minOff*i,ymin), Point(xcenter-maxOff*i,ymax), Scalar(255,0,0));
    line(src, Point(xcenter+minOff*i,ymin), Point(xcenter+maxOff*i,ymax), Scalar(255,0,0));
  }
}

Mat rotateMat(Mat& src, Point2f origin, double degree){

  Mat rotMat = getRotationMatrix2D(origin, degree, 1.0);
  Mat rotImg;
  warpAffine(src,rotImg, rotMat, Size(), INTER_LINEAR);
  return rotImg;
}

// void moveImage(Mat& src, int xCenter, int originXCenter){
//
//   int mvOffset = xCenter - originXCenter;
//
//   for(int y = 0; y<src.rows; y++){
//     for(int x = 0; x<src.cols; x++){
//       src[y][x] = src[y][x-originXCenter];
//     }
//   }
// }

vector< vector< vector<int> > > makeCalTable(double xcenter, double ycenter, int maxOff, int minOff, Mat& src){
  vector<int> yVec = { 828, 672, 563, 487, 429, 381, 344, 313, 286, 265, 246};
  vector<int> xVec;

  vector< vector< vector<int> > > calTable; // 칼리브레이션 테이블에 대한 자료구조. 마지막 2개짜리 벡터에는 각각 x, y좌표가 등록된다

  calTable.assign(yVec.size(), vector< vector<int> >(11, vector<int>(2,0)));

  cout<<"Yvec[10] "<<yVec[10]<<endl;
  cout<<"yVecsize" <<yVec.size()<<endl;

  // 원점 좌측 기울기
  for(int i=5 ; i>0; i--){
    gradientVec.push_back( (ycenter - yVec[10] ) / ( ( xcenter - maxOff*i ) - ( xcenter - minOff*i ) ) );
  }

  // 원점 생략
  gradientVec.push_back(1);

  // 원점 우측 기울기
  for(int i=4; i>=0; i--){
    gradientVec.push_back(-1 * gradientVec[i]);
  }

  cout<<"gradient :"<<endl;
  for(int i=0; i<11; i++){
    cout<<gradientVec[i]<<endl;
  }

  // 원점 좌측 y절편
  for(int i=5 ; i>0; i--){
    // yInterceptVec.push_back(gradientVec[5-i] * ( xcenter - minOff ) / yVec[9] + 246);
    yInterceptVec.push_back( -1 * gradientVec[5-i] * ( xcenter - minOff * i ) + yVec[10] );

  }

  // 원점 생략
  yInterceptVec.push_back(0);

  // 원점 우측 y절편
  for(int i=6; i<11; i++){
    yInterceptVec.push_back( -1 * gradientVec[i] * ( xcenter + minOff * ( i - 5 ) ) + yVec[10] );
  }

  cout<<"yIntercept :"<<endl;
  for(int i=0; i<11; i++){
    cout<<yInterceptVec[i]<<endl;
  }

  // 칼리브레이션 테이블 생성
  for(int row = 0; row < yVec.size(); row++){
    for(int col = 0; col < HORIZONTAL_LINE_NUMBER; col++){

        calTable[row][col][1] = yVec[ yVec.size() - row - 1 ];

        if(col == 5) calTable[row][col][0] = xcenter;
        else{
          calTable[row][col][0] = ( calTable[row][col][1] - yInterceptVec[col] ) / gradientVec[col];
        }

        circle(src, Point( calTable[row][col][0], calTable[row][col][1] ), 10, Scalar(0,0,255));
        cout<<"{ " << calTable[row][col][0] << "," << calTable[row][col][1] <<" }\t";
      }
      cout<<endl;
    }
  return calTable;
}

void calDistance( vector< vector< vector<int> > >& calTable, int x, int y , int& xDist, int& yDist){
  Point ptUpLeft(0,0);
  Point ptUpRight(0,0);
  Point ptDownLeft(0,0);
  Point ptDownRight(0,0);
  int upXGap, downXGap, yGap;
  int calX, calY;
  int upRowIdx, downRowIdx, leftColIdx, rightColIdx;
  int xDistOff, yDistOff;
  vector<int> yVec = { 828, 672, 563, 487, 429, 381, 344, 313, 286, 265, 246};

  cout<<"x :"<<x<<", y :"<<y<<endl;

  // 인접한 4개의 기준점 찾기

  for(int row = 0; row < yVec.size(); row++){
    upRowIdx = row-1;
    downRowIdx = row;

    calY = calTable[row][0][1];
    if(calY > y){
      ptUpLeft.y = calTable[upRowIdx][0][1];
      ptUpRight.y = calTable[upRowIdx][0][1];
      ptDownRight.y = calTable[downRowIdx][0][1];
      ptDownLeft.y = calTable[downRowIdx][0][1];
      break;
    }
  }

  ptDownLeft.y = calTable[downRowIdx][0][1];

  for(int col = 0; col < HORIZONTAL_LINE_NUMBER; col++){
    leftColIdx = col - 1;
    rightColIdx = col;
    calX = calTable[downRowIdx][col][0];

    if(calX > x){
      ptUpLeft.x = calTable[upRowIdx][leftColIdx][0];
      ptUpRight.x = calTable[upRowIdx][rightColIdx][0];
      ptDownLeft.x = calTable[downRowIdx][leftColIdx][0];
      ptDownRight.x = calTable[downRowIdx][rightColIdx][0];
      break;
    }
  }

  upXGap = abs(ptUpRight.x - ptUpLeft.x);
  downXGap = abs(ptDownRight.x - ptDownLeft.x);
  yGap = abs(ptUpRight.y - ptDownRight.y);


  cout<<"UL : "<< ptUpLeft.x <<", "<<ptUpLeft.y << endl;
  cout<<"UR : "<< ptUpRight.x <<", "<<ptUpRight.y << endl;
  cout<<"DL : "<< ptDownLeft.x <<", "<<ptDownLeft.y << endl;
  cout<<"DR : "<< ptDownRight.x <<", "<<ptDownRight.y << endl;

  cout<<"upXGap" << upXGap<<endl;
  cout<<"downXGap" << downXGap<<endl;
  cout<<"yGap" << yGap<<endl;

  // 거리계산

  int tYGap, tXGap;
  int tGradientIdx, tYInterceptIdx;






  return;
}
