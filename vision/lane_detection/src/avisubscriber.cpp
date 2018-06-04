

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <lane_detection/coordinateMsg.h>
#include <lane_detection/vecMsg.h>
#include <lane_detection/dataSize.h>
using namespace std;
using namespace cv;
static const std::string OPENCV_WINDOW = "Image window";

#define COLS_OFFSET 0.1
#define ROWS_OFFSET 0.5
#define WANT_FRAME_RATE 10

vector<Vec2i> laneLinearEquation(int x0, int y0, int x1, int y1);
cv::Mat frame;
vector<Mat> frameVec;
int check = 0;

lane_detection::coordinateMsg data;
lane_detection::vecMsg vec_msg;
lane_detection::dataSize vec_data_size;

class InitImgProcessing{
    public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_img;
  
        InitImgProcessing() 
        : it_(nh_){
      
                sub_img = it_.subscribe("/videofile/image_raw",1,&InitImgProcessing::imgCb,this);
           
        }
        ~InitImgProcessing(){
       
        } 
        void initLane(cv::Mat frame);
        void imgCb(const sensor_msgs::ImageConstPtr& msg){
            cv_bridge::CvImagePtr cv_ptr;
            
            try{
                cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
                
            }
            catch(cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception : %s",e.what());
                return;
            }
            initLane(cv_ptr->image);
           
        }
        
   
        
};



std::vector<Vec4i> HoughTransform(cv::Mat rawImg){
    vector<Vec4i> laneInfo;
    cv::HoughLinesP(rawImg, laneInfo, 1, CV_PI/180.0, 100, 80, 5);//함수 재구성..
    return laneInfo;
}


cv::Mat myCanny(cv::Mat rawImg){
    int minThrForCanny = 80;
    int maxThrForCanny = 15;
    int kernelSize = 3;
    vector<Vec4i> laneInfo;
    cv::Mat grayImg, blurImg, cannyImg;
    cv::cvtColor(rawImg,grayImg,COLOR_BGR2GRAY);
    cv::GaussianBlur(grayImg,blurImg,Size(5,5),0,5);
    cv::Canny(blurImg,cannyImg,minThrForCanny,maxThrForCanny,kernelSize);//얘네 매개변수 결정하는거 다 함수로 빼기
    return cannyImg;
}
void storeFrame(cv::Mat frame){
    if(!frame.empty()){
    frameVec.push_back(frame);
    check++;
    ROS_INFO("check \t %d",check);
    cout<<frameVec.back()<<endl<<endl<<endl;
    if(check>10){
        check = 0;
        frameVec.clear();
    }
    }
}

void checkFrame(vector<Mat> framevector){
    vector<Mat>::iterator FI;
    if(!framevector.empty()&&check>=10){
        FI = framevector.begin();
    }
    if(check>=10){
        
    while(FI != framevector.end()){
        
       // if(FI){
        cout<<*FI<<endl;
        ++FI;
       // }
//        cout<<framevector.back()<<endl;
    }
    }FI = framevector.begin();
}

vector<Vec2i> laneLinearEquation(int x0, int y0, int x1, int y1){//for PointCloud
        vector<Vec2i> laneXYVec;      
        float m,j;
        if(x0 != x1){
            m = (float)(y1-y0)/(float)(x1-x0);             
        }
        for(int i = x0; i<x1; ++i){
            j = m*(float)(i-x0) + y0;       
            laneXYVec.push_back(Vec2i(i,j));
        }
        return laneXYVec;
}

void InitImgProcessing::initLane(cv::Mat frame){
   
        cv::Mat laneROI;
        vector<Vec4i> laneInfo;
        
        if(!frame.empty()){        
             cv::resize(frame,frame,cv::Size(10,10),0,0,CV_INTER_AREA);//resize(img,resizeImg,size(W,H),interpol)
                                                                        //interpolation : CV_INTER_AREA(size down), CV_INTER_CUBIC(size up) 
                                                                        //CV_INTER_NN : 최근방 이웃 보간법
                                                                        //CV_INTER_LINEAR : 양선형 보간법
                                                                        //CV_INTER_AREA : 픽셀 영역 재 샘플링
                                                                        //CV_INTER_CUBIC : 3차 회선 보간법(화질 젤 좋다)
             laneROI = frame(//Rect(int x, int y, int width, int height) 
             cv::Rect(frame.cols*COLS_OFFSET, 
             frame.rows*ROWS_OFFSET, 
             frame.cols - frame.cols*COLS_OFFSET,
             frame.rows - frame.rows*ROWS_OFFSET) );//frame.rows - (frame.rows*0.07) - frame.rows*ROWS_OFFSET)  
                    
             frame = myCanny(frame);//lane 안찾은상태임

             storeFrame(frame);
             if(!frameVec.empty()){
                checkFrame(frameVec);
             }
        }else{//frame.empty() == TRUE
            ROS_INFO("frame is empty!");
        }

        if(!laneInfo.empty()){
            vector<Vec4i>::iterator it = laneInfo.begin();
            vector<Point2i> corners(4);////wapping작업하기!!!
            
            float ladian = atan2f((*it)[3] - (*it)[1], (*it)[2] - (*it)[0]);
            int degree = ladian*180/CV_PI; 

            while(it != laneInfo.end()){
              
                ladian = atan2f((*it)[3] - (*it)[1], //한 벡터에서 각 인덱스 순서대로 x0,y0,x1,y2를 뜻한다.
                                (*it)[2] - (*it)[0]);
                degree = ladian*180/CV_PI;

                if(abs(degree)>25 && abs(degree)<36){//true lane degree
                    if(degree<=0){//left lane
                        cout<<"__LeftLane : "<<*it<<endl;
                   
                    }else{//degree>0 means right lane
                        cout<<"__RighLane : "<<*it<<endl;
                        }
                         
                     cv::line(frame, 
                             Point((*it)[0] + COLS_OFFSET*frame.cols, 
                                   (*it)[1] + ROWS_OFFSET*frame.rows), 
                             Point((*it)[2] + COLS_OFFSET*frame.cols, 
                                   (*it)[3] + ROWS_OFFSET*frame.rows), 
                             Scalar(255,0, 0), 3, 0);
                              
                    vector<Vec2i> int2vec;
                    vector<Vec2i>::iterator int2iter;
              
                    int2vec = laneLinearEquation((*it)[0],(*it)[1],(*it)[2],(*it)[3]);
                    int2iter = int2vec.begin();
        /*            if(!vec_msg.coordinateVec.empty()){
                        vec_msg.coordinateVec.clear();
                    }
                    while(int2iter != int2vec.end()){//int2vec use debug...
                        data.x_data = (*int2iter)[0];
                        data.y_data = (*int2iter)[1];
                        vec_msg.coordinateVec.push_back(data);        
                        ++int2iter;     
                    }vec_data_size.data_size = int2vec.size();
          */       
                }else{//stop lane degree or none lane degree
                    if(abs(degree)>=180 && abs(degree)<190){//stop lane
                        ROS_INFO("STOP!");
                    }else{//none lane
                        ROS_INFO("NONE LANE!");
                    }
                }
                ++it;
                }
            }else{//laneInfo is empty.
                ROS_INFO("NO LaneInfo!");
            }
          //  cv::imshow("video",frame);
            int ckey = waitKey(10);
            if(ckey == 27) exit(1);
  
    }

int main(int argc, char **argv){
     ros::init(argc, argv, "avi_subscriber");
  //   ros::Publisher pub_img_point;
  //   ros::Publisher pub_point_size;
 //   try{
        InitImgProcessing i_ip;
       // ros::spin();
        ros::NodeHandle n;    
        //    pub_img_point = n.advertise<lanedetection::vecMsg>("/lane_point",1000);
        //    pub_point_size = n.advertise<lanedetection::dataSize>("/lane_point_size",1000);         
        ros::Rate loop_rate(5);
        while(i_ip.nh_.ok()){//i_ip.nh_.ok() 
              ros::spinOnce();
              loop_rate.sleep();
        }

     return 0;

}





























//////////////////////////////////////////////////////////

//no class
  //   ros::NodeHandle nh;
  //   cv::startWindowThread();
  //   image_transport::ImageTransport it(nh);
    // ros::Rate loop_rate(5);
  //   image_transport::Subscriber sub = it.subscribe("/videofile/image_raw",10,imageCallback);
     
 //    ros::Publisher pub = nh.advertise<lanedetection::vecMsg>("/test",1000);
 //    ros::Publisher pub2 = nh.advertise<lanedetection::dataSize>("/dataSize",1000);

   //  while (nh.ok()){
     
//     pub.publish(msg2);
//     pub2.publish(vecdatasize);
  //   loop_rate.sleep();
 //    }








   /*               //using minimum lane
                    //degree = ladian*180/CV_PI;

                    vector<Vec4i> leftLaneInfo;
                    vector<Vec4i>::iterator leftit;
                    vector<Vec4i> rightLaneInfo;
                    vector<Vec4i>::iterator rightit;
                    leftit = leftLaneInfo.begin();
                    rightit = rightLaneInfo.begin();
                    int min_left = 0,min_right = 0,temp_left = 0, temp_right = 0;
                 while(checkit != checkLane.end()){
                     ladian = atan2f((*checkit)[3] - (*checkit)[1], //한 벡터에서 각 인덱스 순서대로 x0,y0,x1,y2를 뜻한다.
                                     (*checkit)[2] - (*checkit)[0]);
                     degree = ladian*180/CV_PI;
                    if(abs(degree)>25 && abs(degree)<36){//true lane degree
                    if(degree<=0){//left lane
                        leftLaneInfo.push_back(*checkit);
                        if(temp_left == 0){
                            min_left = (*checkit)[0]; 
                        }
                        temp_left = (*checkit)[0];
                        if(min_left>temp_left){
                            min_left = temp_left;
                            }              
                    }else{//degree>0 means right lane
                        rightLaneInfo.push_back(*checkit);
                        if(temp_right == 0){
                            min_right = (*checkit)[0];
                        }
                        temp_right = (*checkit)[0];
                        if(min_right>temp_right){
                            min_right = temp_right;
                           }
                        }
                    }
                    ++checkit;
                 }
                 leftit = leftLaneInfo.begin();
                 rightit = rightLaneInfo.begin();

               if(!leftLaneInfo.empty()&&!rightLaneInfo.empty()){
                while(leftit!=leftLaneInfo.end() && rightit!=rightLaneInfo.end()){
                    if(leftit<leftLaneInfo.end()){
                    if(abs(min_left - (*leftit)[0])<30){
//                    if(abs((*leftit)[2]-(*(leftit+1))[0])<30 || abs((*leftit)[0]-(*(leftit+1))[0])<50){
                       line(frame, 
                            Point((*leftit)[0] + COLS_OFFSET*frame.cols, 
                                  (*leftit)[1] + ROWS_OFFSET*frame.rows), 
                            Point((*leftit)[2] + COLS_OFFSET*frame.cols, 
                                  (*leftit)[3] + ROWS_OFFSET*frame.rows), 
                            Scalar(255,0, 0), 3, 0);
                    }
                    ++leftit;
                    }
                    if(rightit<rightLaneInfo.end()){
                        if(abs(min_right - (*rightit)[0])<30){
                            line(frame, 
                            Point((*rightit)[0] + COLS_OFFSET*frame.cols, 
                                  (*rightit)[1] + ROWS_OFFSET*frame.rows), 
                            Point((*rightit)[2] + COLS_OFFSET*frame.cols, 
                                  (*rightit)[3] + ROWS_OFFSET*frame.rows), 
                            Scalar(255,0, 0), 3, 0); 
                        }
                        ++rightit;
                    }
                    
                    }              
                }
                //if(abs(degree)>25 && abs(degree)<36){//true lane degree
                */





            //한 벡터에다가 while 3번정도 돌린 벡터정보 저장, size도 같이저장. 차선 부분 근처에서 나오는 좌표값들 중에서 크게 차이나는 x,y값은 버림.
            //크게 차이나지 않는 값중 안쪽에 있는 차선값을 선택함. -->그 차선을 탑뷰로 바꿈==> 각도 수직선으로 해서 차선 검출 및 라인그리기
            //라인정보를 publish.(it [3]까지의 벡터를 리턴하면 될듯.)




//Effective Robotics Programming with ROS
