/////////////////////////////////////////////////////////////////////////
////////////////////////////////////카메라 연결시 이용///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>

#include <iostream>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
using namespace std;
using namespace cv;


int main(int argc, char** argv){ 

    ros::init(argc, argv, "avi_publish");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/image_raw",1);
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    /*// 명령행 인자 이용 시
        if(argv[1]==NULL){
        while(1){
            std::istringstream video_sourceCmd(argv[1]);
            if(argv[1]!=NULL){
                break;
            }        
        }
    }
    std::istringstream video_sourceCmd(argv[1]);//카메라 번호를 매개변수로 받음.
    int video_source;
    if(!(video_sourceCmd >> video_source)){
        return 1;
    }
    cv::VideoCapture cap(video_source);    
    */
    
    VideoCapture cap(0);//실행파일이 있는 폴더에 동영상을 넣을 것. 절대경로 참조가 잘 안됨.
    //CvCapture* cap = cvCaptureFromAVI("./DriveSample.avi");
    if(cap.isOpened()){
        ROS_INFO("cap.isOpened() Success");
    }else{
        ROS_INFO("cap.isOpened() Fail");
    }
 /*   if(!cap.isOpened()){
        while(1){
            cv::VideoCapture cap("DriveSample.avi");
            if(cap.isOpened()){
                break;
            ROS_INFO("__6_error");    
            }
        }
    }
   */ 
    cap.set(CAP_PROP_FRAME_WIDTH,320);
    cap.set(CAP_PROP_FRAME_HEIGHT,240);
    ros::Rate loop_rate(5);
    while (nh.ok()){
        cap >> frame;
        if(!frame.empty()){//실제로 frame이 들어왔는지 확인.
            msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }else{
            ROS_INFO("no frame");
        }


      //  imshow("test",frame);

        ros::spinOnce();
        loop_rate.sleep();
        //ROS_INFO("while");
    }
   /* cv::namedWindow("video",1);
    
    while(1){
        cap >> frame;
        imshow("video",frame);
        if(waitKey(1)>=27){//esc누르면 종료.
              break;
             }
        }
       
        return 0;    
    */
 
}

//컴파일 :  g++ -o pixelpoint pixelpoint.cpp $(pkg-config --libs --cflags opencv)
//실행 : ./pixelpoint