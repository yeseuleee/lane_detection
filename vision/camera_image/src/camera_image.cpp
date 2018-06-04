#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <cstring>
#include <string>

// argv[0] : camera_number, argv[1] : frequency

static const std::string OPENCV_WINDOW = "Raw Image Window";

static int camera_num;
static int frequency;
static int debug;
static int calibration;
static int sizeup;
static std::string groupName;

//template < typename T > std::string to_string( const T& n );

class CameraImage{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher camera_image_pub_;

public:
    CameraImage(cv::Mat camMat, cv::Mat distMat)
        : it_(nh_),cameraMatrix(camMat), distCoeffs(distMat)
    {
        initParam();
        //createTopicName();
        if(debug) std::cout<<"topic_name : "<<"/"+ groupName +"/raw_image"<<std::endl;
        camera_image_pub_ = it_.advertise("/"+ groupName +"/raw_image",1);
        cap.open(camera_num);

        if(calibration){

            if(groupName == "main") {
              cameraMatrix=(cv::Mat1d(3, 3) << 625.546173, 0, 329.374745, 0, 625.809222, 221.233758, 0, 0, 1);
              distCoeffs=(cv::Mat1d(1, 5) << 0.103856, -0.124799, -0.001845, 0.012219, 0);
              ROS_INFO("main");
            }
            else if(groupName == "left"){// cam 1
              cameraMatrix=(cv::Mat1d(3, 3) << 630.071853, 0, 322.309971, 0, 632.842228, 247.329905, 0, 0, 1);
              distCoeffs=(cv::Mat1d(1, 5) << 0.010162, -0.060262, 0.001452, -0.001965, 0);
              ROS_INFO("left");
            }
            else if(groupName == "right"){// cam 3
              cameraMatrix=(cv::Mat1d(3, 3) << 603.652456, 0, 328.452174, 0, 604.1248849999999, 228.433349, 0, 0, 1);
              distCoeffs=(cv::Mat1d(1, 5) << -0.033672, -0.031004, 0.001614, 0.007620999999999999, 0);
              ROS_INFO("right");
            }


        }
    }

    ~CameraImage()
    {
        if(debug) cv::destroyWindow(OPENCV_WINDOW);
    }

    void sendImage(); // image 퍼블리시
    std::string createTopicName();// topic이름 생성
    void initParam();

private:
    cv::VideoCapture cap;
    std::string topic_name;
    sensor_msgs::ImagePtr msg;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

};

int main(int argc, char** argv){

    ros::init(argc, argv, "camera_image");
    groupName = argv[1];
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);

    ROS_INFO("%s", groupName.c_str());
    CameraImage cimage(cameraMatrix, distCoeffs);

    if(debug) ROS_INFO("Start publishing");

    cimage.sendImage();

    if(debug) ROS_INFO("Publishing done");

    return 0;
}

// 구현부

void CameraImage::sendImage(){
    ros::Rate loop_rate(frequency);
        cv::Mat frame;
        cv::Mat temp;
    while(nh_.ok()){
        cap >> frame;
        if(!frame.empty()){

            if(sizeup) cv::resize(frame, frame, cv::Size(frame.cols * 2, frame.rows * 2), 0, 0, CV_INTER_NN);
            if(calibration) {
                cv::undistort(frame, temp, cameraMatrix, distCoeffs);
                frame = temp;
            }
            if(debug) cv::imshow(OPENCV_WINDOW,frame);

            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            camera_image_pub_.publish(msg);
        }
        int ckey = cv::waitKey(1);
        if(ckey == 27)break;
        loop_rate.sleep();
    }
}

// std::string CameraImage::createTopicName(){
//     topic_name =  "/"+ groupName +"/raw_image";
// }

// template < typename T >
// std::string to_string( const T& n )
// {
//   stm << n ;~
//     std::ostringstream stm ;
//     return stm.str() ;
// }
void CameraImage::initParam(){

  nh_.param("/"+groupName+"/camera_image/camera_num", camera_num, 0);
  nh_.param("/"+groupName+"/camera_image/FREQUENCY", frequency, 30);
  nh_.param("/"+groupName+"/camera_image/debug", debug, 0);
  nh_.param("/"+groupName+"/camera_image/calibration", calibration, 0);
  nh_.param("/"+groupName+"/camera_image/sizeup", sizeup, 0);
  ROS_INFO("camera Image : %d %d %d %d %d", camera_num, frequency, debug, calibration, sizeup);
}
