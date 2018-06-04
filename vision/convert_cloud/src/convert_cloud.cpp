
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include "opencv2/opencv.hpp"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <string>

static int debug;
static std::string groupName;

class ConvertCloud{
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Subscriber sub_scan;

public:
    ConvertCloud(){
        initParam();
        sub_ = nh_.subscribe("/"+groupName+"/dist",100,&ConvertCloud::distCb,this);
        sub_scan = nh_.subscribe("/"+groupName+"/scan",100,&ConvertCloud::laserCb,this);
        // sub_scan = nh_.subscribe("/scan",100,&ConvertCloud::laserCb,this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/"+groupName+"/point_cloud",100);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ref_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pc = ref_pc;
    }
  void distCb(const std_msgs::Float32MultiArray::ConstPtr& distData);
  void laserCb(const sensor_msgs::LaserScan input);
  void parseVec();
  void convert();
  void initParam();
  ros::NodeHandle getNh();
  ros::Publisher getPub();
  sensor_msgs::PointCloud2 getPc_out();

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc;
  sensor_msgs::PointCloud2 pc_out;
  std::vector<cv::Vec3f> distVec;
  std::vector<float> distXdata;
  std::vector<float> distYdata;
  int size;


};

int main(int argc, char** argv){
  ros::init(argc,argv,"convert_cloud");
  groupName = argv[1];

  ConvertCloud cvtCloud;
  ros::NodeHandle nh = cvtCloud.getNh();
  ros::Publisher pub = cvtCloud.getPub();
  bool is_cross_walk = false;

  while(nh.ok()){

    if(ros::param::get("hl_controller/crosswalk",is_cross_walk)){
      //차량정지 및 파라미터값 복구 작성 예정
    }
    cvtCloud.getPc_out().header.stamp = ros::Time::now();
    pub.publish(cvtCloud.getPc_out());
    ros::spinOnce();
  }

  return 0;
}
void ConvertCloud::laserCb(const sensor_msgs::LaserScan input){

}
void ConvertCloud::distCb(const std_msgs::Float32MultiArray::ConstPtr& distData){

    if(debug) std::cout<<"start call back"<<std::endl;

    distXdata.clear();
    distYdata.clear();
    distXdata.resize(0);
    distYdata.resize(0);

    std::vector<float>::const_iterator it;

    if(debug) std::cout<<"before push_back"<<std::endl;

    it = distData->data.begin();

    size = (int)(*it);
    ++it;

    while(it != distData->data.end()){
        try{
            distXdata.push_back(*it);
            ++it;

            distYdata.push_back(*it);
            ++it;
        }
        catch(std::exception& e){
            break;
        }
    }
//    ROS_INFO("size : %u",size);
    if(debug) {
        std::cout<<"size : "<<distVec.size()<<std::endl;
        for(int i=0; i<distXdata.size(); i++){
            std::cout<<"X : "<<distXdata[i]<<" / Y : "<<distYdata[i]<<std::endl;
        }
    }

    parseVec();
    convert();
}


void ConvertCloud::parseVec(){
    distVec.clear();
    std::vector<float>::iterator itX = distXdata.begin();
    std::vector<float>::iterator itY = distYdata.begin();
    while( (itX!=distYdata.end())&&(itY!=distYdata.end()) ){
        if(debug){
           std::cout<<"parse X : "<<(*itX)<<" / parse Y : "<<(*itY)<<std::endl;
        }
        distVec.push_back(cv::Vec3f( (*itX),(*itY),0) );
        ++itX;
        ++itY;
    }
    if(debug){
        std::cout<<"parse done"<<std::endl;
    }
}


// 클라우드 변환
void ConvertCloud::convert(){
  pc->clear();

  // Create header
  // std::string frame_id(groupName+"/camera_main");
  // added by Park
  std::string frame_id("/camera_main");
  // std::string frame_id("/camera_main");

  pc->header.frame_id = frame_id;
  pc->header.seq = ros::Time::now().toNSec()/1e3;
    //ros::Time start = ros::Time::now();

      // pc->header.stamp = start;
  //pc->header.stamp = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ>::iterator pc_iter = pc->begin();
  std::vector<cv::Vec3f>::iterator data_iter = distVec.begin();

  int count = 0;
  while(data_iter != distVec.end()){
    pcl::PointXYZ inputPoint((*data_iter)[0],(*data_iter)[1],(*data_iter)[2]);
    //pc_iter = pc->insert(pc_iter, inputPoint);
    (*pc).points.push_back(inputPoint);
    ++data_iter;
  }

  pcl::toROSMsg(*pc, pc_out);
  if(debug) std::cout<<"make cloud done"<<std::endl;

  pc->clear();


}

ros::NodeHandle ConvertCloud::getNh(){ return nh_; }
ros::Publisher ConvertCloud::getPub(){ return pub_; }
sensor_msgs::PointCloud2 ConvertCloud::getPc_out(){ return pc_out; }

void ConvertCloud::initParam(){
  nh_.param("/"+groupName+"/convert_cloud/debug", debug, 4);
}
