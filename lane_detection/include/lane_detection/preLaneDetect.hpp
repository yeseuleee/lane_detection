#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
//using namespace std;

namespace lane_detect_algo
{
    
    
    typedef std::vector<cv::Vec2i> vec2i_t;
    typedef std::vector<cv::Vec4i> vec4i_t;
    typedef std::vector<cv::Mat> vec_mat_t;
    typedef std::vector<cv::Point> vec_p_t;

    class CalLane {
      public:
            struct sPoint{
                double x, y;
            };

            struct sLine{
                double mx, my;//x 기울기, y 기울기
                double sx, sy;//직선의 중심점 좌표
            };
            void printVec4i(vec4i_t print_vec_4i);
            void printVec2i(vec2i_t print_vec_2i);
            void printMat(cv::Mat& print_mat);
            void sortLaneVec(vec_p_t &lane_p_vec);
            void storePointVec(cv::Point &lane_point, vec_p_t &lane_p_vec);
            void storeMatVec(cv::Mat lane_mat, vec_mat_t &lane_m_vec);
            void scoreBoard(cv::Mat &b_score_board, cv::Mat b_img);
            cv::Mat addMatUsingIt(vec_mat_t add_mat);
            cv::Mat addMatUsingOR(vec_mat_t add_mat);
            cv::Mat addMatVec(vec_mat_t add_mat_vec);
            void cannyToBImg(cv::Mat& canny_img);
            int cannyToNSample(cv::Mat canny_img);
            cv::Mat outputRANSAC(cv::Mat score_board);
            void BImgtoRANSACSample(cv::Mat canny_img, sPoint samples[]);
            //////********http://blog.daum.net/pg365/62*****//////
            /////*********RANSAC algorithm 출처 *************//////
            void outputPCA(cv::Mat sample, sLine &model);
            void computeModelParameter(sPoint samples[], int no_samples, sLine &model);
            bool findDupSamples(sPoint *samples, int no_samples, sPoint *data);
            void getSamples(sPoint *samples, int no_samples, sPoint *data, int no_data);
            double computeDistance(sLine &line, sPoint &x);
            double modelVerification(sPoint *inliers, int *no_inliers, sLine &estimated_model, sPoint *data, int no_data, double distance_threshold);
            double ransacLineFitting(sPoint *data, int no_data, sLine &model, double distance_threshold);
            //////********http://blog.daum.net/pg365/62*****//////
            /////*********RANSAC algorithm 출처 *************//////
    
            vec2i_t laneLinearEquation(int x_0, int y_0, int x_1, int y_1);
            vec4i_t houghTransform(cv::Mat& b_img);
            cv::Mat myCanny(cv::Mat raw_img);
            void storeFrameForVideoFile(cv::Mat& frame);
            void storeFrameForWebCam(cv::Mat& frame);
            void checkFrame(vec_mat_t frame_vec);
            cv::Mat makeLanePoint(cv::Mat lane_img);
            void drawLane(vec2i_t coordi_vec);
            void detectYHSVcolor(const cv::Mat& src, cv::Mat& dst, double minHue, double maxHue, double minSat, double maxSat, double minVal, double maxVal);
            void detectWHSVcolor(const cv::Mat& src, cv::Mat& dst, double minHue, double maxHue, double minSat, double maxSat, double minVal, double maxVal);
            void detectWhiteLane(cv::Mat src, cv::Mat& dst, int hmin, int hmax, int smin, int smax, int vmin, int vmax, int amin, int amax);
            void birdEyeView(cv::Mat src, cv::Mat& dst);
            void inverseBirdEyeView(cv::Mat src, cv::Mat& dst);
            void makeYProjection(cv::Mat src, cv::Mat& dst, unsigned int* H_result);
            void makeXProjection(cv::Mat src, cv::Mat dst, unsigned int* H_result);
            void medianForXHistogram(unsigned int* H_result, int H_result_size);
            void makeWindow(cv::Mat src, cv::Mat dst, int window_width, int window_offset, int per_lane_checked);
            void slideWindow(cv::Mat src, int window_width, int window_height, int per_lane_in_window);
            bool addMat_imsi(cv::Mat src, int per_lane_in_window);
            void makeContoursLeftLane(cv::Mat src, cv::Mat& dst);
            void crosswalkCheck(cv::Mat);
            void makeContoursRightLane(cv::Mat src, cv::Mat& dst, int* crosswalk);
            void makeContoursRightLane(cv::Mat src, cv::Mat& dst);
            
    };
    
}
// cv::threshold(white_hsv, white_thresh, 180, 255, CV_THRESH_BINARY);
// cv::Mat afterSobel;
// cv::Sobel(white_thresh,afterSobel,CV_32F,1,0);
// callane.detectWHSVcolor(bev, white_hsv, w_hmin, w_hmax, w_smin, w_smax, w_vmin, w_vmax);
// callane.detectWHSVcolor(bev, yellow_hsv, 0, 180, 0, 20, 200, 255);

