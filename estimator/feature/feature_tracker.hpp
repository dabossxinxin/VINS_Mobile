
#ifndef feature_tracker_hpp
#define feature_tracker_hpp

#include <string>
#include <list>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "utility.hpp"
#include "global_param.hpp"
#include "vins_pnp.hpp"

#define MAX_CNT 120
#define MIN_DIST 15
#define COL 480
#define ROW 640
#define F_THRESHOLD 1.0
#define EQUALIZE 1

using namespace cv;
using namespace std;
using namespace Eigen;

// image frame
// --------> x:480
// |
// |
// |
// |
// |
// | y:640
struct max_min_pts
{
    Point2f min;
    Point2f max;
};

struct IMU_MSG_LOCAL
{
    double header;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
};

class FeatureTracker
{
public:
    FeatureTracker();
    bool solveVinsPnP(double header, Eigen::Vector3d &P, Eigen::Matrix3d &R, bool vins_normal);
    void readImage(const cv::Mat &_img, int _frame_cnt, std::vector<Point2f> &good_pts, std::vector<double> &track_len,
                   double header, Eigen::Vector3d &P, Eigen::Matrix3d &R, bool vins_normal);
    void setMask();
    void rejectWithF();
    void addPoints();
    bool updateID(unsigned int);
    
    cv::Mat mask;
    cv::Mat cur_img;
    cv::Mat pre_img;
    cv::Mat forw_img;
    
    std::vector<cv::Point2f> n_pts;
    std::vector<cv::Point2f> cur_pts;
    std::vector<cv::Point2f> pre_pts;
    std::vector<cv::Point2f> forw_pts;
    
    std::vector<int> ids;
    std::vector<int> track_cnt;
    
    std::vector<max_min_pts> parallax_cnt;
    
    int             frame_cnt;
    static int      n_id;
    int             img_cnt;
    double          current_time;
    vinsPnP         vins_pnp;
    bool            use_pnp;
    bool            update_finished;
    
    std::map<int, Vector3d>     image_msg;
    std::list<IMG_MSG_LOCAL>    solved_features;;
    std::vector<IMU_MSG_LOCAL>  imu_msgs;
    
    VINS_RESULT solved_vins;
};
#endif
