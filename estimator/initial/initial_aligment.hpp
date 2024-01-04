
#ifndef INITIAL_ALIGMENT_HPP
#define INITIAL_ALIGMENT_HPP

#include <stdio.h>
#include <iostream>
#include <map>

#include "imu_factor.h"
#include "utility.hpp"
#include "feature_manager.hpp"

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class ImageFrame
{
public:
    ImageFrame(){};
    ImageFrame(const std::map<int, Eigen::Vector3d>& _points, double _t)
    :points{_points},t{_t},is_key_frame{false} {};
    std::map<int, Vector3d> points;
    double t;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    IntegrationBase *pre_integration;
    bool is_key_frame;
};

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);
#endif
