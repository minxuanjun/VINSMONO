#pragma once

#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <ros/ros.h>

#include "../factor/imu_factor.h"
#include "../feature_manager.h"
#include "../frontend/frontend_data.h"
#include "../utility/Twist.h"
#include "../utility/utility.h"
#include "../utils/EigenTypes.h"

using namespace Eigen;
using namespace std;

class ImageFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImageFrame() = default;
  ImageFrame(const vins::FeatureTrackerResulst &_points, double _t)
      : timestamp{_t}, is_key_frame{false} {
    points = _points;
  };

  vins::FeatureTrackerResulst points;
  IntegrationBase *pre_integration = nullptr;
  Transformd Twi;
  double timestamp;
  bool is_key_frame = false;
};

bool VisualIMUAlignment(Eigen::aligned_map<double, ImageFrame> &all_image_frame,
                        Vector3d *Bgs, Vector3d &g, VectorXd &x);