#pragma once

#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
#include "../utils/EigenTypes.h"
#include <Eigen/Dense>

using namespace Eigen;

#include <ros/console.h>

class MotionEstimator {
public:
  bool solveRelativeRT(
      const Eigen::aligned_vector<pair<Eigen::Vector3d, Eigen::Vector3d>>
          &corres,
      Eigen::Matrix3d &R, Eigen::Vector3d &T);

private:
  double testTriangulation(const vector<cv::Point2f> &l,
                           const vector<cv::Point2f> &r, cv::Mat_<double> R,
                           cv::Mat_<double> t);
  void decomposeE(cv::Mat E, cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                  cv::Mat_<double> &t1, cv::Mat_<double> &t2);
};
