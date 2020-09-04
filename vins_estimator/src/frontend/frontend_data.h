//
// Created by ubuntu on 2020/9/4.
//

#ifndef FRONTEND_DATA_H
#define FRONTEND_DATA_H

#include <memory>
#include "../utils/EigenTypes.h"
#include <opencv2/core/core.hpp>

namespace vins {
using FeatureID = int;
using TimeFrameId = double;
using FeatureTrackerResulst = Eigen::aligned_map<
    int, Eigen::aligned_vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>;

struct FrontEndResult {
  typedef std::shared_ptr<FrontEndResult> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // optical flow result
  double timestamp;
  FeatureTrackerResulst feature;
  cv::Mat image;
};

} // namespace vins
#endif // FRONTEND_DATA_H
