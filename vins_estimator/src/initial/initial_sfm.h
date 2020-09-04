#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <cstdlib>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "../utils/EigenTypes.h"

using namespace Eigen;
using namespace std;

struct SFMFeature {
  bool state; //状态（是否被三角化）
  int id;
  Eigen::aligned_vector<pair<int, Vector2d>>
      observation; // //所有观测到该特征点的图像帧ID和图像坐标
  double position[3];
  double depth;
};

struct ReprojectionError3D {
  ReprojectionError3D(double observed_u, double observed_v)
      : observed_u(observed_u), observed_v(observed_v) {}

  template <typename T>
  bool operator()(const T *const camera_R, const T *const camera_T,
                  const T *point, T *residuals) const {
    T p[3];
    ceres::QuaternionRotatePoint(camera_R, point, p);
    p[0] += camera_T[0];
    p[1] += camera_T[1];
    p[2] += camera_T[2];
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];
    residuals[0] = xp - T(observed_u);
    residuals[1] = yp - T(observed_v);
    return true;
  }

  static ceres::CostFunction *Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError3D, 2, 4, 3, 3>(
        new ReprojectionError3D(observed_x, observed_y)));
  }

  double observed_u;
  double observed_v;
};

class GlobalSFM {
public:
  GlobalSFM() = default;

  bool construct(int frame_num, Quaterniond *q, Vector3d *T, int l,
                 const Matrix3d relative_R, const Vector3d relative_T,
                 Eigen::aligned_vector<SFMFeature> &sfm_f,
                 Eigen::aligned_map<int, Vector3d> &sfm_tracked_points);

private:
  bool solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i,
                       Eigen::aligned_vector<SFMFeature> &sfm_f);

  void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                        Eigen::Matrix<double, 3, 4> &Pose1, Vector2d &point0,
                        Vector2d &point1, Vector3d &point_3d);

  void triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
                            int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
                            Eigen::aligned_vector<SFMFeature> &sfm_f);

  int feature_num;
};