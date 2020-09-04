#include "estimator.h"
#include "utils/EigenTypes.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

Estimator::Estimator() : f_manager{Rs} {
  LOG(INFO) << "init begins";
  // 向 feature_manager 传递frame 管理器的指针
  f_manager.int_frameid2_time_frameid_ = &int_frameid2_time_frameid;
  f_manager.time_frameid2_int_frameid_ = &time_frameid2_int_frameid;
  f_manager.local_active_frames_ = &local_active_frames;
  clearState();
}

void Estimator::setParameter() {
  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
  }
  f_manager.setRic(ric);
  ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  td = TD;
}

void Estimator::clearState() {
  local_active_frames.clear();
  int_frameid2_time_frameid.clear();
  time_frameid2_int_frameid.clear();

  global_frame_cnt = 0;

  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    Rs[i].setIdentity();
    Ps[i].setZero();
    Vs[i].setZero();
    Bas[i].setZero();
    Bgs[i].setZero();
    dt_buf[i].clear();
    linear_acceleration_buf[i].clear();
    angular_velocity_buf[i].clear();

    if (pre_integrations[i] != nullptr)
      delete pre_integrations[i];
    pre_integrations[i] = nullptr;
  }

  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] = Vector3d::Zero();
    ric[i] = Matrix3d::Identity();
  }

  for (auto &it : localWindowFrames) {
    if (it.second.pre_integration != nullptr) {
      delete it.second.pre_integration;
      it.second.pre_integration = nullptr;
    }
  }

  solver_flag = SolverFlag::INITIAL;
  first_imu = false;
  sum_of_back = 0;
  sum_of_front = 0;
  frame_count = 0;

  initial_timestamp = 0;
  localWindowFrames.clear();
  td = TD;

  if (tmp_pre_integration != nullptr)
    delete tmp_pre_integration;
  if (last_marginalization_info != nullptr)
    delete last_marginalization_info;

  tmp_pre_integration = nullptr;
  last_marginalization_info = nullptr;
  last_marginalization_parameter_blocks.clear();

  f_manager.clearState();

  failure_occur = 0;
  relocalization_info = 0;

  drift_correct_r = Matrix3d::Identity();
  drift_correct_t = Vector3d::Zero();
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration,
                           const Vector3d &angular_velocity) {
  if (!first_imu) {
    first_imu = true;
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
  }

  if (!pre_integrations[frame_count]) {
    pre_integrations[frame_count] =
        new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
  }
  if (frame_count != 0) {
    pre_integrations[frame_count]->push_back(dt, linear_acceleration,
                                             angular_velocity);
    // if(solver_flag != NON_LINEAR)
    tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

    dt_buf[frame_count].push_back(dt);
    linear_acceleration_buf[frame_count].push_back(linear_acceleration);
    angular_velocity_buf[frame_count].push_back(angular_velocity);

    int j = frame_count;
    Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
    Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
    Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
    Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
    Vs[j] += dt * un_acc;
  }
  acc_0 = linear_acceleration;
  gyr_0 = angular_velocity;
}

void Estimator::processImage(const vins::FeatureTrackerResulst &image,
                             const std_msgs::Header &header) {
  ROS_DEBUG("new image coming ------------------------------------------");
  ROS_DEBUG("Adding feature points %lu", image.size());
  local_active_frames.insert(header.stamp.toSec());
  recomputeFrameId(); // 梳理局部滑窗中frame的id
  LOG(INFO) << "local_active_frames.size: " << local_active_frames.size();
  if (f_manager.addFeatureCheckParallax(frame_count, header.stamp.toSec(),
                                        image, td))
    marginalization_flag = MarginalizationFlag::MARGIN_OLD;
  else
    marginalization_flag = MarginalizationFlag::MARGIN_SECOND_NEW;

  ROS_DEBUG("this frame is--------------------%s",
            marginalization_flag == MarginalizationFlag::MARGIN_OLD ? "reject"
                                                                    : "accept");
  ROS_DEBUG("%s", marginalization_flag == MarginalizationFlag::MARGIN_OLD
                      ? "Non-keyframe"
                      : "Keyframe");
  ROS_DEBUG("Solving %d", frame_count);
  ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());

  Headers[frame_count] = header;

  ImageFrame imageframe(image, header.stamp.toSec());
  imageframe.pre_integration = tmp_pre_integration;
  localWindowFrames.insert(make_pair(header.stamp.toSec(), imageframe));
  tmp_pre_integration =
      new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

  if (ESTIMATE_EXTRINSIC == 2) {
    ROS_INFO("calibrating extrinsic param, rotation movement is needed");
    if (frame_count != 0) {
      Eigen::aligned_vector<pair<Vector3d, Vector3d>> corres =
          f_manager.getCorresponding(frame_count - 1, frame_count);
      Matrix3d calib_ric;
      if (initial_ex_rotation.CalibrationExRotation(
              corres, pre_integrations[frame_count]->delta_q, calib_ric)) {
        ROS_WARN("initial extrinsic rotation calib success");
        ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
        ric[0] = calib_ric;
        RIC[0] = calib_ric;
        ESTIMATE_EXTRINSIC = 1;
      }
    }
  }

  if (solver_flag == SolverFlag::INITIAL) {
    if (frame_count == WINDOW_SIZE) {
      bool result = false;
      if (ESTIMATE_EXTRINSIC != 2 &&
          (header.stamp.toSec() - initial_timestamp) > 0.1) {
        result = initialStructure();
        initial_timestamp = header.stamp.toSec();
      }
      if (result) {
        solver_flag = SolverFlag::NON_LINEAR;
        solveOdometry();
        slideWindow();
        f_manager.removeFailures();
        ROS_INFO("Initialization finish!");
        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];

      } else
        slideWindow();
    } else
      frame_count++;
  } else {
    TicToc t_solve;
    solveOdometry();
    ROS_DEBUG("solver costs: %fms", t_solve.toc());

    if (failureDetection()) {
      ROS_WARN("failure detection!");
      failure_occur = 1;
      clearState();
      setParameter();
      ROS_WARN("system reboot!");
      return;
    }

    TicToc t_margin;
    slideWindow();
    f_manager.removeFailures();
    ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
    // prepare output of VINS
    key_poses.clear();
    for (int i = 0; i <= WINDOW_SIZE; i++)
      key_poses.push_back(Ps[i]);

    last_R = Rs[WINDOW_SIZE];
    last_P = Ps[WINDOW_SIZE];
    last_R0 = Rs[0];
    last_P0 = Ps[0];
  }
  recomputeFrameId();
}

void Estimator::recomputeFrameId() {

  int_frameid2_time_frameid.clear();
  time_frameid2_int_frameid.clear();

  int localwindow_id = 0;
  std::string output;
  for (const auto tid : local_active_frames) {
    int_frameid2_time_frameid[localwindow_id] = tid;
    time_frameid2_int_frameid[tid] = localwindow_id;
    output +=
        std::to_string(localwindow_id) + " ->" + std::to_string(tid) + "\n";

    localwindow_id++;
  }
  // LOG(INFO) << "WINDOW Frame: \n" << output;

} // recomputeFrameId

bool Estimator::initialStructure() {
  TicToc t_sfm;
  // check imu observibility
  {
    Eigen::aligned_map<double, ImageFrame>::iterator frame_it;
    Vector3d sum_g;
    for (frame_it = localWindowFrames.begin(), frame_it++;
         frame_it != localWindowFrames.end(); frame_it++) {
      double dt = frame_it->second.pre_integration->sum_dt;
      Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
      sum_g += tmp_g;
    }
    Vector3d aver_g;
    aver_g = sum_g * 1.0 / ((int)localWindowFrames.size() - 1);
    double var = 0;
    for (frame_it = localWindowFrames.begin(), frame_it++;
         frame_it != localWindowFrames.end(); frame_it++) {
      double dt = frame_it->second.pre_integration->sum_dt;
      Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
      var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
      // cout << "frame g " << tmp_g.transpose() << endl;
    }
    var = sqrt(var / ((int)localWindowFrames.size() - 1));
    // ROS_WARN("IMU variation %f!", var);
    if (var < 0.25) {
      ROS_INFO("IMU excitation not enouth!");
      // return false;
    }
  }
  // global sfm
  Quaterniond Q[frame_count + 1];
  Vector3d T[frame_count + 1];
  Eigen::aligned_map<int, Vector3d> sfm_tracked_points;
  Eigen::aligned_vector<SFMFeature> sfm_f;
  for (auto &landmark : f_manager.KeyPointLandmarks) {
    SFMFeature tmp_feature;
    tmp_feature.state = false;
    tmp_feature.id = landmark.second.feature_id;
    for (const auto &obser_per_frame : landmark.second.obs) {
      auto frame_intid = time_frameid2_int_frameid.at(obser_per_frame.first);
      Vector3d pts_j = obser_per_frame.second.normalpoint;
      tmp_feature.observation.push_back(
          make_pair(frame_intid, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
    }
    sfm_f.emplace_back(tmp_feature);
  }
  Matrix3d relative_R;
  Vector3d relative_T;
  int l;
  //保证具有足够的视差,由F矩阵恢复Rt
  //第l帧是从第一帧开始到滑动窗口中第一个满足与当前帧的平均视差足够大的帧，会作为参考帧到下面的全局sfm使用
  //此处的relative_R，relative_T为当前帧到参考帧（第l帧）的坐标系变换Rt

  if (!relativePose(relative_R, relative_T, l)) {
    ROS_INFO("Not enough features or parallax; Move device around");
    return false;
  }
  GlobalSFM sfm;
  if (!sfm.construct(frame_count + 1, Q, T, l, relative_R, relative_T, sfm_f,
                     sfm_tracked_points)) {
    ROS_DEBUG("global SFM failed!");
    marginalization_flag = MarginalizationFlag::MARGIN_OLD;
    return false;
  }

  // solve pnp for all frame
  Eigen::aligned_map<double, ImageFrame>::iterator frame_it;
  Eigen::aligned_map<int, Vector3d>::iterator it;
  frame_it = localWindowFrames.begin();
  for (int i = 0; frame_it != localWindowFrames.end(); frame_it++) {
    // provide initial guess
    cv::Mat r, rvec, t, D, tmp_r;
    if ((frame_it->first) == Headers[i].stamp.toSec()) {
      frame_it->second.is_key_frame = true;
      frame_it->second.Twi = frame_it->second.Twi =
          Transformd(Q[i].toRotationMatrix() * RIC[0].transpose(), T[i]);
      ;

      i++;
      continue;
    }
    if ((frame_it->first) > Headers[i].stamp.toSec()) {
      i++;
    }
    Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
    Vector3d P_inital = -R_inital * T[i];
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    frame_it->second.is_key_frame = false;
    vector<cv::Point3f> pts_3_vector;
    vector<cv::Point2f> pts_2_vector;
    for (auto &id_pts : frame_it->second.points) {
      int feature_id = id_pts.first;
      for (auto &i_p : id_pts.second) {
        it = sfm_tracked_points.find(feature_id);
        if (it != sfm_tracked_points.end()) {
          Vector3d world_pts = it->second;
          cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
          pts_3_vector.push_back(pts_3);
          Vector2d img_pts = i_p.second.head<2>();
          cv::Point2f pts_2(img_pts(0), img_pts(1));
          pts_2_vector.push_back(pts_2);
        }
      }
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    if (pts_3_vector.size() < 6) {
      cout << "pts_3_vector size " << pts_3_vector.size() << endl;
      ROS_DEBUG("Not enough points for solve pnp !");
      return false;
    }
    if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
      ROS_DEBUG("solve pnp fail!");
      return false;
    }
    cv::Rodrigues(rvec, r);
    MatrixXd R_pnp, tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);
    R_pnp = tmp_R_pnp.transpose();
    MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    T_pnp = R_pnp * (-T_pnp);

    frame_it->second.Twi = Transformd(R_pnp * RIC[0].transpose(), T_pnp);
  }
  if (visualInitialAlign())
    return true;
  else {
    ROS_INFO("misalign visual structure with IMU");
    return false;
  }
}

bool Estimator::visualInitialAlign() {
  TicToc t_g;
  VectorXd x;
  // solve scale
  bool result = VisualIMUAlignment(localWindowFrames, Bgs, g, x);
  if (!result) {
    ROS_DEBUG("solve g failed!");
    return false;
  }

  // change state
  for (int i = 0; i <= frame_count; i++) {
    Matrix3d Ri =
        localWindowFrames[Headers[i].stamp.toSec()].Twi.rotationMatrix();
    ;
    Vector3d Pi = localWindowFrames[Headers[i].stamp.toSec()].Twi.pos;
    Ps[i] = Pi;
    Rs[i] = Ri;
    localWindowFrames[Headers[i].stamp.toSec()].is_key_frame = true;
  }

  f_manager.resetDepth();

  // triangulat on cam pose , no tic
  Vector3d TIC_TMP[NUM_OF_CAM];
  for (int i = 0; i < NUM_OF_CAM; i++)
    TIC_TMP[i].setZero();
  ric[0] = RIC[0];

  f_manager.setRic(ric);
  f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

  double s = (x.tail<1>())(0);
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
  }
  for (int i = frame_count; i >= 0; i--)
    Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
  int kv = -1;
  Eigen::aligned_map<double, ImageFrame>::iterator frame_i;
  for (frame_i = localWindowFrames.begin(); frame_i != localWindowFrames.end();
       frame_i++) {
    if (frame_i->second.is_key_frame) {
      kv++;
      Vs[kv] = frame_i->second.Twi.rot * x.segment<3>(kv * 3);
    }
  }

  for (auto &landmark : f_manager.KeyPointLandmarks) {
    landmark.second.used_num = landmark.second.obs.size();
    if (!(landmark.second.used_num >= 2 &&
          time_frameid2_int_frameid.at(landmark.second.kf_id) <
              WINDOW_SIZE - 2))
      continue;
    landmark.second.estimated_depth *= s;
  }

  Matrix3d R0 = Utility::g2R(g);
  double yaw = Utility::R2ypr(R0 * Rs[0]).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  g = R0 * g;
  // Matrix3d rot_diff = R0 * Rs[0].transpose();
  Matrix3d rot_diff = R0;
  for (int i = 0; i <= frame_count; i++) {
    Ps[i] = rot_diff * Ps[i];
    Rs[i] = rot_diff * Rs[i];
    Vs[i] = rot_diff * Vs[i];
  }
  ROS_DEBUG_STREAM("g0     " << g.transpose());
  ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

  return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T,
                             int &l) {
  // find previous frame which contians enough correspondance and parallex with
  // newest frame
  for (int i = 0; i < WINDOW_SIZE; i++) {
    Eigen::aligned_vector<pair<Vector3d, Vector3d>> corres;
    corres = f_manager.getCorresponding(i, WINDOW_SIZE);
    if (corres.size() > 20) {
      double sum_parallax = 0;
      double average_parallax;
      for (int j = 0; j < int(corres.size()); j++) {
        Vector2d pts_0(corres[j].first(0), corres[j].first(1));
        Vector2d pts_1(corres[j].second(0), corres[j].second(1));
        double parallax = (pts_0 - pts_1).norm();
        sum_parallax = sum_parallax + parallax;
      }
      average_parallax = 1.0 * sum_parallax / int(corres.size());
      if (average_parallax * 460 > 30 &&
          m_estimator.solveRelativeRT(corres, relative_R, relative_T)) {
        l = i;
        ROS_DEBUG("average_parallax %f choose l %d and newest frame to "
                  "triangulate the whole structure",
                  average_parallax * 460, l);
        return true;
      }
    }
  }
  return false;
}

void Estimator::solveOdometry() {
  if (frame_count < WINDOW_SIZE)
    return;
  if (solver_flag == SolverFlag::NON_LINEAR) {
    TicToc t_tri;
    f_manager.triangulate(Ps, tic, ric);
    ROS_DEBUG("triangulation costs %f", t_tri.toc());
    optimization();
    updateFramePose();
  }
}

void Estimator::vector2double() {
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    para_Pose[i][0] = Ps[i].x();
    para_Pose[i][1] = Ps[i].y();
    para_Pose[i][2] = Ps[i].z();
    Quaterniond q{Rs[i]};
    para_Pose[i][3] = q.x();
    para_Pose[i][4] = q.y();
    para_Pose[i][5] = q.z();
    para_Pose[i][6] = q.w();

    para_SpeedBias[i][0] = Vs[i].x();
    para_SpeedBias[i][1] = Vs[i].y();
    para_SpeedBias[i][2] = Vs[i].z();

    para_SpeedBias[i][3] = Bas[i].x();
    para_SpeedBias[i][4] = Bas[i].y();
    para_SpeedBias[i][5] = Bas[i].z();

    para_SpeedBias[i][6] = Bgs[i].x();
    para_SpeedBias[i][7] = Bgs[i].y();
    para_SpeedBias[i][8] = Bgs[i].z();
  }
  for (int i = 0; i < NUM_OF_CAM; i++) {
    para_Ex_Pose[i][0] = tic[i].x();
    para_Ex_Pose[i][1] = tic[i].y();
    para_Ex_Pose[i][2] = tic[i].z();
    Quaterniond q{ric[i]};
    para_Ex_Pose[i][3] = q.x();
    para_Ex_Pose[i][4] = q.y();
    para_Ex_Pose[i][5] = q.z();
    para_Ex_Pose[i][6] = q.w();
  }

  // TODO: 3D点更新
  f_manager.depth2InvDepth();

  if (ESTIMATE_TD)
    para_Td[0][0] = td;
}

void Estimator::double2vector() {
  Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
  Vector3d origin_P0 = Ps[0];

  if (failure_occur) {
    origin_R0 = Utility::R2ypr(last_R0);
    origin_P0 = last_P0;
    failure_occur = 0;
  }
  Vector3d origin_R00 =
      Utility::R2ypr(Quaterniond(para_Pose[0][6], para_Pose[0][3],
                                 para_Pose[0][4], para_Pose[0][5])
                         .toRotationMatrix());
  double y_diff = origin_R0.x() - origin_R00.x();
  // TODO
  Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
  if (abs(abs(origin_R0.y()) - 90) < 1.0 ||
      abs(abs(origin_R00.y()) - 90) < 1.0) {
    ROS_DEBUG("euler singular point!");
    rot_diff = Rs[0] * Quaterniond(para_Pose[0][6], para_Pose[0][3],
                                   para_Pose[0][4], para_Pose[0][5])
                           .toRotationMatrix()
                           .transpose();
  }

  for (int i = 0; i <= WINDOW_SIZE; i++) {

    Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3],
                                   para_Pose[i][4], para_Pose[i][5])
                           .normalized()
                           .toRotationMatrix();

    Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                para_Pose[i][1] - para_Pose[0][1],
                                para_Pose[i][2] - para_Pose[0][2]) +
            origin_P0;

    Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0], para_SpeedBias[i][1],
                                para_SpeedBias[i][2]);

    Bas[i] = Vector3d(para_SpeedBias[i][3], para_SpeedBias[i][4],
                      para_SpeedBias[i][5]);

    Bgs[i] = Vector3d(para_SpeedBias[i][6], para_SpeedBias[i][7],
                      para_SpeedBias[i][8]);
  }

  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] =
        Vector3d(para_Ex_Pose[i][0], para_Ex_Pose[i][1], para_Ex_Pose[i][2]);
    ric[i] = Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3],
                         para_Ex_Pose[i][4], para_Ex_Pose[i][5])
                 .toRotationMatrix();
  }

  // TODO: 3D 点更新
  // ------------------
  f_manager.invDepth2Depth();
  //-------------------

  if (ESTIMATE_TD)
    td = para_Td[0][0];

  // relative info between two loop frame
  if (relocalization_info) {
    Matrix3d relo_r;
    Vector3d relo_t;
    relo_r = rot_diff *
             Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5])
                 .normalized()
                 .toRotationMatrix();
    relo_t = rot_diff * Vector3d(relo_Pose[0] - para_Pose[0][0],
                                 relo_Pose[1] - para_Pose[0][1],
                                 relo_Pose[2] - para_Pose[0][2]) +
             origin_P0;
    double drift_correct_yaw;
    drift_correct_yaw =
        Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
    drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
    drift_correct_t = prev_relo_t - drift_correct_r * relo_t;
    relo_relative_t =
        relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
    relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
    relo_relative_yaw =
        Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() -
                                Utility::R2ypr(relo_r).x());
    // cout << "vins relo " << endl;
    // cout << "vins relative_t " << relo_relative_t.transpose() << endl;
    // cout << "vins relative_yaw " <<relo_relative_yaw << endl;
    relocalization_info = 0;
  }
}

void Estimator::updateFramePose() {
  for (size_t i = 0; i < WINDOW_SIZE + 1; i++) {
    auto timestamp = Headers[i].stamp.toSec();
    if (localWindowFrames.find(timestamp) != localWindowFrames.end()) {
      localWindowFrames[timestamp].Twi.rot = Rs[i];
      localWindowFrames[timestamp].Twi.pos = Ps[i];
    }
  }
}

bool Estimator::failureDetection() {
  if (f_manager.last_track_num < 2) {
    ROS_INFO(" little feature %d", f_manager.last_track_num);
    // return true;
  }
  if (Bas[WINDOW_SIZE].norm() > 2.5) {
    ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
    return true;
  }
  if (Bgs[WINDOW_SIZE].norm() > 1.0) {
    ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
    return true;
  }
  /*
  if (tic(0) > 1)
  {
      ROS_INFO(" big extri param estimation %d", tic(0) > 1);
      return true;
  }
  */
  Vector3d tmp_P = Ps[WINDOW_SIZE];
  if ((tmp_P - last_P).norm() > 5) {
    ROS_INFO(" big translation");
    return true;
  }
  if (abs(tmp_P.z() - last_P.z()) > 1) {
    ROS_INFO(" big z translation");
    return true;
  }
  Matrix3d tmp_R = Rs[WINDOW_SIZE];
  Matrix3d delta_R = tmp_R.transpose() * last_R;
  Quaterniond delta_Q(delta_R);
  double delta_angle;
  delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
  if (delta_angle > 50) {
    ROS_INFO(" big delta_angle ");
    // return true;
  }
  return false;
}

void Estimator::optimization() {
  ceres::Problem problem;
  ceres::LossFunction *loss_function;
  // loss_function = new ceres::HuberLoss(1.0);
  loss_function = new ceres::CauchyLoss(1.0);
  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    ceres::LocalParameterization *local_parameterization =
        new PoseLocalParameterization();
    problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
    problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
  }
  for (int i = 0; i < NUM_OF_CAM; i++) {
    ceres::LocalParameterization *local_parameterization =
        new PoseLocalParameterization();
    problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE,
                              local_parameterization);
    if (!ESTIMATE_EXTRINSIC) {
      ROS_DEBUG("fix extinsic param");
      problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    } else
      ROS_DEBUG("estimate extinsic param");
  }
  if (ESTIMATE_TD) {
    problem.AddParameterBlock(para_Td[0], 1);
    // problem.SetParameterBlockConstant(para_Td[0]);
  }

  TicToc t_whole, t_prepare;
  vector2double();

  if (last_marginalization_info) {
    // construct new marginlization_factor
    MarginalizationFactor *marginalization_factor =
        new MarginalizationFactor(last_marginalization_info);
    problem.AddResidualBlock(marginalization_factor, NULL,
                             last_marginalization_parameter_blocks);
  }

  for (int i = 0; i < WINDOW_SIZE; i++) {
    int j = i + 1;
    if (pre_integrations[j]->sum_dt > 10.0)
      continue;
    IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
    problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i],
                             para_Pose[j], para_SpeedBias[j]);
  }

  int f_m_cnt = 0;

  // TODO: 使用 unordered map 代替　list

  for (auto &landmark : f_manager.KeyPointLandmarks) {
    landmark.second.used_num = landmark.second.obs.size();
    if (!(landmark.second.used_num >= 2 &&
          time_frameid2_int_frameid.at(landmark.second.kf_id) <
              WINDOW_SIZE - 2))
      continue;

    auto host_tid = landmark.second.kf_id; // 第一个观测值对应的就是主导真
    int host_id = time_frameid2_int_frameid.at(host_tid);
    const auto &obs = landmark.second.obs;
    Vector3d pts_i = obs.at(host_tid).normalpoint;

    for (const auto &it_per_frame : obs) {
      auto target_tid = it_per_frame.first;
      int target_id = time_frameid2_int_frameid.at(target_tid);

      if (host_tid == target_tid)
        continue;

      Vector3d pts_j = it_per_frame.second.normalpoint;
      if (ESTIMATE_TD) {
        ProjectionTdFactor *f_td = new ProjectionTdFactor(
            pts_i, pts_j, obs.at(host_tid).velocity,
            it_per_frame.second.velocity, obs.at(host_tid).cur_td,
            it_per_frame.second.cur_td, obs.at(host_tid).uv.y(),
            it_per_frame.second.uv.y());
        problem.AddResidualBlock(f_td, loss_function, para_Pose[host_id],
                                 para_Pose[target_id], para_Ex_Pose[0],
                                 landmark.second.data.data(), para_Td[0]);

      } else {
        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
        problem.AddResidualBlock(f, loss_function, para_Pose[host_id],
                                 para_Pose[target_id], para_Ex_Pose[0],
                                 landmark.second.data.data());
      }
      f_m_cnt++;
    }
  }
  // TODO: 使用 unordered map 代替　list

  ROS_DEBUG("visual measurement count: %d", f_m_cnt);
  ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

  if (relocalization_info) {
    // printf("set relocalization factor! \n");
    ceres::LocalParameterization *local_parameterization =
        new PoseLocalParameterization();
    problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);
    int retrive_feature_index = 0;
    int feature_index = -1;

    // TODO: 使用 unordered map 代替　list
    for (auto &match_point : match_points) {
      int feature_id = match_point.z();

      // 查看landmarks数据库中能否找到id号相同的landmark
      if (f_manager.KeyPointLandmarks.find(feature_id) !=
          f_manager.KeyPointLandmarks.end()) {
        auto &landmark = f_manager.KeyPointLandmarks.at(feature_id);

        // 确定landmark是否合法
        landmark.used_num = landmark.obs.size();
        if (!(landmark.used_num >= 2 and
            time_frameid2_int_frameid.at(landmark.kf_id) < WINDOW_SIZE - 2)) {
          continue;
        }

        // 如果landmark合法
        auto host_tid = landmark.kf_id;
        int host_id = time_frameid2_int_frameid.at(host_tid);

        Vector3d pts_i = landmark.obs.at(host_tid).normalpoint;
        Vector3d pts_j = match_point;
        pts_j[2] = 1.0;

        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);

        // TODO: 感觉需要把逆深度优化的写入到landmark中
        // -------------------------------------
        problem.AddResidualBlock(f, loss_function, para_Pose[host_id],
                                 relo_Pose, para_Ex_Pose[0],
                                 landmark.data.data());
        // ------------------------------------
      }
    }
  }

  ceres::Solver::Options options;

  options.linear_solver_type = ceres::DENSE_SCHUR;
  // options.num_threads = 2;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_num_iterations = NUM_ITERATIONS;
  // options.use_explicit_schur_complement = true;
  // options.minimizer_progress_to_stdout = true;
  // options.use_nonmonotonic_steps = true;
  if (marginalization_flag == MarginalizationFlag::MARGIN_OLD)
    options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
  else
    options.max_solver_time_in_seconds = SOLVER_TIME;
  TicToc t_solver;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // cout << summary.BriefReport() << endl;
  ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
  ROS_DEBUG("solver costs: %f", t_solver.toc());

  double2vector();

  TicToc t_whole_marginalization;
  if (marginalization_flag == MarginalizationFlag::MARGIN_OLD) {
    MarginalizationInfo *marginalization_info = new MarginalizationInfo();
    vector2double();

    if (last_marginalization_info) {
      vector<int> drop_set;
      for (int i = 0;
           i < static_cast<int>(last_marginalization_parameter_blocks.size());
           i++) {
        if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
            last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
          drop_set.push_back(i);
      }
      // construct new marginlization_factor
      MarginalizationFactor *marginalization_factor =
          new MarginalizationFactor(last_marginalization_info);
      ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
          marginalization_factor, NULL, last_marginalization_parameter_blocks,
          drop_set);

      marginalization_info->addResidualBlockInfo(residual_block_info);
    }

    {
      if (pre_integrations[1]->sum_dt < 10.0) {
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            imu_factor, NULL,
            vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1],
                             para_SpeedBias[1]},
            vector<int>{0, 1});
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }

    {
      // TODO: 使用 unordered map 代替　list
      for (auto &landmark : f_manager.KeyPointLandmarks) {
        landmark.second.used_num = landmark.second.obs.size();
        if (!(landmark.second.used_num >= 2 &&
              time_frameid2_int_frameid.at(landmark.second.kf_id) <
                  WINDOW_SIZE - 2))
          continue;

        auto host_tid = landmark.second.kf_id;
        int host_id = time_frameid2_int_frameid.at(host_tid);

        if (host_id != 0)
          continue;

        const auto &obs = landmark.second.obs;

        Vector3d pts_i = obs.at(host_tid).normalpoint;

        for (const auto &it_per_frame : obs) {

          auto target_tid = it_per_frame.first;
          int target_id = time_frameid2_int_frameid.at(target_tid);

          if (host_tid == target_tid)
            continue;

          Vector3d pts_j = it_per_frame.second.normalpoint;

          if (ESTIMATE_TD) {
            ProjectionTdFactor *f_td = new ProjectionTdFactor(
                pts_i, pts_j, obs.at(host_tid).velocity,
                it_per_frame.second.velocity, obs.at(host_tid).cur_td,
                it_per_frame.second.cur_td, obs.at(host_tid).uv.y(),
                it_per_frame.second.uv.y());

            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                f_td, loss_function,
                vector<double *>{para_Pose[host_id], para_Pose[target_id],
                                 para_Ex_Pose[0], landmark.second.data.data(),
                                 para_Td[0]},
                vector<int>{0, 3});
            marginalization_info->addResidualBlockInfo(residual_block_info);
          } else {
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                f, loss_function,
                vector<double *>{para_Pose[host_id], para_Pose[target_id],
                                 para_Ex_Pose[0], landmark.second.data.data()},
                vector<int>{0, 3});
            marginalization_info->addResidualBlockInfo(residual_block_info);
          }
        }
      }
    }

    TicToc t_pre_margin;
    marginalization_info->preMarginalize();
    ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

    TicToc t_margin;
    marginalization_info->marginalize();
    ROS_DEBUG("marginalization %f ms", t_margin.toc());

    std::unordered_map<long, double *> addr_shift;
    for (int i = 1; i <= WINDOW_SIZE; i++) {
      addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
      addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] =
          para_SpeedBias[i - 1];
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
      addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
    if (ESTIMATE_TD) {
      addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
    }
    vector<double *> parameter_blocks =
        marginalization_info->getParameterBlocks(addr_shift);

    if (last_marginalization_info)
      delete last_marginalization_info;
    last_marginalization_info = marginalization_info;
    last_marginalization_parameter_blocks = parameter_blocks;

  } else {
    if (last_marginalization_info &&
        std::count(std::begin(last_marginalization_parameter_blocks),
                   std::end(last_marginalization_parameter_blocks),
                   para_Pose[WINDOW_SIZE - 1])) {

      MarginalizationInfo *marginalization_info = new MarginalizationInfo();
      vector2double();
      if (last_marginalization_info) {
        vector<int> drop_set;
        for (int i = 0;
             i < static_cast<int>(last_marginalization_parameter_blocks.size());
             i++) {
          ROS_ASSERT(last_marginalization_parameter_blocks[i] !=
                     para_SpeedBias[WINDOW_SIZE - 1]);
          if (last_marginalization_parameter_blocks[i] ==
              para_Pose[WINDOW_SIZE - 1])
            drop_set.push_back(i);
        }
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor =
            new MarginalizationFactor(last_marginalization_info);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            marginalization_factor, NULL, last_marginalization_parameter_blocks,
            drop_set);

        marginalization_info->addResidualBlockInfo(residual_block_info);
      }

      TicToc t_pre_margin;
      ROS_DEBUG("begin marginalization");
      marginalization_info->preMarginalize();
      ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

      TicToc t_margin;
      ROS_DEBUG("begin marginalization");
      marginalization_info->marginalize();
      ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

      std::unordered_map<long, double *> addr_shift;
      for (int i = 0; i <= WINDOW_SIZE; i++) {
        if (i == WINDOW_SIZE - 1)
          continue;
        else if (i == WINDOW_SIZE) {
          addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
          addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] =
              para_SpeedBias[i - 1];
        } else {
          addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
          addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] =
              para_SpeedBias[i];
        }
      }
      for (int i = 0; i < NUM_OF_CAM; i++)
        addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
      if (ESTIMATE_TD) {
        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
      }

      vector<double *> parameter_blocks =
          marginalization_info->getParameterBlocks(addr_shift);
      if (last_marginalization_info)
        delete last_marginalization_info;
      last_marginalization_info = marginalization_info;
      last_marginalization_parameter_blocks = parameter_blocks;
    }
  }
  ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());

  ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void Estimator::slideWindow() {
  TicToc t_margin;
  if (marginalization_flag == MarginalizationFlag::MARGIN_OLD) {

    local_active_frames.erase(int_frameid2_time_frameid[0]);
    slideWindowOld();

    double t_0 = Headers[0].stamp.toSec();

    if (frame_count == WINDOW_SIZE) {
      for (int i = 0; i < WINDOW_SIZE; i++) {
        Rs[i].swap(Rs[i + 1]);

        std::swap(pre_integrations[i], pre_integrations[i + 1]);

        dt_buf[i].swap(dt_buf[i + 1]);
        linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
        angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

        Headers[i] = Headers[i + 1];
        Ps[i].swap(Ps[i + 1]);
        Vs[i].swap(Vs[i + 1]);
        Bas[i].swap(Bas[i + 1]);
        Bgs[i].swap(Bgs[i + 1]);
      }
      Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
      Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
      Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
      Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
      Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
      Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

      delete pre_integrations[WINDOW_SIZE];
      pre_integrations[WINDOW_SIZE] =
          new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

      dt_buf[WINDOW_SIZE].clear();
      linear_acceleration_buf[WINDOW_SIZE].clear();
      angular_velocity_buf[WINDOW_SIZE].clear();

      Eigen::aligned_map<double, ImageFrame>::iterator it_0;
      it_0 = localWindowFrames.find(t_0);
      delete it_0->second.pre_integration;
      it_0->second.pre_integration = nullptr;

      for (auto it = localWindowFrames.begin(); it != it_0; ++it) {
        if (it->second.pre_integration)
          delete it->second.pre_integration;
        it->second.pre_integration = nullptr;
      }

      localWindowFrames.erase(localWindowFrames.begin(), it_0);
      localWindowFrames.erase(t_0);
    }
  } else {
    local_active_frames.erase(int_frameid2_time_frameid[WINDOW_SIZE - 1]);
    slideWindowNew();

    if (frame_count == WINDOW_SIZE) {
      for (size_t i = 0; i < dt_buf[frame_count].size(); i++) {
        double tmp_dt = dt_buf[frame_count][i];
        Vector3d tmp_linear_acceleration =
            linear_acceleration_buf[frame_count][i];
        Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

        pre_integrations[frame_count - 1]->push_back(
            tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

        dt_buf[frame_count - 1].push_back(tmp_dt);
        linear_acceleration_buf[frame_count - 1].push_back(
            tmp_linear_acceleration);
        angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
      }

      Headers[frame_count - 1] = Headers[frame_count];
      Ps[frame_count - 1] = Ps[frame_count];
      Vs[frame_count - 1] = Vs[frame_count];
      Rs[frame_count - 1] = Rs[frame_count];
      Bas[frame_count - 1] = Bas[frame_count];
      Bgs[frame_count - 1] = Bgs[frame_count];

      delete pre_integrations[WINDOW_SIZE];
      pre_integrations[WINDOW_SIZE] =
          new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

      dt_buf[WINDOW_SIZE].clear();
      linear_acceleration_buf[WINDOW_SIZE].clear();
      angular_velocity_buf[WINDOW_SIZE].clear();
    }
  }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew() {
  sum_of_front++;
  // 剔除次新帧的观测
  f_manager.removeOneFrameObservation(
      int_frameid2_time_frameid[WINDOW_SIZE - 1]);
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld() {
  sum_of_back++;

  bool shift_depth = (solver_flag == SolverFlag::NON_LINEAR);
  if (shift_depth) {
    f_manager.removeOneFrameObservationAndShiftDepth(
        int_frameid2_time_frameid[0], Ps, tic, ric);
  } else
    f_manager.removeOneFrameObservation(int_frameid2_time_frameid[0]);
}

void Estimator::setReloFrame(double _frame_stamp, int _frame_index,
                             Eigen::aligned_vector<Vector3d> &_match_points, Vector3d _relo_t,
                             Matrix3d _relo_r) {
  relo_frame_stamp = _frame_stamp;
  relo_frame_index = _frame_index;
  match_points.clear();
  match_points = _match_points;
  prev_relo_t = _relo_t;
  prev_relo_r = _relo_r;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    if (relo_frame_stamp == Headers[i].stamp.toSec()) {
      relo_frame_local_index = i;
      relocalization_info = 1;
      for (int j = 0; j < SIZE_POSE; j++)
        relo_Pose[j] = para_Pose[i][j];
    }
  }
}
