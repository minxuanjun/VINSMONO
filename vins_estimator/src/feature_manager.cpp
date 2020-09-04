#include "feature_manager.h"
#include "utility/Twist.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

int KeyPointLandmark::endFrame() {
  return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[]) : Rs(_Rs) {
  for (int i = 0; i < NUM_OF_CAM; i++)
    ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[]) {
  for (int i = 0; i < NUM_OF_CAM; i++) {
    ric[i] = _ric[i];
  }
}

void FeatureManager::clearState() { KeyPointLandmarks.clear(); }

int FeatureManager::getFeatureCount() {
  int cnt = 0;
  // TODO: 使用 unordermap 代替 list
  for (auto &landmark : KeyPointLandmarks) {

    landmark.second.used_num = landmark.second.obs.size();

    if (landmark.second.used_num >= 2 and
        time_frameid2_int_frameid_->at(landmark.second.kf_id) <
            WINDOW_SIZE - 2) {
      cnt++;
    }
  }
  // TODO: 使用 unordermap 代替 list
  return cnt;
}

/**
 * @brief   处理图像特征数据
 * @Description
 * addFeatureCheckParallax()添加特征点到feature中，计算点跟踪的次数和视差，判断是否是关键帧
 *              判断并进行外参标定
 *              进行视觉惯性联合初始化或基于滑动窗口非线性优化的紧耦合VIO
 * @param[in]   image
 * 某帧所有特征点的[camera_id,[x,y,z,u,v,vx,vy]]s构成的map,索引为feature_id
 * @param[in]   header 某帧图像的头信息
 * @return  void
 */

bool FeatureManager::addFeatureCheckParallax(
    int frame_count, const vins::TimeFrameId frame_id,
    const vins::FeatureTrackerResulst &image, double td) {
  ROS_DEBUG("input feature: %d", (int)image.size());
  ROS_DEBUG("num of feature: %d", getFeatureCount());
  double parallax_sum = 0;
  int parallax_num = 0;
  last_track_num = 0;

  int old_feature = 0;
  int new_feature = 0;

  for (auto &id_pts : image) {
    KeypointObservation kpt_obs(id_pts.second[0].second, td);
    vins::FeatureID feature_id = id_pts.first;

    if (KeyPointLandmarks.find(feature_id) == KeyPointLandmarks.end()) {
      // 需要创建新的3D点
      KeyPointLandmarks[feature_id] = KeyPointLandmark(feature_id, frame_id);
      // TODO:新加的观测使用map存放特征点的观测
      CHECK(frame_id != 0) << "frame_id == 0";
      KeyPointLandmarks[feature_id].obs[frame_id] = kpt_obs;
      KeyPointLandmarks[feature_id].kf_id = frame_id;
      new_feature++;
    } else {
      KeyPointLandmarks[feature_id].obs[frame_id] = kpt_obs;
      last_track_num++;
      old_feature++;
    }
  }

  LOG(INFO) << "old feature: " << old_feature
            << " new feature: " << new_feature;
  if (frame_count < 2 || last_track_num < 20)
    return true;

  CHECK(frame_count >= 2) << "frame size < 2";
  CHECK(int_frameid2_time_frameid_->size() >= 2)
      << "size: " << int_frameid2_time_frameid_->size();

  auto target1_tid = int_frameid2_time_frameid_->at(frame_count - 2);
  auto target2_tid = int_frameid2_time_frameid_->at(frame_count - 1);

  // TODO: 使用unordered_map 代替 list start
  for (const auto &landmark : KeyPointLandmarks) {

    if (landmark.second.obs.find(target1_tid) != landmark.second.obs.end() &&
        landmark.second.obs.find(target2_tid) != landmark.second.obs.end()) {
      parallax_sum += compensatedParallax2(landmark.second, frame_count);
      parallax_num++;
    }
  }
  // TODO: 使用unordered_map 代替 list end

  if (parallax_num == 0) {
    return true;
  } else {
    ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum,
              parallax_num);
    ROS_DEBUG("current parallax: %lf",
              parallax_sum / parallax_num * FOCAL_LENGTH);
    return parallax_sum / parallax_num >= MIN_PARALLAX;
  }
}

//void FeatureManager::debugShow() {
//  ROS_DEBUG("debug show");
//  for (auto &it : feature) {
//    ROS_ASSERT(it.feature_per_frame.size() != 0);
//    ROS_ASSERT(it.start_frame >= 0);
//    ROS_ASSERT(it.used_num >= 0);
//
//    ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
//    int sum = 0;
//    for (auto &j : it.feature_per_frame) {
//      ROS_DEBUG("%d,", int(j.is_used));
//      sum += j.is_used;
//      printf("(%lf,%lf) ", j.normalpoint(0), j.normalpoint(1));
//    }
//    ROS_ASSERT(it.used_num == sum);
//  }
//}

Eigen::aligned_vector<pair<Vector3d, Vector3d>>
FeatureManager::getCorresponding(int frame_count_l, int frame_count_r) {
  Eigen::aligned_vector<pair<Vector3d, Vector3d>> corres;

  for (const auto &landmark : KeyPointLandmarks) {
    auto i_tid = int_frameid2_time_frameid_->at(frame_count_l);
    auto j_tid = int_frameid2_time_frameid_->at(frame_count_r);

    if (landmark.second.obs.find(i_tid) != landmark.second.obs.end() &&
        landmark.second.obs.find(j_tid) != landmark.second.obs.end()) {

      Vector3d a = landmark.second.obs.at(i_tid).normalpoint;
      Vector3d b = landmark.second.obs.at(j_tid).normalpoint;

      corres.push_back(make_pair(a, b));
    }
  }

  return corres;
}

void FeatureManager::invDepth2Depth()
{
  for (auto &landmark : KeyPointLandmarks)
  {
    landmark.second.used_num = landmark.second.obs.size();
    if (!(landmark.second.used_num >= 2 && time_frameid2_int_frameid_->at(landmark.second.kf_id) < WINDOW_SIZE - 2))
      continue;

    landmark.second.estimated_depth = 1.0 / landmark.second.data[0];
    //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
    if (landmark.second.estimated_depth < 0)
    {
      landmark.second.solve_flag = KeyPointLandmark::SolveFlag::SOLVE_FAIL;
    }
    else
      landmark.second.solve_flag = KeyPointLandmark::SolveFlag::SOLVE_SUCC;
  }
}
void FeatureManager::depth2InvDepth()
{
  for (auto &landmark : KeyPointLandmarks)
  {
    landmark.second.used_num = landmark.second.obs.size();
    if (!(landmark.second.used_num >= 2 && time_frameid2_int_frameid_->at(landmark.second.kf_id) < WINDOW_SIZE - 2))
      continue;
    landmark.second.data[0] = 1. / landmark.second.estimated_depth;
  }
}

void FeatureManager::resetDepth()
{
  for (auto &landmark : KeyPointLandmarks)
  {
    landmark.second.used_num = landmark.second.obs.size();
    if (!(landmark.second.used_num >= 2 && time_frameid2_int_frameid_->at(landmark.second.kf_id) < WINDOW_SIZE - 2))
      continue;
    landmark.second.estimated_depth = -1.;
  }
}



void FeatureManager::removeFailures()
{
  for (auto iter = KeyPointLandmarks.begin(); iter != KeyPointLandmarks.end();)
  {
    if (iter->second.solve_flag == KeyPointLandmark::SolveFlag::SOLVE_FAIL)
    {
      iter = KeyPointLandmarks.erase(iter);
      continue;
    }
    ++iter;
  }
}



//https://blog.csdn.net/kokerf/article/details/72844455
void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{

  for (auto &landmark : KeyPointLandmarks)
  {
    landmark.second.used_num = landmark.second.obs.size();
    if (!(landmark.second.used_num >= 2 && time_frameid2_int_frameid_->at(landmark.second.kf_id) < WINDOW_SIZE - 2))
      continue;

    if (landmark.second.estimated_depth > 0)
      continue;

    auto host_tid = landmark.second.kf_id;
    auto host_id = time_frameid2_int_frameid_->at(host_tid);

    //一个特征贡献两个约束
    ROS_ASSERT(NUM_OF_CAM == 1);

    Eigen::Matrix<double, 3, 4> P0;
    Eigen::Vector3d t0 = Ps[host_id] + Rs[host_id] * tic[0];
    Eigen::Matrix3d R0 = Rs[host_id] * ric[0];
    P0.leftCols<3>() = Eigen::Matrix3d::Identity();
    P0.rightCols<1>() = Eigen::Vector3d::Zero();

    Eigen::MatrixXd svd_A(2 * landmark.second.obs.size(), 4);
    int svd_idx = 0;
    //P = [P1 P2 P3]^T
    //AX=0      A = [A(2*i) A(2*i+1) A(2*i+2) A(2*i+3) ...]^T
    //A(2*i)   = x(i) * P3 - z(i) * P1
    //A(2*i+1) = y(i) * P3 - z(i) * P2
    for (const auto &it_per_frame : landmark.second.obs)
    {

      auto target_tid = it_per_frame.first;
      auto target_id = time_frameid2_int_frameid_->at(target_tid);

      Eigen::Vector3d t1 = Ps[target_id] + Rs[target_id] * tic[0];
      Eigen::Matrix3d R1 = Rs[target_id] * ric[0];
      Eigen::Vector3d t = R0.transpose() * (t1 - t0);
      Eigen::Matrix3d R = R0.transpose() * R1;
      Eigen::Matrix<double, 3, 4> P;
      P.leftCols<3>() = R.transpose();
      P.rightCols<1>() = -R.transpose() * t;
      Eigen::Vector3d f = it_per_frame.second.normalpoint.normalized();
      svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
      svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);
    }
    //对A的SVD分解得到其最小奇异值对应的单位奇异向量(x,y,z,w)，深度为z/w
    ROS_ASSERT(svd_idx == svd_A.rows());
    Eigen::Vector4d
        svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
    double svd_method = svd_V[2] / svd_V[3];
    //it_per_id->estimated_depth = -b / A;
    //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

    landmark.second.estimated_depth = svd_method;
    //it_per_id->estimated_depth = INIT_DEPTH;

    //0 initial; 1 by depth image; 2 by triangulate
    if (landmark.second.estimated_depth < 0.1)
    {
      landmark.second.estimated_depth = INIT_DEPTH;
    }
  }
} //triangulate

void FeatureManager::removeOneFrameObservation(vins::TimeFrameId marg_frame_tid)
{
  //LOG(INFO) << "marg_frame_tid: " << marg_frame_tid;
  for (auto iter = KeyPointLandmarks.begin(); iter != KeyPointLandmarks.end();)
  {
    auto &obs = iter->second.obs;
    if (obs.find(marg_frame_tid) != obs.end())
    {

      obs.erase(marg_frame_tid);
      if (obs.empty())
      {
        iter = KeyPointLandmarks.erase(iter);
        continue;
      }
      iter->second.kf_id = obs.begin()->first; // 更换3D点的主导
      CHECK(iter->second.kf_id != 0) << "update kf_id error";
    }
    ++iter;
  }
}

void FeatureManager::removeOneFrameObservationAndShiftDepth(vins::TimeFrameId marg_frame_tid,
                                                            Vector3d Ps[],
                                                            Vector3d tic[], Matrix3d ric[])
{
  //LOG(INFO) << "marg_frame_tid: " << std::to_string(marg_frame_tid);

  auto oldhost_tid = marg_frame_tid;
  int oldhost_id = time_frameid2_int_frameid_->at(oldhost_tid);

  Transformd T_i_c(ric[0], tic[0]);
  Transformd T_c_i = T_i_c.inverse();
  Transformd T_w_oldhost_i(Rs[oldhost_id], Ps[oldhost_id]);

  for (auto iter = KeyPointLandmarks.begin(); iter != KeyPointLandmarks.end();)
  {
    auto &obs = iter->second.obs;
    if (obs.find(marg_frame_tid) != obs.end())
    {

      CHECK(iter->second.obs.begin()->first == marg_frame_tid) << "ERROR";
      Eigen::Vector3d normalized_point = iter->second.obs.begin()->second.normalpoint;
      Eigen::Vector3d pts_oldhost = normalized_point * iter->second.estimated_depth;

      obs.erase(marg_frame_tid);
      if (obs.empty())
      {
        iter = KeyPointLandmarks.erase(iter);
        continue;
      }
      //如果3D点的第一个观测帧和记录的主导帧不一样了，那么需要更新3D点的主导帧
      if (iter->second.kf_id != obs.begin()->first)
      {

        iter->second.kf_id = obs.begin()->first; // update 3d点host frame id

        auto newhost_tid = iter->second.kf_id;

        int newhost_id = time_frameid2_int_frameid_->at(newhost_tid);

        Transformd T_w_newhost_i(Rs[newhost_id], Ps[newhost_id]);
        Transformd T_newhost_c_oldhost_c = T_c_i * T_w_newhost_i.inverse() * T_w_oldhost_i * T_i_c;

        Eigen::Vector3d pts_newhost = T_newhost_c_oldhost_c * pts_oldhost;
        double newdepth = pts_newhost(2);
        if (newdepth > 0)
          iter->second.estimated_depth = newdepth;
        else
          iter->second.estimated_depth = INIT_DEPTH;
      }
    }
    ++iter;
  }
} // function removeOneFrameObservationAndShiftDepth



double FeatureManager::compensatedParallax2(const KeyPointLandmark &it_per_id,
                                            int frame_count) {

  // check the second last frame is keyframe or not
  // parallax betwwen seconde last frame and third last frame

  if (frame_count < 1)
    return 0;
  auto target1_tid = int_frameid2_time_frameid_->at(frame_count - 2);
  auto target2_tid = int_frameid2_time_frameid_->at(frame_count - 1);

  CHECK(it_per_id.obs.find(target1_tid) != it_per_id.obs.end() &&
        it_per_id.obs.find(target2_tid) != it_per_id.obs.end())
      << "newest two frame don't have fature observaton";

  const auto &target1_obs = it_per_id.obs.at(target1_tid);
  const auto &target2_obs = it_per_id.obs.at(target2_tid);

  double ans = 0;
  Vector3d p_j = target2_obs.normalpoint;

  double u_j = p_j(0);
  double v_j = p_j(1);

  Vector3d p_i = target1_obs.normalpoint;
  Vector3d p_i_comp;

  // int r_i = frame_count - 2;
  // int r_j = frame_count - 1;
  // p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] *
  // ric[camera_id_i] * p_i;
  p_i_comp = p_i;
  double dep_i = p_i(2);
  double u_i = p_i(0) / dep_i;
  double v_i = p_i(1) / dep_i;
  double du = u_i - u_j, dv = v_i - v_j;

  double dep_i_comp = p_i_comp(2);
  double u_i_comp = p_i_comp(0) / dep_i_comp;
  double v_i_comp = p_i_comp(1) / dep_i_comp;
  double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

  ans = max(
      ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

  return ans;
}