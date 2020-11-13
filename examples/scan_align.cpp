
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Parametrizable.h"

#include <cassert>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <iomanip>

#include <glog/logging.h>
#include "boost/filesystem.hpp"
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;


void TwoScanAlignment() {
  const std::string yaml_file = "/home/xl/ranger/src/lidar_odometry/config/libpointmatcher_config.yaml";
  const std::string pcd_dir = "/home/xl/jiashan_test/0819_map_1/seg_pcd/";
  const std::string save_map_file = "/home/xl/align_scans.pcd";

  PM::TransformationParameters map_tf_record = Eigen::Matrix4f::Identity();
  DP map;
  DP last_scan;

  size_t begin_index = 10;
  for(size_t i = begin_index; i < begin_index + 100; i = i + 2) {
    const std::string pcd_file = pcd_dir + std::to_string(i) + "_pure.pcd";
    const std::string pcd_scan_file = pcd_dir + std::to_string(i) + ".pcd";

    cout << "Load file: " << pcd_file << " !" << endl;
    const DP scan(DP::load(pcd_file));

    if(i == begin_index) {
      map = DP::load(pcd_scan_file);
      map_tf_record = Eigen::Matrix4f::Identity();

    } else {
      PM::ICP icp;
      std::ifstream ifs(yaml_file.c_str());
      icp.loadFromYaml(ifs);

      PM::TransformationParameters align_pose = icp(scan, last_scan);

      map_tf_record = map_tf_record.matrix() * align_pose;
      DP tf_scan(DP::load(pcd_scan_file));
      icp.transformations.apply(tf_scan, map_tf_record);

      map.concatenate(tf_scan);
    }

    last_scan = scan;
  }

  map.save(save_map_file);
}


void Scan2MapAlignment(const std::string &yaml_file,
                       const std::string &pcd_dir,
                       const std::string &save_map_file,
                       size_t begin_index, size_t end_index, size_t step_index) {

  PM::TransformationParameters map_tf_record = Eigen::Matrix4f::Identity();
  DP map;
  DP last_scan;

  for(size_t i = begin_index; i < end_index; i = i + step_index) {
    const std::string pcd_file = pcd_dir + std::to_string(i) + ".pcd";

    cout << "Load file: " << pcd_file << " !" << endl;
    const DP scan(DP::load(pcd_file));

    if(i == begin_index) {
      map = scan;
      map_tf_record = Eigen::Matrix4f::Identity();

    } else {
      PM::ICP icp;
      icp.setDefault();
//      std::ifstream ifs(yaml_file.c_str());
//      icp.loadFromYaml(ifs);

      PM::TransformationParameters align_pose = icp(scan, map);

      DP tf_scan(scan);
      icp.transformations.apply(tf_scan, align_pose);

      map.concatenate(tf_scan);
    }
  }

  map.save(save_map_file);
}


void Scan2MapAlignment(const std::string &yaml_file,
                       const std::string &pcd_dir,
                       const std::string &save_map_file,
                       size_t begin_index, size_t end_index,
                       size_t step_index, size_t centrol_index) {

  PM::TransformationParameters map_tf_record = Eigen::Matrix4f::Identity();
  DP map(DP::load(pcd_dir +  std::to_string(centrol_index) + ".pcd"));

  PM::ICP icp;
  icp.setDefault();

  size_t left(centrol_index - step_index);
  while (left >= begin_index) {
    const std::string pcd_file = pcd_dir + std::to_string(left) + ".pcd";
    cout << "Load file: " << pcd_file << " !" << endl;
    const DP scan(DP::load(pcd_file));

    PM::TransformationParameters align_pose = icp(scan, map);
    DP tf_scan(scan);
    icp.transformations.apply(tf_scan, align_pose);
    map.concatenate(tf_scan);

    left -= step_index;
  }

  size_t right_index(centrol_index + step_index);
  while (right_index <= end_index) {
    const std::string pcd_file = pcd_dir + std::to_string(right_index) + ".pcd";
    cout << "Load file: " << pcd_file << " !" << endl;
    const DP scan(DP::load(pcd_file));

    PM::TransformationParameters align_pose = icp(scan, map);
    DP tf_scan(scan);
    icp.transformations.apply(tf_scan, align_pose);
    map.concatenate(tf_scan);

    right_index += step_index;
  }

  map.save(save_map_file);
}

void Scan2MapAlignment() {
  const std::string yaml_file = "/home/xl/ranger/lidar_odometry/config/libpointmatcher_config.p2plane.yaml";
  const std::string pcd_dir = "/home/xl/jiashan_test/0819_map_1/seg_pcd/";
  const std::string save_map_file = "/home/xl/align_map.pcd";
  size_t begin_index = 10;
  size_t end_index = 54;
  size_t step_index = 2;

  Scan2MapAlignment(yaml_file, pcd_dir, save_map_file, begin_index, end_index, step_index);
}

using namespace PointMatcherSupport;

void TestICP() {
  PM::ICP icp;
  icp.setDefault();
  DP target_scan(DP::load("/home/xl/jiashan_test/0819_map_1/seg_pcd/1.pcd"));
  DP source_scan(DP::load("/home/xl/jiashan_test/0819_map_1/seg_pcd/10.pcd"));

  std::shared_ptr<PM::DataPointsFilter> randomSample =
          PM::get().DataPointsFilterRegistrar.create(
                  "RandomSamplingDataPointsFilter",
                  {{"prob", toParam(0.5)}}
          );


  PM::TransformationParameters align_tf = icp(source_scan, target_scan);
  while (true) {
    cout << " icp " << endl;
    icp.transformations.apply(source_scan, align_tf);
  }
  target_scan.concatenate(source_scan);
  target_scan.save("/home/xl/two_scan_align.pcd");
}

struct SubmapFactor : std::pair<uint16_t, uint16_t>{
    PM::TransformationParameters tf;
};
struct SubmapFactorKey : public std::unary_function<SubmapFactor, uint32_t> {
    uint32_t operator()(const SubmapFactor& f) const {
      return (uint32_t(f.first << 16) | uint32_t(f.second));
    }
};


void SaveSubmapFactors(const std::unordered_set<SubmapFactor, SubmapFactorKey> &factors, const std::string &factor_file) {

  std::ofstream fs(factor_file);

  for (const SubmapFactor &factor : factors) {

    Eigen::Affine3d tf(Eigen::Matrix4d(factor.tf.matrix().template cast<double>()));
    Eigen::Quaterniond q(tf.linear());
    Eigen::Vector3d t(tf.translation());

    fs << std::fixed << std::setprecision(12)
       << factor.first << " "
       << factor.second << " "
       << t[0] << " "
       << t[1] << " "
       << t[2] << " "
       << q.x() << " "
       << q.y() << " "
       << q.z() << " "
       << q.w() << std::endl;
  }
}

void SaveSubmapCentrolPose(const std::unordered_set<SubmapFactor, SubmapFactorKey> &factors,
                           const std::string &out_pose_file) {
  std::vector<SubmapFactor> factor_vec(0);
  for (const auto &f : factors) {
    factor_vec.push_back(f);
  }
  sort(factor_vec.begin(), factor_vec.end(), [](const SubmapFactor &a, const SubmapFactor &b){
      return a.first < b.first;
  });

  std::ofstream fs(out_pose_file);
  Eigen::Affine3d world_tf = Eigen::Affine3d::Identity();


  for(size_t i = 0; i < factor_vec.size(); i++) {
    const auto factor = factor_vec[i];
    const size_t id_1 = factor.first;
    const size_t id_2 = factor.second;

    if(i == 0) {
      Eigen::Affine3d tf(world_tf);
      Eigen::Quaterniond q(tf.linear());
      Eigen::Vector3d t(tf.translation());

      fs << std::fixed << std::setprecision(12)
         << factor.first << " "
         << 0.0 << " "
         << t[0] << " "
         << t[1] << " "
         << t[2] << " "
         << q.x() << " "
         << q.y() << " "
         << q.z() << " "
         << q.w() << " "
         << 0.0 << " "
         << 0.0 << " "
         << 0.0 << std::endl;
    }

    world_tf = world_tf * Eigen::Affine3d(Eigen::Matrix4d(factor.tf.matrix().template cast<double>()));

    Eigen::Quaterniond q(world_tf.linear());
    Eigen::Vector3d t(world_tf.translation());

    fs << std::fixed << std::setprecision(12)
       << factor.second << " "
       << 0.0 << " "
       << t[0] << " "
       << t[1] << " "
       << t[2] << " "
       << q.x() << " "
       << q.y() << " "
       << q.z() << " "
       << q.w() << " "
       << 0.0 << " "
       << 0.0 << " "
       << 0.0 << std::endl;

  }
}


void StitchSubmap() {
  const std::string yaml_file = "/home/xl/ranger/lidar_odometry/config/libpointmatcher_config.p2plane.yaml";
  const std::string pcd_dir = "/home/xl/jiashan_test/0819_map_1/seg_pcd/";
  const std::string save_map_dir = "/home/xl/align_map/";
  const std::string factor_map_dir = "/home/xl/align_map/submap_align/";

  size_t step_index = 2;
  size_t batch_size = 14;
  size_t span_index = 4;

  std::vector<std::string> submap_files(0);

  for(size_t batch = 1; batch < 50; batch++) {
    size_t begin_index = batch_size * batch - span_index;
    size_t end_index = batch_size * (batch + 1) + span_index;

    size_t centrol_index = (begin_index + end_index) / 2;
    const std::string save_map_file = save_map_dir + std::to_string(centrol_index) + ".pcd";

    LOG(INFO) << "begin_index: (" << begin_index << "), end_index: (" << end_index << "), save_map_file: " << save_map_file;
    Scan2MapAlignment(yaml_file, pcd_dir, save_map_file, begin_index, end_index, step_index, centrol_index);
    submap_files.push_back(save_map_file);
  }

  std::unordered_set<SubmapFactor, SubmapFactorKey> factors;

  for(size_t i = 0; i < submap_files.size() - 1; i++) {
    std::vector<std::string> strs_1, strs_2;
    boost::split(strs_1, submap_files[i], boost::is_any_of("/"));
    boost::split(strs_2, submap_files[i+1], boost::is_any_of("/"));
    const std::string save_file = factor_map_dir + strs_1.back().substr(0, strs_1.back().size() - 4) + "_" + strs_2.back();

    PM::ICP icp;
    icp.setDefault();

    DP referen(DP::load(submap_files[i]));
    DP reading(DP::load(submap_files[i+1]));

    PM::TransformationParameters align_pose = icp(reading, referen);

    icp.transformations.apply(reading, align_pose);
    referen.concatenate(reading);
    referen.save(save_file);

    SubmapFactor factor;
    factor.first = uint16_t(stoi(strs_1.back().substr(0, submap_files[i].size() - 4)));
    factor.second = uint16_t(stoi(strs_2.back().substr(0, submap_files[i].size() - 4)));
    factor.tf = align_pose;
    factors.insert(factor);
  }

  SaveSubmapFactors(factors, factor_map_dir + "factors.txt");
  SaveSubmapCentrolPose(factors, factor_map_dir + "pose.txt");
}

int main(int argc, char *argv[]) {
//  TwoScanAlignment();

//  Scan2MapAlignment();

  TestICP();

//  StitchSubmap();
  return 0;
}
