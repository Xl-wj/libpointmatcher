
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Parametrizable.h"
#include <glog/logging.h>

using namespace std;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

int main(int argc, char *argv[]) {

  LOG(INFO) << sizeof(double);
  LOG(INFO) << sizeof(int64_t);
  double time_te = 98.2255255101155;
  int64_t time_c = time_te;
  LOG(INFO) << time_te;
  LOG(INFO) << time_c;

  std::string pcd_file = "/home/xl/Desktop/LIDAR_DATA/velodyne_16/6.pcd";
  DP scan(DP::load(pcd_file));
//  DP scan;
  LOG(INFO) << "features size, rows: " << scan.features.rows() << ", col: " << scan.features.cols();
  LOG(INFO) << "featureLabels size: " << scan.featureLabels.size();

  for(const auto &label : scan.featureLabels)
    LOG(INFO) << "label span: " << label.span << ", text: " << label.text;

  LOG(INFO) << "descriptors size, rows: " << scan.descriptors.rows() << ", col: " << scan.descriptors.cols();
  LOG(INFO) << "featureLabels size: " << scan.descriptorLabels.size();

  for(const auto &label : scan.descriptorLabels)
    LOG(INFO) << "label span: " << label.span << ", text: " << label.text;

  scan.times = Eigen::Matrix<std::int64_t, 1, 30000>::Zero();
  scan.timeLabels.push_back(DP::Label( "timestamp", 1));
  LOG(INFO) << "times size: " << scan.times.size();
  LOG(INFO) << "timeLabels size: " << scan.timeLabels.size();

  for(const auto &label : scan.timeLabels)
    LOG(INFO) << "label span: " << label.span << ", text: " << label.text;

  return 0;
}
