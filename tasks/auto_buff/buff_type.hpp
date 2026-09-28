#ifndef BUFF__TYPE_HPP
#define BUFF__TYPE_HPP

#include <algorithm>
#include <deque>
#include <eigen3/Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

#include "tools/math_tools.hpp"
namespace auto_buff
{
const int INF = 1000000;
enum PowerRune_type { SMALL, BIG };
enum FanBlade_type { _target, _unlight, _light };
enum Track_status { TRACK, TEM_LOSE, LOSE };
enum RuneDetectorBackend { SP25, SZU };

class FanBlade
{
public:
  cv::Point2f center;               // 扇页中心
  std::vector<cv::Point2f> points;  // 四个点按图像轮廓顺序排列
  std::vector<int> point_indices;   // 模型原始关键点编号，SZU 五点模型使用
  double angle, width, height;
  FanBlade_type type;  // 类型
  int class_id = -1;
  float confidence = 0.0f;

  explicit FanBlade() = default;

  // explicit FanBlade(const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, FanBlade_type t);

  explicit FanBlade(
    const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, FanBlade_type t);

  explicit FanBlade(
    const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, FanBlade_type t,
    int class_id, float confidence);

  explicit FanBlade(FanBlade_type t);
};

class PowerRune
{
public:
  cv::Point2f r_center;
  std::vector<FanBlade> fanblades;  // 按target开始顺时针

  int light_num;

  Eigen::Vector3d xyz_in_world;  // 单位：m
  Eigen::Vector3d ypr_in_world;  // 单位：rad
  Eigen::Vector3d ypd_in_world;  // 球坐标系

  Eigen::Vector3d blade_xyz_in_world;  // 单位：m
  Eigen::Vector3d blade_ypd_in_world;  // 球坐标系, 单位: m

  // PnP 诊断信息，由 Solver 填充；与跟踪状态分离，便于排查距离问题。
  double pnp_reprojection_error_px = 0.0;
  double pnp_r_reprojection_error_px = 0.0;
  double pnp_center_distance_m = 0.0;  // PnP 原点（R 中心）到相机的直线距离
  double pnp_blade_horizontal_distance_m = 0.0;  // 待击打叶片中心的水平距离
  cv::Point2f pnp_r_projected_pixel{0.0F, 0.0F};  // PnP 反投影得到的 R 点

  explicit PowerRune(
    std::vector<FanBlade> & ts, const cv::Point2f r_center,
    std::optional<PowerRune> last_powerrune);
  explicit PowerRune() = default;

  FanBlade & target() { return fanblades[0]; };

  bool is_unsolve() const { return unsolvable_; }

  void mark_unsolvable() { unsolvable_ = true; }

private:
  double target_angle_;
  bool unsolvable_ = false;

  double atan_angle(cv::Point2f v) const;  // [0, 2CV_PI]
};
}  // namespace auto_buff
#endif  // BUFF_TYPE_HPP
