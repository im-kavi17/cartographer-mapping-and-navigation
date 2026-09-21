/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf
*
** Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es
*
* Modifications: Jeremie Deray & (see contributons on github)
******************************************************************************************** */

#ifndef CLaserOdometry2D_H
#define CLaserOdometry2D_H

// std header
#include <iostream>
#include <fstream>
#include <numeric>
#include <memory>
// ROS headers
//
// NOTE: this class is a plain (non-ROS) algorithm wrapper. It used to inherit from
// rclcpp::Node, which created a *second* ROS graph entity ("CLaserOdometry2D") next to
// the real node ("CLaserOdometry2DNode"). It now only depends on the two services it
// actually needs -- logging and time -- which are injected through the constructor.
// See: https://github.com/MAPIRlab/rf2o_laser_odometry/issues/42
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
// Eigen headers
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

namespace rf2o {

template <typename T>
inline T sign(const T x) { return x<T(0) ? -1:1; }

template <typename Derived>
inline typename Eigen::MatrixBase<Derived>::Scalar
getYaw(const Eigen::MatrixBase<Derived>& r)
{
  return std::atan2( r(1, 0), r(0, 0) );
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> matrixRollPitchYaw(const T roll,
                                                 const T pitch,
                                                 const T yaw)
{
  const Eigen::AngleAxis<T> ax = Eigen::AngleAxis<T>(roll,  Eigen::Matrix<T, 3, 1>::UnitX());
  const Eigen::AngleAxis<T> ay = Eigen::AngleAxis<T>(pitch, Eigen::Matrix<T, 3, 1>::UnitY());
  const Eigen::AngleAxis<T> az = Eigen::AngleAxis<T>(yaw,   Eigen::Matrix<T, 3, 1>::UnitZ());

  return (az * ay * ax).toRotationMatrix().matrix();
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> matrixYaw(const T yaw)
{
  return matrixRollPitchYaw<T>(0, 0, yaw);
}

using Scalar = float;
using Pose2d = Eigen::Isometry2d;
using Pose3d = Eigen::Isometry3d;
using MatrixS31 = Eigen::Matrix<Scalar, 3, 1>;
using IncrementCov = Eigen::Matrix<Scalar, 3, 3>;


/**
 * Planar (2D) odometry estimator from consecutive range scans.
 *
 * This class is deliberately NOT a rclcpp::Node: it is a self-contained algorithm that only
 * needs a logger (to report progress) and a clock (to measure its own runtime). Both are
 * passed in at construction time, so embedding this class in a node no longer registers a
 * duplicate node in the ROS graph.
 */
class CLaserOdometry2D
{
public:

  /**
   * @brief Preferred constructor: explicit dependency injection.
   * @param logger used by every RCLCPP_* call inside this class.
   * @param clock  used to time the odometry computation. Defaults to a system clock
   *               (equivalent to what rclcpp::Node::get_clock() used to return by default).
   */
  explicit CLaserOdometry2D(const rclcpp::Logger & logger,
                            const rclcpp::Clock::SharedPtr & clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));

  /**
   * @brief Convenience constructor: pulls the logger and the clock out of an existing node.
   * @param node an already created node; only a (non-owning) reference to its logger and a
   *             shared pointer to its clock are kept, the node itself is not stored.
   */
  explicit CLaserOdometry2D(const rclcpp::Node::SharedPtr & node);

  /// Logger used for the RCLCPP_* macros.
  const rclcpp::Logger & logger() const { return logger_; }
  /// Clock used to measure the computation time.
  const rclcpp::Clock::SharedPtr & clock() const { return clock_; }

  void init(const sensor_msgs::msg::LaserScan& scan,
            const geometry_msgs::msg::Pose& initial_robot_pose);

  bool is_initialized();

  bool odometryCalculation(const sensor_msgs::msg::LaserScan& scan);

  void setLaserPose(const Pose3d& laser_pose);

  const Pose3d& getIncrement() const;

  const IncrementCov& getIncrementCovariance() const;

  Pose3d& getPose();
  const Pose3d& getPose() const;
  bool verbose, module_initialized, first_laser_scan;
  rclcpp::Time last_odom_time, current_scan_time;

  // Internal Data
  std::vector<Eigen::MatrixXf> range;
  std::vector<Eigen::MatrixXf> range_old;
  std::vector<Eigen::MatrixXf> range_inter;
  std::vector<Eigen::MatrixXf> range_warped;
  std::vector<Eigen::MatrixXf> xx;
  std::vector<Eigen::MatrixXf> xx_inter;
  std::vector<Eigen::MatrixXf> xx_old;
  std::vector<Eigen::MatrixXf> xx_warped;
  std::vector<Eigen::MatrixXf> yy;
  std::vector<Eigen::MatrixXf> yy_inter;
  std::vector<Eigen::MatrixXf> yy_old;
  std::vector<Eigen::MatrixXf> yy_warped;
  std::vector<Eigen::MatrixXf> transformations;

  Eigen::MatrixXf range_wf;
  Eigen::MatrixXf dtita;
  Eigen::MatrixXf dt;
  Eigen::MatrixXf rtita;
  Eigen::MatrixXf normx, normy, norm_ang;
  Eigen::MatrixXf weights;
  Eigen::MatrixXi null;

  Eigen::MatrixXf A,Aw;
  Eigen::MatrixXf B,Bw;

  MatrixS31 Var;	//3 unknowns: vx, vy, w
  IncrementCov cov_odo;

  //std::string LaserVarName;				//Name of the topic containing the scan lasers \laser_scan
  float fps;								//In Hz
  float fovh;								//Horizontal FOV
  unsigned int cols;
  unsigned int cols_i;
  unsigned int width;
  unsigned int ctf_levels;
  unsigned int image_level, level;
  unsigned int num_valid_range;
  unsigned int iter_irls;
  float g_mask[5];

  double lin_speed, ang_speed;

  //rclcpp::wall	m_runtime;

  MatrixS31 kai_abs_;
  MatrixS31 kai_loc_;
  MatrixS31 kai_loc_old_;
  MatrixS31 kai_loc_level_;

  Pose3d last_increment_;
  Pose3d laser_pose_on_robot_;
  Pose3d laser_pose_on_robot_inv_;
  Pose3d laser_pose_;
  Pose3d laser_oldpose_;
  Pose3d robot_pose_;
  Pose3d robot_oldpose_;

  bool test;
  std::vector<double> last_m_lin_speeds;
  std::vector<double> last_m_ang_speeds;

  // Methods
  void createImagePyramid();
  void calculateCoord();
  void performWarping();
  void calculaterangeDerivativesSurface();
  void computeNormals();
  void computeWeights();
  void findNullPoints();
  void solveSystemOneLevel();
  void solveSystemNonLinear();
  bool filterLevelSolution();
  void PoseUpdate();
  void Reset(const Pose3d& ini_pose/*, CObservation2DRangeScan scan*/);

private:
  rclcpp::Logger        logger_;   ///< injected logger (replaces rclcpp::Node::get_logger())
  rclcpp::Clock::SharedPtr clock_; ///< injected clock (replaces rclcpp::Node::get_clock())
};


} /* namespace rf2o */

#endif // CLaserOdometry2D_H
