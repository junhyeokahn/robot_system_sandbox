#pragma once

#define __pinocchio_compute_all_terms_hpp__

#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/fwd.hpp>

#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include "robot_system/robot_system.hpp"

using namespace pinocchio;

/*
 *  Pinnochio considers floating base with 7 positions and 6 velocities with the
 *  order of [x, y, z, quat_x, quat_y, quat_z, quat_w, joints] and
 *  [xdot, ydot, zdot, ang_x, ang_y, ang_z, joints].
 *  Note that first six element of generalized velocities are represented in the
 *  base joint frame acting on the base joint frame.
 */

/// class PinocchioRobotSystem
class PinocchioRobotSystem : public RobotSystem {
public:
  /// \{ \name Constructor and Destructor
  PinocchioRobotSystem(const std::string _urdf_file,
                       const std::string _package_dir, const bool _b_fixed_base,
                       const bool _b_print_info = false);
  virtual ~PinocchioRobotSystem();
  /// \}

  virtual int get_q_idx(const std::string joint_name);
  virtual int get_q_dot_idx(const std::string joint_name);
  virtual int get_joint_idx(const std::string joint_name);
  virtual std::map<std::string, double>
  vector_to_map(const Eigen::VectorXd &cmd_vec);
  virtual Eigen::VectorXd map_to_vector(std::map<std::string, double>);
  virtual Eigen::Vector3d get_base_local_com_pos();
  virtual std::string get_base_link_name();
  virtual void update_system(const Eigen::Vector3d base_com_pos,
                             const Eigen::Quaternion<double> base_com_quat,
                             const Eigen::Vector3d base_com_lin_vel,
                             const Eigen::Vector3d base_com_ang_vel,
                             const Eigen::Vector3d base_joint_pos,
                             const Eigen::Quaternion<double> base_joint_quat,
                             const Eigen::Vector3d base_joint_lin_vel,
                             const Eigen::Vector3d base_joint_ang_vel,
                             const std::map<std::string, double> joint_pos,
                             const std::map<std::string, double> joint_vel,
                             const bool b_cent = false);
  virtual Eigen::VectorXd get_q();
  virtual Eigen::VectorXd get_q_dot();
  virtual Eigen::MatrixXd get_mass_matrix();
  virtual Eigen::VectorXd get_gravity();
  virtual Eigen::VectorXd get_coriolis();
  virtual Eigen::Vector3d get_com_pos();
  virtual Eigen::Vector3d get_com_lin_vel();
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> get_com_lin_jacobian();
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> get_com_lin_jacobian_dot();
  virtual Eigen::Isometry3d get_link_iso(const std::string link_id);
  virtual Eigen::Matrix<double, 6, 1> get_link_vel(const std::string link_id);
  virtual Eigen::Matrix<double, 6, Eigen::Dynamic>
  get_link_jacobian(const std::string link_id);
  virtual Eigen::Matrix<double, 6, 1>
  get_link_jacobian_dot_times_qdot(const std::string link_id);

private:
  virtual void _update_centroidal_quantities();
  virtual void _config_robot();

  Model model;
  GeometryModel collision_model;
  GeometryModel visual_model;

  Data data;
  GeometryData collision_data;
  GeometryData visual_data;

  Eigen::VectorXd q;
  Eigen::VectorXd q_dot;

  std::string urdf_file_;
  std::string package_dir_;

  /// Map of joint name and joint idx
  std::map<std::string, int> joint_id_;

  /// Map of link name and link idx
  std::map<std::string, int> link_id_;
};
