#include "robot_system/pinocchio_robot_system.hpp"

PinocchioRobotSystem::PinocchioRobotSystem(const std::string _urdf_file,
                                           const std::string _package_dir,
                                           const bool _b_fixed_base,
                                           const bool _b_print_info)
    : RobotSystem(_b_fixed_base, _b_print_info), urdf_file_(_urdf_file),
      package_dir_(_package_dir) {
  this->_config_robot();

  joint_positions.resize(n_a);
  joint_velocities.resize(n_a);

  Ag.resize(6, n_q_dot);
}

PinocchioRobotSystem::~PinocchioRobotSystem() {}

void PinocchioRobotSystem::_config_robot() {}

Eigen::Vector3d PinocchioRobotSystem::get_base_local_com_pos() {}

std::string PinocchioRobotSystem::get_base_link_name() {}

int PinocchioRobotSystem::get_q_idx(const std::string joint_name) {
    return joint_id_[joint_name];
}

int PinocchioRobotSystem::get_q_dot_idx(const std::string joint_name) {
    return joint_id_[joint_name];
}

int PinocchioRobotSystem::get_joint_idx(const std::string joint_name) {
    return joint_id_[joint_name] - n_floating;
}

std::map<std::string, double>
PinocchioRobotSystem::vector_to_map(const Eigen::VectorXd &cmd_vec) {}

Eigen::VectorXd
PinocchioRobotSystem::map_to_vector(std::map<std::string, double> _map) {}

void PinocchioRobotSystem::update_system(
    const Eigen::Vector3d base_com_pos,
    const Eigen::Quaternion<double> base_com_quat,
    const Eigen::Vector3d base_com_lin_vel,
    const Eigen::Vector3d base_com_ang_vel,
    const Eigen::Vector3d base_joint_pos,
    const Eigen::Quaternion<double> base_joint_quat,
    const Eigen::Vector3d base_joint_lin_vel,
    const Eigen::Vector3d base_joint_ang_vel,
    const std::map<std::string, double> joint_pos,
    const std::map<std::string, double> joint_vel, const bool b_cent) {}

void PinocchioRobotSystem::_update_centroidal_quantities() {}

Eigen::VectorXd PinocchioRobotSystem::get_q() {}

Eigen::VectorXd PinocchioRobotSystem::get_q_dot() {}

Eigen::MatrixXd PinocchioRobotSystem::get_mass_matrix() {}

Eigen::VectorXd PinocchioRobotSystem::get_gravity() {}

Eigen::VectorXd PinocchioRobotSystem::get_coriolis() {}

Eigen::Vector3d PinocchioRobotSystem::get_com_pos() {}

Eigen::Vector3d PinocchioRobotSystem::get_com_lin_vel() {}

Eigen::Matrix<double, 3, Eigen::Dynamic>
PinocchioRobotSystem::get_com_lin_jacobian() {}

Eigen::Matrix<double, 3, Eigen::Dynamic>
PinocchioRobotSystem::get_com_lin_jacobian_dot() {}

Eigen::Isometry3d
PinocchioRobotSystem::get_link_iso(const std::string link_id) {}

Eigen::Matrix<double, 6, 1>
PinocchioRobotSystem::get_link_vel(const std::string link_id) {}

Eigen::Matrix<double, 6, Eigen::Dynamic>
PinocchioRobotSystem::get_link_jacobian(const std::string link_id) {}

Eigen::Matrix<double, 6, 1>
PinocchioRobotSystem::get_link_jacobian_dot_times_qdot(
    const std::string link_id) {}
