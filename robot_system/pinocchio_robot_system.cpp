#include "robot_system/pinocchio_robot_system.hpp"
using namespace pinocchio;
using namespace std;

PinocchioRobotSystem::PinocchioRobotSystem(const std::string _urdf_file,
                                           const std::string _package_dir,
                                           const bool _b_fixed_base,
                                           const bool _b_print_info)
    : RobotSystem(_b_fixed_base, _b_print_info), urdf_file_(_urdf_file),
      package_dir_(_package_dir) {
  this->_config_robot();

  joint_positions.resize(n_a);
  joint_velocities.resize(n_a);
  q = Eigen::VectorXd::Zero(n_q);
  q_dot = Eigen::VectorXd::Zero(n_q_dot);

  Ag.resize(6, n_q_dot);
}

PinocchioRobotSystem::~PinocchioRobotSystem() {}

void PinocchioRobotSystem::_config_robot() {

  if (b_fixed_base) {
    pinocchio::urdf::buildModel(urdf_file_, model);
    pinocchio::urdf::buildGeom(model, urdf_file_, COLLISION, collision_model,
                               package_dir_);
    pinocchio::urdf::buildGeom(model, urdf_file_, VISUAL, visual_model,
                               package_dir_);

    n_floating = 0;
  } else {
    pinocchio::urdf::buildModel(urdf_file_, pinocchio::JointModelFreeFlyer(),
                                model);
    pinocchio::urdf::buildGeom(model, urdf_file_, COLLISION, collision_model,
                               package_dir_);
    pinocchio::urdf::buildGeom(model, urdf_file_, VISUAL, visual_model,
                               package_dir_);

    n_floating = 6;
  }

  data = Data(model);
  collision_data = GeometryData(collision_model);
  visual_data = GeometryData(visual_model);

  n_q = model.nq;
  n_q_dot = model.nv;
  n_a = n_q_dot - n_floating;

  int passing_index = 0;
  for (JointIndex i = 1; i < (JointIndex)model.njoints;
       ++i) { // NOT SURE IF START AT 0???
    if (model.names[i] == "root_joint" || model.names[i] == "universe") {
      passing_index += 1;
    } else {
      joint_id_[model.names[i]] = i - passing_index;
    }
  }

  for (FrameIndex i = 1; i < (FrameIndex)model.nframes; ++i) {
    string frameName = model.frames[i].name;
    if (frameName == "root_joint" || frameName == "universe") {
    } // pass
    else {
      if (i % 2 == 0) {
        // Link
        int link_id = floor(i / 2 - 1);
        link_id_[frameName] = link_id;
      } else {
      } // Joint
    }
  }

  assert(joint_id_.size() == n_a);

  for (size_t i = 1; i < (size_t)(model.njoints); ++i) {
    total_mass += model.inertias[i].mass();
  }

  joint_pos_limit.resize(n_a, 2);
  joint_vel_limit.resize(n_a, 2);
  joint_trq_limit.resize(n_a, 2);

  if (b_fixed_base) {
    joint_pos_limit.block(0, 0, n_a, 1) = model.lowerPositionLimit;
    joint_pos_limit.block(0, 1, n_a, 1) = model.upperPositionLimit;
  } else {
    joint_pos_limit.block(0, 0, n_a, 1) =
        model.lowerPositionLimit.segment(n_floating, n_a);
    joint_pos_limit.block(0, 1, n_a, 1) =
        model.upperPositionLimit.segment(n_floating, n_a);
    joint_vel_limit.block(0, 0, n_a, 1) =
        -model.velocityLimit.segment(n_floating, n_a);
    joint_vel_limit.block(0, 1, n_a, 1) =
        model.velocityLimit.segment(n_floating, n_a);
    joint_trq_limit.block(0, 0, n_a, 1) =
        -model.effortLimit.segment(n_floating, n_a);
    joint_trq_limit.block(0, 1, n_a, 1) =
        model.effortLimit.segment(n_floating, n_a);
  }
}

Eigen::Vector3d PinocchioRobotSystem::get_base_local_com_pos() {}

std::string PinocchioRobotSystem::get_base_link_name() {}

int PinocchioRobotSystem::get_q_idx(const std::string joint_name) {
  return model.joints[model.getJointId(joint_name)].idx_q();
}

int PinocchioRobotSystem::get_q_dot_idx(const std::string joint_name) {
  return model.joints[model.getJointId(joint_name)].idx_v();
}

int PinocchioRobotSystem::get_joint_idx(const std::string joint_name) {
  return model.joints[model.getJointId(joint_name)].idx_v() - n_floating;
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
    const std::map<std::string, double> joint_vel, const bool b_cent) {

  assert(joint_pos.size() == n_a);

  Eigen::VectorXd quat_vec(4);
  quat_vec << base_joint_quat.x(), base_joint_quat.y(), base_joint_quat.z(),
      base_joint_quat.w();

  if (!b_fixed_base) {

    q.segment(0, 3) = base_joint_pos;
    q.segment(3, 4) = quat_vec;

    Eigen::Matrix<double, 3, 3> rot_w_basejoint =
        base_joint_quat.normalized().toRotationMatrix();
    Eigen::Matrix<double, 6, 1> twist_basejoint_in_world;
    twist_basejoint_in_world.segment(0, 3) = base_joint_ang_vel;
    twist_basejoint_in_world.segment(3, 3) = base_joint_lin_vel;

    Eigen::Matrix<double, 6, 6> augrot_joint_world;
    augrot_joint_world.setZero();
    augrot_joint_world.block(0, 0, 3, 3) = rot_w_basejoint.transpose();
    augrot_joint_world.block(3, 3, 3, 3) = rot_w_basejoint.transpose();

    Eigen::Matrix<double, 6, 1> twist_basejoint_in_joint =
        augrot_joint_world * twist_basejoint_in_world;
    q_dot.segment(0, 3) = twist_basejoint_in_joint.segment(3, 3);
    q_dot.segment(3, 3) = twist_basejoint_in_joint.segment(0, 3);
  } else {
  } // fixed base robot

  for (const auto [key, value] : joint_pos) {
    q(get_q_idx(key)) = value;
    joint_positions(get_joint_idx(key)) = value;
  }
  for (const auto [key, value] : joint_vel) {
    q_dot(get_q_dot_idx(key)) = value;
    joint_velocities(get_joint_idx(key)) = value;
  }

  forwardKinematics(model, data, q, q_dot);

  if (b_cent) {
    _update_centroidal_quantities();
  }
}

void PinocchioRobotSystem::_update_centroidal_quantities() {
  ccrba(model, data, q, q_dot);

  hg.segment(0, 3) = data.hg.angular();
  hg.segment(3, 3) = data.hg.linear();

  Ag.topRows(3) = data.Ag.template middleRows<3>(Force::ANGULAR);
  Ag.bottomRows(3) = data.Ag.template middleRows<3>(Force::LINEAR);

  Ig.setZero();
  Ig.block(0, 0, 3, 3) = data.Ig.matrix().block(3, 3, 3, 3);
  Ig.block(3, 3, 3, 3) = data.Ig.matrix().block(0, 0, 3, 3);
}

Eigen::VectorXd PinocchioRobotSystem::get_q() {
  Eigen::VectorXd q_copy = q;
  return q_copy;
}

Eigen::VectorXd PinocchioRobotSystem::get_q_dot() {
  Eigen::VectorXd q_dot_copy = q_dot;
  return q_dot_copy;
}

Eigen::MatrixXd PinocchioRobotSystem::get_mass_matrix() {
  crba(model, data, q);
  data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();

  Eigen::MatrixXd mass_matrix = data.M;
  return mass_matrix;
}

Eigen::VectorXd PinocchioRobotSystem::get_gravity() {
  Eigen::VectorXd gen_gravity = computeGeneralizedGravity(model, data, q);
  return gen_gravity;
}

Eigen::VectorXd PinocchioRobotSystem::get_coriolis() {
  Eigen::VectorXd coriolis =
      nonLinearEffects(model, data, q, q_dot) - get_gravity();
  return coriolis;
}

Eigen::Vector3d PinocchioRobotSystem::get_com_pos() {
  crba(model, data, q);
  Eigen::Vector3d com = centerOfMass(model, data, q, q_dot);
  return com;
}

Eigen::Vector3d PinocchioRobotSystem::get_com_lin_vel() {
  crba(model, data, q);
  centerOfMass(model, data, q, q_dot);
  Eigen::Vector3d vcom = data.vcom[0];
  return vcom;
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
PinocchioRobotSystem::get_com_lin_jacobian() {
  crba(model, data, q);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jcom =
      jacobianCenterOfMass(model, data, q);
  return Jcom;
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
PinocchioRobotSystem::get_com_lin_jacobian_dot() {
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jcom_dot =
      (computeCentroidalMapTimeVariation(model, data, q, q_dot).topRows(3)) /
      total_mass;
  return Jcom_dot;
}

Eigen::Isometry3d
PinocchioRobotSystem::get_link_iso(const std::string link_id) {
  Eigen::Isometry3d ret;
  Model::Index frame_id = model.getFrameId(link_id);
  const SE3Tpl<double, 0> trans = updateFramePlacement(model, data, frame_id);
  ret = trans.rotation();
  ret.translation() = trans.translation();
  return ret;
}

Eigen::Matrix<double, 6, 1>
PinocchioRobotSystem::get_link_vel(const std::string link_id) {
  Eigen::Matrix<double, 6, 1> ret;
  Model::Index frame_id = model.getFrameId(link_id);
  Motion vf = getFrameVelocity(model, data, frame_id, LOCAL_WORLD_ALIGNED);

  ret.segment(0, 3) = vf.angular();
  ret.segment(3, 3) = vf.linear();

  return ret;
}

Eigen::Matrix<double, 6, Eigen::Dynamic>
PinocchioRobotSystem::get_link_jacobian(const std::string link_id) {
  Model::Index frame_id = model.getFrameId(link_id);
  computeJointJacobians(model, data, q);

  cout << "before" << endl;
  Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(6, n_q_dot);
  getFrameJacobian(model, data, frame_id, LOCAL_WORLD_ALIGNED, jac);

  cout << "after" << endl;
  Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(6, n_q_dot);

  cout << "before toprows" << endl;
  ret.topRows(3) = jac.bottomRows(3);
  ret.bottomRows(3) = jac.topRows(3);
  cout << "after rows" << endl;

  return ret;
}

Eigen::Matrix<double, 6, 1>
PinocchioRobotSystem::get_link_jacobian_dot_times_qdot(
    const std::string link_id) {
  Model::Index frame_id = model.getFrameId(link_id);

  forwardKinematics(model, data, q, q_dot, 0 * q_dot);
  Motion af =
      getFrameClassicalAcceleration(model, data, frame_id, LOCAL_WORLD_ALIGNED);

  Eigen::Matrix<double, 6, 1> ret;
  ret.segment(0, 3) = af.angular();
  ret.segment(3, 3) = af.linear();

  return ret;
}
