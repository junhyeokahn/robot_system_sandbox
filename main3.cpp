#include "robot_system/pinocchio_robot_system.cpp"
#include <cmath>
#include <iostream>
//#include "robot_system/dart_robot_system.cpp"

using namespace std;

int main(int argc, char *argv[]) {

  PinocchioRobotSystem atlas(
      "/home/junhyeok/Repository/PyPnC/robot_model/atlas/atlas.urdf",
      "/home/junhyeok/Repository/PyPnC/robot_model/atlas", false, true);

  Eigen::Vector3d com_pos(0, 0, 0);
  Eigen::Vector3d com_lv(0.1, 0, 0);
  Eigen::Vector3d com_av(0.1, 0, 0);
  Eigen::Quaternion<double> com_quat(1, 0, 0, 0);
  Eigen::Vector3d bjoint_pos(0, 0, 0);
  Eigen::Vector3d bjoint_lv(0.1, 0, 0);
  Eigen::Vector3d bjoint_av(0.1, 0, 0);
  Eigen::Quaternion<double> bjoint_quat(1, 0, 0, 0);
  map<string, double> joint_pos = {
      {"back_bkx", 0},    {"back_bky", 0},    {"back_bkz", 0},
      {"l_arm_elx", 0},   {"l_arm_ely", 0},   {"l_arm_shx", 0},
      {"l_arm_shz", 0},   {"l_arm_wrx", 0},   {"l_arm_wry", 0},
      {"l_arm_wry2", 0},  {"l_leg_akx", 0},   {"l_leg_aky", 0},
      {"l_leg_hpx", 0},   {"l_leg_hpy", 0},   {"l_leg_hpz", 0},
      {"l_leg_kny", 0},   {"neck_ry", 0},     {"r_arm_elx", 0},
      {"r_arm_ely", 0},   {"r_arm_shx", 0},   {"r_arm_shz", 0},
      {"r_arm_wrx", 0},   {"r_arm_wry", 0},   {"r_arm_wry2", 0},
      {"r_leg_akx", 0.1}, {"r_leg_aky", 0.2}, {"r_leg_hpx", 0},
      {"r_leg_hpy", 0},   {"r_leg_hpz", 0},   {"r_leg_kny", 0}};
  map<string, double> joint_vel = {
      {"back_bkx", 0},    {"back_bky", 0},    {"back_bkz", 0},
      {"l_arm_elx", 0},   {"l_arm_ely", 0},   {"l_arm_shx", 0},
      {"l_arm_shz", 0},   {"l_arm_wrx", 0},   {"l_arm_wry", 0},
      {"l_arm_wry2", 0},  {"l_leg_akx", 0},   {"l_leg_aky", 0},
      {"l_leg_hpx", 0},   {"l_leg_hpy", 0},   {"l_leg_hpz", 0},
      {"l_leg_kny", 0},   {"neck_ry", 0},     {"r_arm_elx", 0},
      {"r_arm_ely", 0},   {"r_arm_shx", 0},   {"r_arm_shz", 0},
      {"r_arm_wrx", 0},   {"r_arm_wry", 0},   {"r_arm_wry2", 0},
      {"r_leg_akx", 0.1}, {"r_leg_aky", 0.2}, {"r_leg_hpx", 0},
      {"r_leg_hpy", 0},   {"r_leg_hpz", 0},   {"r_leg_kny", 0}};

  atlas.update_system(com_pos, com_quat, com_lv, com_av, bjoint_pos,
                      bjoint_quat, bjoint_lv, bjoint_av, joint_pos, joint_vel,
                      true);

  cout << "robot updated" << endl;

  cout << "r_sole position: " << endl;
  Eigen::Isometry3d iso = atlas.get_link_iso("r_sole");
  cout << iso.linear() << endl;
  cout << iso.translation() << endl;

  cout << "r_sole velocity" << endl;
  Eigen::MatrixXd vel = atlas.get_link_vel("r_sole");
  cout << vel << endl;

  /*cout << "Jacobian Matrix:" << endl;
  Eigen::MatrixXd jacobian = atlas.get_link_jacobian("r_sole");
  cout << jacobian <<  endl;

  cout << "base_local_com_pos:" << endl;
  Eigen::Vector3d localcom = atlas.get_base_local_com_pos();
  cout << localcom <<  endl;

  cout << "base_link_name:" << endl;
  string bLinkName = atlas.get_base_link_name();
  cout << bLinkName <<  endl;

  cout << "q_idx:" << endl;
  int q_idx = atlas.get_q_idx("back_bkx");
  cout << q_idx <<  endl;

  cout << "q_dot_idx:" << endl;
  int q_d_idx = atlas.get_q_dot_idx("back_bkx");
  cout << q_d_idx <<  endl;

  cout << "joint_idx:" << endl;
  int j_idx = atlas.get_joint_idx("back_bkx");
  cout << j_idx <<  endl;

  cout << "q:" << endl;
  Eigen::VectorXd _q = atlas.get_q();
  cout << _q  << endl;

  cout << "q_dot:" << endl;
  Eigen::VectorXd _q_dot = atlas.get_q_dot();
  cout << _q_dot  << endl;

  cout << "mass matrix:" << endl;
  Eigen::MatrixXd mass = atlas.get_mass_matrix();
  cout << mass << endl;

  cout << "gravity:" << endl;
  Eigen::VectorXd gravity = atlas.get_gravity();
  cout << gravity  << endl;

  cout << "coriolis:" << endl;
  Eigen::VectorXd cori = atlas.get_coriolis();
  cout << cori  << endl;*/

  cout << "com pos:" << endl;
  Eigen::Vector3d com = atlas.get_com_pos();
  cout << com << endl;

  cout << "com lin vel:" << endl;
  Eigen::Vector3d comlv = atlas.get_com_lin_vel();
  cout << comlv << endl;

  /*cout << "com jacobian:" << endl;
  Eigen::MatrixXd comJ = atlas.get_com_lin_jacobian();
  cout << comJ  << endl;

  cout << "com jacobian_dot:" << endl;
  Eigen::MatrixXd comJd = atlas.get_com_lin_jacobian_dot();
  cout << comJd << endl;*/

  cout << "link jacobian_dot times q_dot:" << endl;
  Eigen::MatrixXd linkJ = atlas.get_link_jacobian_dot_times_qdot("r_sole");
  cout << linkJ << endl;

  cout << "Ig" << endl;
  cout << atlas.Ig << endl;

  cout << "hg" << endl;
  cout << atlas.hg << endl;

  return 0;
}
