#include "robot_system/pinocchio_robot_system.cpp"
#include <iostream>
#include <cmath>
//#include "robot_system/dart_robot_system.cpp"



using namespace std;


int main(int argc, char *argv[]) {

  //DartRobotSystem twoLinkRobot("/home/narwhal22/robot_system_sandbox/two_link_manipulator.urdf", true, true);
  PinocchioRobotSystem twoLinkRobot ("/home/narwhal22/robot_system_sandbox/two_link_manipulator.urdf","/home/narwhal22/robot_system_sandbox/two_link_manipulator.urdf", true, true);

  Eigen::Vector3d com_pos(0,0,0);
  Eigen::Vector3d com_lv(0,0,0);
  Eigen::Vector3d com_av(0,0,0);
  Eigen::Quaternion<double> com_quat(1,0,0,0);
  Eigen::Vector3d bjoint_pos(1,0.5,0);
  Eigen::Vector3d bjoint_lv(2,3,0);
  Eigen::Vector3d bjoint_av(4,1,0);
  Eigen::Quaternion<double> bjoint_quat(1 ,0,0,0);
  map<string, double> joint_pos = {{"j0", M_PI/4}, {"j1", M_PI/4}};
  map<string, double> joint_vel = {{"j0", 0.5}, {"j1", 0.25}};

  twoLinkRobot.update_system(com_pos, com_quat, com_lv, com_av, bjoint_pos, bjoint_quat, bjoint_lv, bjoint_av, joint_pos, joint_vel, true);

  cout << "robot updated" << endl;

  cout << "End-effector position: " << endl;
  Eigen::Isometry3d iso = twoLinkRobot.get_link_iso("ee");
  cout << iso.linear() << endl;
  cout << iso.translation() << endl;

  cout << "End-effector velocity" << endl;
  Eigen::MatrixXd vel = twoLinkRobot.get_link_vel("ee");
  cout << vel << endl;

  cout << "Jacobian Matrix:" << endl;
  Eigen::MatrixXd jacobian = twoLinkRobot.get_link_jacobian("ee");
  cout << jacobian <<  endl;

  cout << "base_local_com_pos:" << endl;
  Eigen::Vector3d localcom = twoLinkRobot.get_base_local_com_pos();
  cout << localcom <<  endl;

  cout << "base_link_name:" << endl;
  string bLinkName = twoLinkRobot.get_base_link_name();
  cout << bLinkName <<  endl;

  cout << "q_idx:" << endl;
  int q_idx = twoLinkRobot.get_q_idx("ee");
  cout << q_idx <<  endl;

  cout << "q_dot_idx:" << endl;
  int q_d_idx = twoLinkRobot.get_q_dot_idx("ee");
  cout << q_d_idx <<  endl;

  cout << "joint_idx:" << endl;
  int j_idx = twoLinkRobot.get_joint_idx("ee");
  cout << j_idx <<  endl;

  twoLinkRobot._update_centroidal_quantities();

  cout << "q:" << endl;
  Eigen::VectorXd _q = twoLinkRobot.get_q();
  cout << _q  << endl;

  cout << "q_dot:" << endl;
  Eigen::VectorXd _q_dot = twoLinkRobot.get_q_dot();
  cout << _q_dot  << endl;

  cout << "mass matrix:" << endl;
  Eigen::MatrixXd mass = twoLinkRobot.get_mass_matrix();
  cout << mass << endl;

  cout << "gravity:" << endl;
  Eigen::VectorXd gravity = twoLinkRobot.get_gravity();
  cout << gravity  << endl;

  cout << "coriolis:" << endl;
  Eigen::VectorXd cori = twoLinkRobot.get_coriolis();
  cout << cori  << endl;

  cout << "com pos:" << endl;
  Eigen::Vector3d com = twoLinkRobot.get_com_pos();
  cout << com << endl;

  cout << "com lin vel:" << endl;
  Eigen::Vector3d comlv = twoLinkRobot.get_com_lin_vel();
  cout << _q  << endl;

  cout << "com jacobian:" << endl;
  Eigen::MatrixXd comJ = twoLinkRobot.get_com_lin_jacobian();
  cout << comJ  << endl;

  cout << "com jacobian_dot:" << endl;
  Eigen::MatrixXd comJd = twoLinkRobot.get_com_lin_jacobian_dot();
  cout << comJd << endl;

  cout << "link jacobian_dot times q_dot:" << endl;
  Eigen::MatrixXd linkJ = twoLinkRobot.get_link_jacobian_dot_times_qdot("ee");
  cout << linkJ << endl;


  return 0;
}
