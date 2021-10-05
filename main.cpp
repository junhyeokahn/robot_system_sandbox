#include "robot_system/pinocchio_robot_system.cpp"
#include <iostream>
#include <cmath>
//#include "robot_system/dart_robot_system.cpp"



using namespace std;


int main(int argc, char *argv[]) {

  //DartRobotSystem twoLinkRobot("/home/narwhal22/robot_system_sandbox/two_link_manipulator.urdf", true, true);
  PinocchioRobotSystem pRobot ("a","b", true, true);

  /*Eigen::Vector3d com_pos(0,0,0);
  Eigen::Vector3d com_lv(0,0,0);
  Eigen::Vector3d com_av(0,0,0);
  Eigen::Quaternion<double> com_quat(1,0,0,0);
  Eigen::Vector3d bjoint_pos(1,0.5,0);
  Eigen::Vector3d bjoint_lv(2,3,0);
  Eigen::Vector3d bjoint_av(4,1,0);
  Eigen::Quaternion<double> bjoint_quat(1 ,0,0,0);
  map<string, double> joint_pos = {{"j0", M_PI/4}, {"j1", M_PI/4}};
  map<string, double> joint_vel = {{"j0", 0.5}, {"j1", 0.25}};

  twoLinkRobot.update_system(com_pos, com_quat, com_lv, com_av, bjoint_pos, bjoint_quat, bjoint_lv, bjoint_av, joint_pos, joint_vel, false);

  cout << "End-effector position: " << endl;
  Eigen::Isometry3d iso = twoLinkRobot.get_link_iso("ee");
  cout << iso.linear() << endl;
  cout << iso.translation() << endl;

  cout << "End-effector velocity" << endl;
  Eigen::Matrix vel = twoLinkRobot.get_link_vel("ee");
  cout << vel << endl;

  cout << "Jacobian Matrix:" << endl;
  Eigen::Matrix jacobian = twoLinkRobot.get_link_jacobian("ee");
  cout << jacobian <<  endl;


  cout << "robot updated" << endl;*/

  return 0;
}
