#include <torque_controller.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "nturt_torque_cmd_node");

  auto n = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

  TC my_tc(n);

  ros::Rate loop_rate(1000);

  while (ros::ok()) {
    // std::cout << "here" << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}