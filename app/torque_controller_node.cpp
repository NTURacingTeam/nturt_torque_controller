#include "torque_controller.hpp"

int main(int argc, char **argv) {
    // register as a ros node
    ros::init(argc, argv, "nturt_torque_command_node");

    // create a node handle
    std::shared_ptr<ros::NodeHandle> nh = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

    // initialize torque controller
    TorqueController torque_controller(nh);

    // frequancy 100 Hz
    ros::Rate loop_rate(100);

    // main loop
    while (ros::ok()) {
        ros::spinOnce();
        torque_controller.update();
        loop_rate.sleep();
    }

    return 0;
}
