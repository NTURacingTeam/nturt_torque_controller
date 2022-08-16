#include "torque_controller.hpp"

TorqueController::TorqueController(std::shared_ptr<ros::NodeHandle> &_nh) : 
                                   nh_(_nh) , mcu_pub_(_nh->advertise<can_msgs::Frame>("sent_messages", 10)),
                                   can_sub_(_nh->subscribe("received_messages", 10, &TorqueController::onCan, this)) {
    // Initialize can parser
    parser_.init_parser();

    // Set default inverter command setting
    can_msgs::Frame can_msg;
    can_msg.id = _CAN_MCM;
    can_msg.header.stamp = ros::Time::now();
    // Disable inverter
    parser_.set_tbe("MIE", "N", 0);
    parser_.encode(_CAN_MCM, can_msg.data);
    // Publish
    mcu_pub_.publish(can_msg);
    
    // Enable inverter
    parser_.set_tbe("MIE", "N", 1);
    parser_.encode(_CAN_MCM, can_msg.data);
    // Forward direction
    parser_.set_tbe("MDC", "N", 1);
    parser_.encode(_CAN_MCM, can_msg.data);
    // Publish
    mcu_pub_.publish(can_msg);
}

void TorqueController::onCan(const can_msgs::Frame::ConstPtr &_msg) {
    int id = _msg->id;
    // only publish when recieving can signal from "FB2"
    if (id == _CAN_FB2) {
        if (parser_.decode(_CAN_FB2, _msg->data) == OK) {
            // send can message to mcu
            can_msgs::Frame can_msg;
            can_msg.id = _CAN_MCM;
            can_msg.header.stamp = _msg->header.stamp;
            torque_cmd_ = parser_.get_afd("THR", "A");
            // std::cout << torque_cmd_ << std::endl;
            parser_.set_tbe("MTC", "N", torque_cmd_ * 130);
            parser_.encode(_CAN_MCM, can_msg.data);
            mcu_pub_.publish(can_msg);
        }
    }
}

void TorqueController::shutdown() {
    can_msgs::Frame can_msg;
    can_msg.id = _CAN_MCM;
    can_msg.header.stamp = ros::Time::now();
    parser_.set_tbe("MIE", "N", 0);
    parser_.encode(_CAN_MCM, can_msg.data);
    mcu_pub_.publish(can_msg);
}

void TorqueController::test() {
    can_msgs::Frame can_msg;
    can_msg.id = _CAN_MCM;
    can_msg.header.stamp = ros::Time::now();
    // test message
    parser_.set_tbe("MTC", "N", 130);
    parser_.encode(_CAN_MCM, can_msg.data);
    // mcu_pub_.publish(can_msg);

    can_msg.id = _CAN_MCM;
    can_msg.header.stamp = ros::Time::now();
    parser_.set_tbe("MIE", "N", 1);
    parser_.encode(_CAN_MCM, can_msg.data);
    mcu_pub_.publish(can_msg);
}

void TorqueController::test1() {
    can_msgs::Frame can_msg;
    can_msg.id = _CAN_MCM;
    can_msg.header.stamp = ros::Time::now();
    parser_.set_tbe("MTC", "N", 0);
    parser_.encode(_CAN_MCM, can_msg.data);
    // mcu_pub_.publish(can_msg);
    
    can_msg.id = _CAN_MCM;
    can_msg.header.stamp = ros::Time::now();
    parser_.set_tbe("MIE", "N", 0);
    parser_.encode(_CAN_MCM, can_msg.data);
    mcu_pub_.publish(can_msg);
}
