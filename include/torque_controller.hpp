#ifndef TORQUE_CONTROLLER_HPP
#define TORQUE_CONTROLLER_HPP

#include <NTURT_CAN_Parser.hpp>
#include <can_msgs/Frame.h>
#include <cp_can_id.hpp>
#include <memory>
#include <signal.h>

// ROS include
#include <ros/ros.h>

class Torque_Controller {
public:
  Torque_Controller(std::shared_ptr<ros::NodeHandle> &nh);

  void onCan(const can_msgs::Frame::ConstPtr &msg) {
    int id = msg->id;
    // std::cout << "id: " << id << std::endl;
    if (id == _CAN_FB2) {
      if (parser_.decode(_CAN_FB2, msg->data) == OK) {
        tq_cmd = parser_.get_afd("THR", "A");
        // send can message to the controller
        can_msgs::Frame cmdmsg;
        cmdmsg.id = _CAN_MCM;
        cmdmsg.header.stamp = msg->header.stamp;
        if (state_ == 1) {
          parser_.set_tbe("MTC", "N", tq_cmd);
          parser_.encode(_CAN_MCM, cmdmsg.data);
        }
        else {
          parser_.set_tbe("MTC", "N", 0);
          parser_.encode(_CAN_MCM, cmdmsg.data);
        }
        mcu_pub_.publish(cmdmsg);
        // std::cout << "torque command: " << tq_cmd << std::endl;
      }
    }
  }

  void State(const std_msgs::Bool::ConstPtr &msg)
  {
    state_ = msg->data;
  }

private:
  double tq_cmd = 0;
  bool state_ = 0;
  Parser parser_;
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher mcu_pub_;
  ros::Subscriber can_sub_;
  ros::Subscriber state_sub_;
};

Torque_Controller::Torque_Controller(std::shared_ptr<ros::NodeHandle> &nh)
    : nh_(nh) {
  std::cout << "node init" << std::endl;
  parser_.init_parser();
  mcu_pub_ = nh_->advertise<can_msgs::Frame>("sent_messages", 5);
  can_sub_ =
      nh_->subscribe("received_messages", 5, &Torque_Controller::onCan, this);
}

typedef Torque_Controller TC;
