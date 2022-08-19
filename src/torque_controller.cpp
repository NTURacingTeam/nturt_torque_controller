#include "torque_controller.hpp"

TorqueController::TorqueController(std::shared_ptr<ros::NodeHandle> &_nh) : 
                                   nh_(_nh), timestemp_last_(ros::Time::now().toSec()),
                                   mcu_pub_(_nh->advertise<can_msgs::Frame>("sent_messages", 10)),
                                   can_sub_(_nh->subscribe("received_messages", 10, &TorqueController::onCan, this)),
                                   state_sub_(_nh->subscribe("node_state", 10, &TorqueController::onState, this)) {
    // initialize can parser
    parser_.init_parser();
    //initialize can data
    // can has data length of 8
    can_msg_.dlc = 8;
}

void TorqueController::update() {
    double timestemp = ros::Time::now().toSec();
    double time_period = timestemp - timestemp_last_;
    
    // get accelerator travel after checking pedal plausibility
    double accelerator_travel = plausibility_check(time_period);

    // get torque command after considering soft start
    double torque_command = soft_start(accelerator_travel, time_period);

    // construct can message
    can_msg_.header.stamp = ros::Time::now();
    parser_.set_tbe("MTC", "N", torque_command);
    parser_.encode(_CAN_MCM, can_msg_.data);
    // manually modify the data
    // motor direction to forward
    can_msg_.data[4] = 1;
    // disable inverter when node is not activate or pedal plausibility check error
    if(!is_activated_ || appc_error_ || bppc_error_) {
        can_msg_.data[5] = 0;
    }
    else {
        can_msg_.data[5] = 1;
    }
    // publish it
    mcu_pub_.publish(can_msg_);

    timestemp_last_ = timestemp;
}

std::string TorqueController::get_string() {
    return std::string("torque_controller_state:") +
           std::string("\n\tmessage in:") +
           std::string("\n\t\taccelerator_level_a: ") + std::to_string(accelerator_level_a_) +
           std::string("\n\t\taccelerator_level_b: ") + std::to_string(accelerator_level_b_) +
           std::string("\n\t\tbrake_level: ")+ std::to_string(brake_level_) +
           std::string("\n\t\taccelerator_trigger: ") + std::string(accelerator_triger_ ? "true" : "false") +
           std::string("\n\t\tbrake_trigger: ") + std::string(brake_trigger_ ? "true" : "false") +
           std::string("\n\t\tmotor_speed: ") + std::to_string(motor_speed_) +
           std::string("\n\t\tis_activated: ") + std::string(is_activated_ ? "true" : "false") +
           std::string("\n\tinternal state:") +
           std::string("\n\t\tappc_error: ") + std::string(appc_error_ ? "true" : "false") +
           std::string("\n\t\tappc_duration: ") + std::to_string(appc_duration_) +
           std::string("\n\t\tbppc_error: ") + std::string(bppc_error_ ? "true" : "false") +
           std::string("\n\t\ttorque_command_last: ") + std::to_string(torque_command_last_) + '\n';
}

void TorqueController::onCan(const can_msgs::Frame::ConstPtr &_msg) {
    // can signal from front box
    if(_msg->id == _CAN_FB2) {
        if(parser_.decode(_CAN_FB2, _msg->data) == OK) {
            // accelerator level a
            // accelerator_level_a_ = parser_.get_afd("THR", "A");
            accelerator_level_a_ = _msg->data[1];
            // accelerator level b
            // accelerator_level_b_ = parser_.get_afd("THR", "B");
            accelerator_level_b_ = _msg->data[2];
            // brake level
            // brake_level_ = _parser_.get_afd("BRK", "N");
            brake_level_ = _msg->data[0];
            // accelerator trigger
            accelerator_triger_ = std::bitset<8>(_msg->data[7])[0];
            // brake trigger
            brake_trigger_ = std::bitset<8>(_msg->data[7])[1];
        }
    }
    // can signal from inverter speed frame
    else if(_msg->id == _CAN_MMS) {
        if (parser_.decode(_CAN_MMS, _msg->data) == OK) {
            motor_speed_ = parser_.get_afd("MMS", "N");
        }
    }
}

void TorqueController::onState(const std_msgs::Bool::ConstPtr &_msg) {
    is_activated_ = _msg->data;
}

double TorqueController::plausibility_check(double _time_period) {
    double accelerator_travel_a = ((double)accelerator_level_a_ - 1.0) / 254;
    double accelerator_travel_b = ((double)accelerator_level_b_ - 1.0) / 254;

    // use minium accelerator travel data
    double accelerator_travel = std::min(accelerator_travel_a, accelerator_travel_b);

    // accelerator pedals plausibility check
    // if a pedal malfunction or two accelerator travel disagree
    if(std::abs(accelerator_level_a_ == 0 || accelerator_level_a_ == 255 ||
       accelerator_level_b_ == 0 || accelerator_level_b_ == 255 ||
       accelerator_travel_a - accelerator_travel_b) > appc_travel_threshold_) {
        // increase error duration
        appc_duration_ += _time_period;
        if(appc_duration_ > appc_duration_threshold_) {
            // set appc_error state to true to disable inverter
            appc_error_ = true;
            return 0;
        }
    }
    else if(appc_duration_ > 0.01) {
        // discount appc_duration if it is postive
        appc_duration_ = std::max(0.0, appc_duration_ - appc_duration_discount_ * _time_period);
        return 0;
    }
    else {
        // release accelerator pedal plausibility check error
        appc_error_ = false;
    }

    // brake pedal plausibility check
    // if brake pedal plausibility check is triggering
    if(bppc_error_) {
        if(accelerator_travel > bppc_release_threshold_) {
            return 0;
        }
        else {
            // release brake pedal plausibility check error
            bppc_error_ = false;
        }
    }
    else if(brake_trigger_ && accelerator_travel > bppc_trigger_threshold_) {
        // set bppc_error to true to disable inverter
        bppc_error_ = true;
        return 0;
    }
    return accelerator_travel;
}

double TorqueController::soft_start(double _accelerator_travel, double _time_period) {
    double torque_command = _accelerator_travel * torque_max_;
    // if trigger soft start
    if(motor_speed_ < soft_start_threshold_) {
        double torque_command_threshold = std::min(torque_max_, torque_command_last_ + soft_start_torque_slope_ * _time_period);
        if(torque_command > torque_command_threshold) {
            torque_command_last_ = torque_command_threshold;
            return torque_command_last_;
        }
    }
    torque_command_last_ = torque_command;
    return torque_command_last_;
}
