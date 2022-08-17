#include "torque_controller.hpp"

TorqueController::TorqueController(std::shared_ptr<ros::NodeHandle> &_nh) : 
                                   nh_(_nh) , mcu_pub_(_nh->advertise<can_msgs::Frame>("sent_messages", 10)),
                                   can_sub_(_nh->subscribe("received_messages", 10, &TorqueController::onCan, this)),
                                   state_sub_(_nh->subscribe("node_state", 10, &TorqueController::onState, this)) {
    // Initialize can parser
    parser_.init_parser();

    // Initialize timestemp
    timestemp_last_ = ros::Time::now().toSec();

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
    // When recieving can signal from "FB2"
    if (id == _CAN_FB2) {
        // Get current time
        double timestemp = ros::Time::now().toSec();

        // Decode can message
        // pedal a
        int pedal_a = _msg->data[1];
        // pedel b
        int pedal_b = _msg->data[2];
        // torque command
        double torque_cmd;

        // get the correct data
        if((pedal_a == 0 || pedal_a == 255) && (pedal_b == 0 || pedal_b == 255)) {
            error_duration_ += timestemp - timestemp_last_;
            if(error_duration_ > error_duration_threshold_) {
                error_state_ = true;
            }
            else {
                torque_cmd = torque_cmd_last_;
            }
        }
        // when one pedel is compromised
        else if(pedal_a == 0) {
            torque_cmd = pedal_b / 254 * torque_max_;
            // decrease error duration
            error_duration_ = std::max(0.0, error_duration_ - error_duration_discount_ * (timestemp - timestemp_last_));
        }
        // when one pedel is compromised
        else if(pedal_b == 0) {
            torque_cmd = pedal_a / 254 * torque_max_;
            // decrease error duration
            error_duration_ = std::max(0.0, error_duration_ - error_duration_discount_ * (timestemp - timestemp_last_));
        }
        else {
            torque_cmd = std::min(pedal_a, pedal_b) / 254 * torque_max_;
            // decrease error duration
            error_duration_ = std::max(0.0, error_duration_ - error_duration_discount_ * (timestemp - timestemp_last_));
        }
        // Construct can message for mcu
        can_msgs::Frame can_msg;
        can_msg.id = _CAN_MCM;
        can_msg.header.stamp = _msg->header.stamp;
        
        // If ready to drive
        if (state_ && (!error_state_)) {
            // Trigger soft start
            if (motor_speed_ < motor_speed_threshold_) {
                double torque_threshold = torque_cmd_last_ + torque_slope_ * (timestemp - timestemp_last_);
                // If torque command increases too fast
                if (torque_cmd > torque_threshold && torque_max_ > torque_threshold) {
                    parser_.set_tbe("MTC", "N", torque_threshold);
                    torque_cmd_last_ = torque_threshold;
                }
                else {
                    parser_.set_tbe("MTC", "N", torque_cmd);
                    torque_cmd_last_ = torque_cmd;
                }
            }
            else {
                parser_.set_tbe("MTC", "N", torque_cmd);
                torque_cmd_last_ = torque_cmd;
            }
        }
        else {
            parser_.set_tbe("MTC", "N", 0);
            torque_cmd_last_ = 0;
        }
        parser_.encode(_CAN_MCM, can_msg.data);
        mcu_pub_.publish(can_msg);

        // Update control flags
        timestemp_last_ = timestemp;
        // printf("Output torque: %f\n", torque_cmd_last_);
    }

    // When recieving motor spped signal from "MMS"
    if (id == _CAN_MMS) {
        if (parser_.decode(_CAN_MMS, _msg->data) == OK) {
            motor_speed_ = parser_.get_afd("MMS", "N");
        }
    }
}

void TorqueController::onState(const std_msgs::Bool::ConstPtr &_msg) {
    state_ = _msg->data;
}

void TorqueController::test() {
    // Create a fake accelerator pedal signal
    can_msgs::Frame fake_msg;
    fake_msg.id = _CAN_FB2;
    fake_msg.header.stamp = ros::Time::now();
    fake_msg.data = {0, 0, 0, 0, 0, 0, 0, 0};

    // call onCan with fake message
    can_msgs::Frame::ConstPtr fake_msg_ptr = boost::make_shared<can_msgs::Frame>(fake_msg);
    onCan(fake_msg_ptr);
}
