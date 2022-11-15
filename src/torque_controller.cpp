#include "torque_controller.hpp"

TorqueController::TorqueController(std::shared_ptr<ros::NodeHandle> &_nh) : 
    nh_(_nh), timestemp_last_(ros::Time::now().toSec()),
    publish_frame_pub_(_nh->advertise<std_msgs::String>("/publish_can_frame", 10)),
    update_data_pub_(_nh->advertise<nturt_ros_interface::UpdateCanData>("/update_can_data", 10)),
    state_sub_(_nh->subscribe("/node_state", 10, &TorqueController::onState, this)),
    get_data_clt_(_nh->serviceClient<nturt_ros_interface::GetCanData>("/get_can_data")),
    register_clt_(_nh->serviceClient<nturt_ros_interface::RegisterCanNotification>("/register_can_notification")) {

    // register to can parser
    // wait until "/register_can_notification" service is avalible
    if(!ros::service::waitForService("/register_can_notification", 10000)) {
        ROS_FATAL("register to can parser timeout after 10 seconds");
        ros::shutdown();
    }
    // construct register call
    nturt_ros_interface::RegisterCanNotification register_srv;
    register_srv.request.node_name = ros::this_node::getName();
    /*
    data name registering to be notified
    brake -> brake level (front box 2)
    gear_dial -> inverter mode (dashboard)
    accelerator_1 -> accelerator level 1 (front box 2)
    accelerator_2 -> accelerator level 2 (front box 2)
    brake_micro -> brake trigger (front box 2)
    accelerator_micro -> accelerator trigger (front box 2)
    motor_speed -> motor speed (mcu_motor_speed)
    */
    register_srv.request.data_name = {
        "brake",
        "gear_dial",
        "accelerator_1",
        "accelerator_2",
        "brake_micro",
        "accelerator_micro",
        "motor_speed"
    };
    // call service
    if(!register_clt_.call(register_srv)) {
        ROS_FATAL("register to can parser failed");
        ros::shutdown();
    }

    // subscribe to the register topic
    notification_sub_ = nh_->subscribe(register_srv.response.topic, 10, &TorqueController::onNotification, this);
}

void TorqueController::update() {
    double timestemp = ros::Time::now().toSec();
    double dt = timestemp - timestemp_last_;
    
    // get accelerator travel after checking pedal plausibility
    double accelerator_travel = plausibility_check(dt);

    // get torque command after considering soft start and gear dial
    double torque_command = soft_start(accelerator_travel, dt);

    // update mcu command data
    nturt_ros_interface::UpdateCanData update_msg;
    // torque, direction command, inverter enable
    // disable inverter when node is not activate or pedal plausibility check error
    if(!is_activated_ || apps_error_ || bse_error_ || bppc_error_) {
        update_msg.name = "torque_command";
        update_msg.data = 0;
        update_data_pub_.publish(update_msg);
        update_msg.name = "direction_command";
        update_msg.data = 0;
        update_data_pub_.publish(update_msg);
        update_msg.name = "inverter_enable";
        update_msg.data = 0;
        update_data_pub_.publish(update_msg);
    }
    else {
        update_msg.name = "torque_command";
        update_msg.data = torque_command;
        update_data_pub_.publish(update_msg);
        update_msg.name = "direction_command";
        update_msg.data = 1;
        update_data_pub_.publish(update_msg);
        update_msg.name = "inverter_enable";
        update_msg.data = 1;
        update_data_pub_.publish(update_msg);
    }

    timestemp_last_ = timestemp;
}

std::string TorqueController::get_string() const {
    return std::string("torque_controller state:") +
        "\n\tmessage in:" +
        "\n\t\taccelerator_level_a: " + std::to_string(accelerator_level_a_) +
        "\n\t\taccelerator_level_b: " + std::to_string(accelerator_level_b_) +
        "\n\t\tbrake_level: "+ std::to_string(brake_level_) +
        "\n\t\taccelerator_trigger: " + (accelerator_triger_ ? "true" : "false") +
        "\n\t\tbrake_trigger: " + (brake_trigger_ ? "true" : "false") +
        "\n\t\tmotor_speed: " + std::to_string(motor_speed_) +
        "\n\t\tgear_dial: " + std::to_string(gear_dial_) +
        "\n\t\tis_activated: " + (is_activated_ ? "true" : "false") +
        "\n\tinternal state:" +
        "\n\t\tapps_error: " + (apps_error_ ? "true" : "false") +
        "\n\t\tapps_duration: " + std::to_string(apps_duration_) +
        "\n\t\tbse_error: " + (bse_error_ ? "true" : "false") +
        "\n\t\tbse_duration: " + std::to_string(bse_duration_) +
        "\n\t\tbppc_error: " + (bppc_error_ ? "true" : "false") +
        "\n\t\ttorque_command_last: " + std::to_string(torque_command_last_);
}

void TorqueController::onNotification(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg) {
    if(_msg->name == "brake") {
        brake_level_ = _msg->data;
    }
    else if(_msg->name == "gear_dial") {
        gear_dial_ = _msg->data;
    }
    else if(_msg->name == "accelerator_1") {
        accelerator_level_a_ = _msg->data;
    }
    else if(_msg->name == "accelerator_2") {
        accelerator_level_b_ = _msg->data;
    }
    else if(_msg->name == "brake_micro") {
        brake_trigger_ = _msg->data;
    }
    else if(_msg->name == "accelerator_micro") {
        accelerator_triger_ = _msg->data;
    }
    else if(_msg->name == "motor_speed") {
        motor_speed_ = _msg->data;
    }
}

void TorqueController::onState(const std_msgs::Bool::ConstPtr &_msg) {
    is_activated_ = _msg->data;
}

double TorqueController::plausibility_check(double _dt) {
    double accelerator_travel_a = (accelerator_level_a_ - 1) / 254.0;
    double accelerator_travel_b = (accelerator_level_b_ - 1) / 254.0;

    // use minium accelerator travel data
    double accelerator_travel = std::min(accelerator_travel_a, accelerator_travel_b);

    // accelerator pedals position sensor
    // if a pedal malfunction or two accelerator travel disagree
    if(accelerator_level_a_ == 0 || accelerator_level_a_ == 255 ||
       accelerator_level_b_ == 0 || accelerator_level_b_ == 255 ||
       std::abs(accelerator_travel_a - accelerator_travel_b) > apps_travel_threshold_) {
        // not increment apps_duration when apps error is triggered
        if(apps_duration_ > apps_duration_threshold_) {
            // set apps_error state to true to disable inverter
            apps_error_ = true;
            return 0;
        }
        else {
            apps_duration_ += _dt;
        }
    }
    else if(apps_duration_ > 0.001) {
        // discount apps_duration if it is postive
        apps_duration_ = std::max(0.0, apps_duration_ - apps_duration_discount_ * _dt);
        return 0;
    }
    else {
        // release accelerator pedal position sensor error
        apps_error_ = false;
    }

    // brake system encoder
    // if a pedal malfunction
    if(brake_level_ == 0 || brake_level_ == 255) {
        // not increment bse_duration when bse error is triggered
        if(bse_duration_ > bse_duration_threshold_) {
            // set bse_error state to true to disable inverter
            bse_error_ = true;
            return 0;
        }
        else {
            bse_duration_ += _dt;
        }
    }
    else if(bse_duration_ > 0.001) {
        // discount bse_duration if it is postive
        bse_duration_ = std::max(0.0, bse_duration_ - bse_duration_discount_ * _dt);
        return 0;
    }
    else {
        // release brake system encoder error
        bse_error_ = false;
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

double TorqueController::soft_start(double _accelerator_travel, double _dt) {
    double torque_command = _accelerator_travel * (gear_dial_ ? torque_max_slow_ : torque_max_);
    // if trigger soft start
    if(motor_speed_ < soft_start_threshold_) {
        double torque_command_threshold = std::min(torque_max_, torque_command_last_ + soft_start_torque_slope_ * _dt);
        if(torque_command > torque_command_threshold) {
            torque_command_last_ = torque_command_threshold;
            return torque_command_last_;
        }
    }
    torque_command_last_ = torque_command;
    return torque_command_last_;
}
