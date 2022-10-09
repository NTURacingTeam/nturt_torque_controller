// std include
#include <bitset>

// ros include
#include "ros/ros.h"
#include <can_msgs/Frame.h>
#include "std_msgs/Bool.h"

using namespace std;

class State_Controller {
    public:
        State_Controller(std::shared_ptr<ros::NodeHandle> &nh) : nh_(nh),
                         can_pub_(nh_->advertise<can_msgs::Frame>("received_messages", 10)),
                         state_pub_(nh_->advertise<std_msgs::Bool>("node_state", 10)) {
            std::cout << "node init" << std::endl;
        }
        void read() {
            cout << "read\n";
            string input;
            getline(cin, input);
            int num = stoi(input);
            switch (num) {
                case 0:
                    node_state();
                    break;
                case 1:
                    paddles10percent();
                    break;
                case 2:
                    brake_torque();
                    break;
                case 3:
                    slowstart();
                    break;    
                case 4:
                    initialize();
                    break;
                default:
                    cout << "enter 0, 1, 2 or 3\n";
            }
        }
    void node_state() {
        string input;
        cout << "Enter enable (0 for false, 1 for true)\n";
        getline(cin, input);
        bool e = stoi(input);
        std_msgs::Bool node_state;
        node_state.data = e;
        state_pub_.publish(node_state);
        std::cout << "Node state published" << std::endl;
    }

    void paddles10percent() {
        string input1, input2;
        cout << "Enter left torque\n";
        getline(cin, input1);
        cout << "Enter right torque\n";
        getline(cin, input2);
        uint8_t a = stoi(input1), b = stoi(input2);
        can_msgs::Frame front_box2;
        front_box2.header.stamp = ros::Time::now();
        front_box2.id = 0x080AD092;
        front_box2.is_extended = 1;
        front_box2.dlc = 8;
        front_box2.data = { 0, a, b, 0, 0, 0, 0, 0 };
        can_pub_.publish(front_box2);
        std::cout << "CAN published" << std::endl;
    }

    void brake_torque() {
        string input1, input2;
        cout << "Enter brake\n";
        getline(cin, input1);
        cout << "Enter torque\n";
        getline(cin, input2);
        uint8_t a = stoi(input1), b = stoi(input2);
        uint8_t trigger = 0;
        if (a != 0) {
            trigger += 2;
            std::cout << "trigger" << std::endl;
        }
        if (b != 0) {
            trigger += 1;
            std::cout << "trigger" << std::endl;
        }
        can_msgs::Frame front_box2;
        front_box2.header.stamp = ros::Time::now();
        front_box2.id = 0x080AD092;
        front_box2.is_extended = 1;
        front_box2.dlc = 8;
        front_box2.data = { a, b, b, 0, 0, 0, 0, trigger};
        can_pub_.publish(front_box2);
        std::cout << "CAN published" << std::endl;
    }

    void slowstart() {
        string input;
        cout << "Enter rpm\n";
        getline(cin, input);
        int rpm = stoi(input);
        can_msgs::Frame _can_msg;
        _can_msg.header.stamp = ros::Time::now();
        _can_msg.id = 0x0A5;
        _can_msg.is_extended = 0;
        _can_msg.dlc = 8;
        uint8_t a = rpm / 255, b = rpm % 255;
        _can_msg.data = {0, 0, a, b, 0 , 0, 0, 0};
        can_pub_.publish(_can_msg);
        std::cout << "CAN published" << std::endl;
    }

    void initialize() {
        can_msgs::Frame _can_msg;
        _can_msg.header.stamp = ros::Time::now();
        _can_msg.id = 0x080AD092;
        _can_msg.is_extended = 1;
        _can_msg.dlc = 8;
        _can_msg.data = {0, 1, 1, 0, 0 , 0, 0, 0};
        can_pub_.publish(_can_msg);
        std_msgs::Bool node_state;
        node_state.data = true;
        state_pub_.publish(node_state);
        std::cout << "Initialized" << std::endl;
    }
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Publisher state_pub_;
    ros::Publisher can_pub_;
};

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "nturt_can_test");
    auto node = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

    State_Controller my_sc(node);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        my_sc.read();
        loop_rate.sleep();
    }

  return 0;
}
