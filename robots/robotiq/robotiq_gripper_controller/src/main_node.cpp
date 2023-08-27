#include <ros/ros.h>
#include <modbus/modbus.h>
#include "sensor_msgs/JointState.h"
#include "robotiq_gripper_msgs/GripperCommand.h"
#include "robotiq_gripper_msgs/GripperStatus.h"


class RobotiqGripperController
{
    public:
        RobotiqGripperController():
            pnh_("~")
        {
            std::string port_name = "";
            int baudrate = 0;
            double rate = 0.0;

            pnh_.param<std::string>("port_name", port_name, "/dev/ttyUSB0");
            pnh_.param<int>("baudrate", baudrate, 115200);
            pnh_.param<double>("rate", rate, 20.0);

            ROS_INFO("[%s] Port: [%s] and baudrate %d", ros::this_node::getName().c_str(), port_name.c_str(), baudrate);

            modbus_ = modbus_new_rtu(port_name.c_str(), baudrate, 'N', 8, 1);
            assert(modbus_ != NULL);
            modbus_set_slave(modbus_, 9);
            if(modbus_connect(modbus_) == -1)
            {
                modbus_free(modbus_);
                assert(false);
            }

            req_gripper_pos_ = 0;
            req_gripper_speed_ = 255;
            req_gripper_force_ = 100;


            uint16_t send_registers[3] = {0, 0, 0};
            modbus_write_registers(modbus_, 0x3E8, 3, send_registers);
            ros::Duration(0.5).sleep();


            sub_gripper_command_ = nh_.subscribe("gripper_command", 10, &RobotiqGripperController::callback_gripper_command, this);
            pub_gripper_status_ = nh_.advertise<robotiq_gripper_msgs::GripperStatus>("gripper_status", 10);
            pub_joint_states_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

            timer_pub_status_ = nh_.createTimer(ros::Duration(1/rate), &RobotiqGripperController::callback_timer_pub_status, this);
            ROS_INFO("[%s] initialized successfully.", ros::this_node::getName().c_str());
        }
        ~RobotiqGripperController()
        {

        }

    private:
        void callback_timer_pub_status(const ros::TimerEvent& event)
        {
            uint16_t send_registers[3] = {0, 0, 0};

            send_registers[0] = 0x0900;  // gACT and gGTO is always on
            send_registers[1] = req_gripper_pos_ & 0xFF;
            send_registers[2] = (req_gripper_speed_ << 8) + (req_gripper_force_);

            modbus_write_registers(modbus_, 0x3E8, 3, send_registers);
            ros::Duration(0.001).sleep();

            // Receive status from Gripper
            uint16_t recv_registers[3] = {0, };
            modbus_read_registers(modbus_, 0x7D0, 3, recv_registers);

            auto msg = robotiq_gripper_msgs::GripperStatus();
            msg.g_act = (uint8_t)(recv_registers[0] >> 8) & 0x01;
            msg.g_gto = (uint8_t)(recv_registers[0] >> 11) & 0x01;
            msg.g_sta = (uint8_t)(recv_registers[0] >> 12) & 0x03;
            msg.g_obj = (uint8_t)(recv_registers[0] >> 14) & 0x03;
            msg.g_flt = (uint8_t)(recv_registers[1] >> 8) & 0x0F;
            msg.g_pr = (uint8_t)(recv_registers[1]) & 0xFF;
            msg.g_po = (uint8_t)(recv_registers[2] >> 8) & 0xFF;
            msg.g_cu = (uint8_t)(recv_registers[2]) & 0xFF;

            pub_gripper_status_.publish(msg);

            auto js_msg = sensor_msgs::JointState();
            js_msg.header.stamp = ros::Time::now();
            js_msg.name.push_back("finger_joint");
            js_msg.position.push_back(msg.g_po / 255.0);
            js_msg.velocity.push_back(0.0);
            js_msg.effort.push_back(msg.g_cu / 255.0);

            pub_joint_states_.publish(js_msg);
        }

        void callback_gripper_command(const robotiq_gripper_msgs::GripperCommandConstPtr &msg)
        {
            if(msg->position > 1.0 && msg->position < 0.0)
            {
                ROS_WARN("[%s] Gripper positon is in [0...1]...", ros::this_node::getName().c_str());
                return;
            }

            if(msg->speed > 1.0 && msg->speed < 0.0)
            {
                ROS_WARN("[%s] Gripper speed is in [0...1]...", ros::this_node::getName().c_str());
                return;
            }

            if(msg->force > 1.0 && msg->force < 0.0)
            {
                ROS_WARN("[%s] Gripper force is in [0...1]...", ros::this_node::getName().c_str());
                return;
            }

            req_gripper_pos_ = (uint8_t)(msg->position * 255.0);
            req_gripper_speed_ = (uint8_t)(msg->speed * 255.0);
            req_gripper_force_ = (uint8_t)(msg->force * 255.0);

            ROS_INFO("%d %d %d", req_gripper_pos_, req_gripper_speed_, req_gripper_force_);
        }

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        modbus_t *modbus_;
        ros::Timer timer_pub_status_;

        uint8_t req_gripper_pos_;
        uint8_t req_gripper_speed_;
        uint8_t req_gripper_force_;

        ros::Publisher pub_joint_states_;
        ros::Publisher pub_gripper_status_;
        ros::Subscriber sub_gripper_command_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotiq_gripper_controller");
    RobotiqGripperController m;
    ros::spin();

    return 0;
}