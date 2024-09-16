#include <iostream>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"

#include "uclv_dynamixel_utils/hand.hpp"

using namespace uclv::dynamixel_utils;

class HandDriver : public rclcpp::Node
{
public:
    std::shared_ptr<Hand> hand_;               // Shared pointer to Hand object
    dynamixel::PortHandler *portHandler;       // Dynamixel PortHandler
    dynamixel::PacketHandler *packetHandler;   // Dynamixel PacketHandler

    int millisecondsTimer_;                    // Timer duration in milliseconds
    std::string serial_port_;                  // Serial port for communication
    int baudrate_;                             // Baudrate for communication
    float protocol_version_;                   // Protocol version for communication
    std::vector<int64_t> motor_ids_;           // List of motor IDs
    std::vector<int64_t> motor_thresholds_;    // Thresholds for motor positions

    std::string motor_state_topic_;            // Topic to publish motor states
    std::string desired_position_topic_;       // Topic to subscribe to desired motor positions

    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr publisher_;    // Publisher for motor positions
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr subscription_;  // Subscriber for desired positions
    rclcpp::TimerBase::SharedPtr timer_;       // Timer for publishing state

    HandDriver()
        : Node("hand_driver"),
          millisecondsTimer_(this->declare_parameter<int>("millisecondsTimer", 2)),
          serial_port_(this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0")),
          baudrate_(this->declare_parameter<int>("baudrate", 1000000)),
          protocol_version_(this->declare_parameter<float>("protocol_version", 2.0)),
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())),
          motor_thresholds_(this->declare_parameter<std::vector<int64_t>>("motor_thresholds", {0, 4095})),
          motor_state_topic_(this->declare_parameter<std::string>("motor_state_topic", "motor_state")),
          desired_position_topic_(this->declare_parameter<std::string>("desired_position_topic", "desired_position"))
    {
        hand_ = std::make_shared<Hand>(serial_port_, baudrate_, protocol_version_); // Initialize Hand
        hand_->setSerialPortLowLatency(serial_port_);   // Set serial port to low latency
        if (!hand_->initialize())
        {
            throw std::runtime_error("Error: Hand not initialized");
        }

        if (motor_ids_.empty())
        {
            throw std::runtime_error("Error: Motor IDs parameter is empty");
        }

        // Add motors based on their IDs
        for (const auto &id : motor_ids_)
        {
            if (id <= 0 || id >= 255)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid motor ID: %ld. Must be between 1 and 254.", id);
                throw std::invalid_argument("Invalid motor ID");
            }
            if (id > 30 && id < 34)
            {
                hand_->addWristMotor(id);   // Add wrist motor
            }
            if (id > 33 && id < 39)
            {
                hand_->addFingerMotor(id);  // Add finger motor
            }
        }

        // Create publisher for motor state
        publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(motor_state_topic_, 1);

        // Create subscription to desired motor positions
        subscription_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            desired_position_topic_, 1,
            std::bind(&HandDriver::topic_callback, this, std::placeholders::_1));

        // Timer to periodically publish the motor states
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(millisecondsTimer_),
            std::bind(&HandDriver::publish_state, this));
    }

private:
    // Function to publish motor states
    void publish_state()
    {
        std::vector<uint8_t> motor_ids_uint8t_vec;
        motor_ids_uint8t_vec.reserve(motor_ids_.size());
        for (size_t i = 0; i < motor_ids_.size(); i++)
        {
            motor_ids_uint8t_vec.push_back(static_cast<uint8_t>(motor_ids_[i]));
        }

        std::vector<uint32_t> motor_pos;
        try
        {
            motor_pos = hand_->readMotorsPositions(motor_ids_uint8t_vec);   // Read motor positions
        }
        catch (...)
        {
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "ERROR");
            return;
        }

        // Create message and populate with motor positions and IDs
        auto message = uclv_seed_robotics_ros_interfaces::msg::MotorPositions();
        message.positions.resize(motor_pos.size());
        message.ids = motor_ids_uint8t_vec;

        for (size_t i = 0; i < motor_pos.size(); i++)
        {
            message.positions[i] = static_cast<float>(motor_pos[i]);
        }

        publisher_->publish(message);   // Publish motor positions
    }

    // Callback function to handle incoming desired positions
    void topic_callback(const uclv_seed_robotics_ros_interfaces::msg::MotorPositions::SharedPtr pos)
    {
        for (size_t i = 0; i < pos->ids.size(); ++i)
        {
            auto id = pos->ids[i];
            auto position = pos->positions[i];

            // Check if the desired position is within the valid threshold range
            if (position < motor_thresholds_[0] || position > motor_thresholds_[1])
            {
                RCLCPP_ERROR(this->get_logger(), "Position %f for motor ID %d out of range [%ld, %ld]", position, id, motor_thresholds_[0], motor_thresholds_[1]);
                return;
            }
        }

        try
        {
            hand_->moveMotors(pos->ids, pos->positions);   // Move motors to the desired positions
        }
        catch (...)
        {
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "ERROR");
            return;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto hand_driver_node = std::make_shared<HandDriver>();
        rclcpp::spin(hand_driver_node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
