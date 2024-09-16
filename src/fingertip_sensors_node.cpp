#include <iostream>
#include <vector>
#include <memory>

#include "uclv_dynamixel_utils/colors.hpp"

#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/fts3_sensors.hpp"

#include "serial/serial.h"

class FingertipSensors : public rclcpp::Node
{
public:
    rclcpp::Time time;

    // Parameters
    int millisecondsTimer_;  // Timer duration for publishing sensor data
    std::string serial_port_;  // Serial port for communication with the sensors
    int baudrate_;  // Baud rate for serial communication
    uint32_t serial_timeout_ = 1000;  // Timeout for serial communication
    bool timestamp = false;  // Flag to check if the sensor data contains a timestamp

    // Serial communication objects
    std::shared_ptr<serial::Serial> sensor_read_;
    std::string line;

    // Number of sensors and timestamp offset
    int num_sensors = 5;
    int var_timestamp = 0;

    // Vectors for storing sensor IDs and forces
    std::vector<uint16_t> ids;
    geometry_msgs::msg::Vector3 vec;

    // Topic name for publishing sensor state
    std::string sensor_state_topic_;

    // ROS publisher and timer
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Constructor
    FingertipSensors()
        : Node("fingertip_sensors"),
          millisecondsTimer_(this->declare_parameter<int>("millisecondsTimer", 2)),
          serial_port_(this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB1")),
          baudrate_(this->declare_parameter<int>("baudrate", 1000000)),
          sensor_state_topic_(this->declare_parameter<std::string>("sensor_state_topic", "sensor_state"))
    {
        // Set serial port to low latency
        setSerialPortLowLatency(serial_port_);
        sensor_read_ = std::make_shared<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(serial_timeout_));

        // Create publisher for sensor data
        publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(sensor_state_topic_, 1);

        // Flush and initialize the sensor
        sensor_read_->flush();
        write_on_serial("pausedata\r\n");
        sensor_read_->flush();
        write_on_serial("calibrate\r\n");
        write_on_serial("enabletime\r\n");
        write_on_serial("resume\r\n");

        // Create a timer to periodically publish sensor state
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(millisecondsTimer_),
            std::bind(&FingertipSensors::publish_state, this));
    }

private:
    // Function to set the serial port to low latency mode
    void setSerialPortLowLatency(const std::string &serial_port)
    {
        std::cout << "Setting low latency for " << WARN_COLOR << serial_port << CRESET << std::endl;
        std::string command = "setserial " + serial_port + " low_latency";
        int result = system(command.c_str());
        std::cout << "Set low latency for " << WARN_COLOR << serial_port << CRESET
                  << " result: " << SUCCESS_COLOR << result << CRESET << std::endl;
    }

    // Function to send a command over the serial connection
    void write_on_serial(const std::string &cmd)
    {
        sensor_read_->write(cmd);
        sensor_read_->waitByteTimes(100 * cmd.size());
    }

    // Function to read and publish sensor data
    void publish_state()
    {
        // Create a new message
        auto message = uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors();
        message.header.stamp = rclcpp::Clock{}.now();  // Get current time

        // Read data from the sensor
        line = sensor_read_->readline();
        if (line.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No data received from the sensor.");
            return;
        }

        // Tokenize the received line using commas as delimiters
        std::string token;
        std::vector<std::string> tokens;
        char delimiter = ',';
        std::stringstream ss(line);

        while (getline(ss, token, delimiter))
        {
            tokens.push_back(token);
        }
        tokens.pop_back();  // Remove any empty token at the end

        // Check if there is a timestamp in the data
        if (tokens.size() == 18)
        {
            timestamp = true;
            var_timestamp = 2;
        }

        // Resize the vectors to accommodate sensor data
        message.ids.resize(num_sensors);
        ids.resize(num_sensors);

        // Parse and assign sensor forces
        if (tokens[0] == "@")
        {
            for (size_t i = 1; i < 6; i++)
            {
                auto fx = tokens[var_timestamp - 2 + 3 * i];
                auto fy = tokens[var_timestamp - 1 + 3 * i];
                auto fz = tokens[var_timestamp + 0 + 3 * i];
                ids[i] = i;
                vec.x = std::stof(fx);
                vec.y = std::stof(fy);
                vec.z = std::stof(fz);
                message.forces.push_back(vec);
            }
            message.ids = ids;  // Assign sensor IDs to the message
        }

        // Publish the message
        publisher_->publish(message);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto fingertip_sensors_node = std::make_shared<FingertipSensors>();
        rclcpp::spin(fingertip_sensors_node);
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
