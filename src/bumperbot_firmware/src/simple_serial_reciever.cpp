#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>

#include <chrono>

using namespace std::chrono_literals;

class SimpleSerialReciever : public rclcpp::Node
{
public:
    SimpleSerialReciever() : Node("simple_serial_reciever")
    {   
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        port_ = get_parameter("port").as_string();
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        pub_ = create_publisher<std_msgs::msg::String>("serial_reciever", 10);
        timer_ = create_wall_timer(0.01s, std::bind(&SimpleSerialReciever::timerCallback, this));
        RCLCPP_INFO(get_logger(), "Publishing at 100 Hz");
    }

    ~SimpleSerialReciever()
    {
        arduino_.Close();
    }

    void timerCallback()
    {
        auto message = std_msgs::msg::String();
        if(rclcpp::ok() && arduino_.IsDataAvailable())
        {
            arduino_.ReadLine(message.data);
        }
        pub_->publish(message);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    LibSerial::SerialPort arduino_;
    std::string port_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSerialReciever>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}