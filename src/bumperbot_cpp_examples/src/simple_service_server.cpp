#include <rclcpp/rclcpp.hpp>
#include <bumperbot_msgs/srv/add_two_ints.hpp>

using namespace std::placeholders;

class SimpleServiceServer : public rclcpp::Node
{
public:
    SimpleServiceServer() : Node("simple_service_server")
    {
        service_ = create_service<bumperbot_msgs::srv::AddTwoInts>("add_two_ints", 
            std::bind(&SimpleServiceServer::serviceCallback, this, _1, _2));

        RCLCPP_INFO_STREAM(get_logger(), "Service add_two_ints ready");
    }

private:
    rclcpp::Service<bumperbot_msgs::srv::AddTwoInts>::SharedPtr service_;

    void serviceCallback(std::shared_ptr<bumperbot_msgs::srv::AddTwoInts::Request> req, 
                         std::shared_ptr<bumperbot_msgs::srv::AddTwoInts::Response> res)
    {
        RCLCPP_INFO_STREAM(get_logger(), "New Request Recieved a: " << req->a << "b: " << req->b);
        res->sum = req->a + req->b;
        RCLCPP_INFO_STREAM(get_logger(), "Response sum: " << res->sum);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}