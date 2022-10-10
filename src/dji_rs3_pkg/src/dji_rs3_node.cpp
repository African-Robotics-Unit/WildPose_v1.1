#include "rclcpp/rclcpp.hpp"


class DjiRs3Node : public rclcpp::Node
{
public:
    DjiRs3Node() : Node("dji_rs3_node")
    {
        RCLCPP_INFO(this->get_logger(), "DJI RS3 Node started.");
    }

private:
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DjiRs3Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}