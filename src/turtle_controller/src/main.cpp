#include "turtle_controller/turtle_controller_node.hpp"

using namespace CatchThemAll;

int main(int argc, char **argv)
{
    int ret = 0;
    rclcpp::init(argc, argv);

    auto tc_node = std::make_shared<TurtleControllerNode>("turtle_controller_node");

    rclcpp::spin(tc_node);
    rclcpp::shutdown();

    return ret;
}
