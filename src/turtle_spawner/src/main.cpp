#include "turtle_spawner/turtle_spawner_node.hpp"

using namespace CatchThemAll;

int main(int argc, char **argv)
{
    int ret = 0;
    rclcpp::init(argc, argv);

    auto ts_node = std::make_shared<TurtleSpawnerNode>("turtle_spawner_node");
    ts_node->init();

    rclcpp::spin(ts_node);
    rclcpp::shutdown();

    return ret;
}
