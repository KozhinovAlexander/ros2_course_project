#include <chrono>
#include <string>
#include <numbers>

#include "turtle_controller/turtle_controller_node.hpp"

#define DEFAULT_SUBSCRIPTION_QUEUE_SIZE    20

#define SERVICE_RESPONSE_WAIT_TIME_MSEC     1000ms
#define SERVICE_WAIT_DELAY_MSEC             2000ms

using namespace CatchThemAll;
using namespace std::chrono_literals;

constexpr auto FAIL_SRV_MSG = "Failed to get response from the server! %s";


TurtleControllerNode::TurtleControllerNode(const std::string& name) : Node(std::move(name))
{
    this->declare_parameter(control_frequency_.name.data(), control_frequency_.value);
    control_frequency_.value = std::move(this->get_parameter(control_frequency_.name.data()).as_double());

    this->declare_parameter(control_turtle_name_.name.data(), control_turtle_name_.value.data());
    control_turtle_name_.value = this->get_parameter(control_turtle_name_.name.data()).as_string();

    control_turtle_.state.time = rclcpp::Clock().now();
    control_turtle_.subscriber = this->create_subscription<turtlesim::msg::Pose>(
        control_turtle_name_.value + "/pose", DEFAULT_SUBSCRIPTION_QUEUE_SIZE,
        std::bind(&TurtleControllerNode::controlTurtlePoseChangeCallback, this, std::placeholders::_1));
    control_turtle_.publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        control_turtle_name_.value + "/cmd_vel", DEFAULT_SUBSCRIPTION_QUEUE_SIZE);

    alive_turtles_subscriber_ = this->create_subscription<turtle_array_obj>("alive_turtles", DEFAULT_SUBSCRIPTION_QUEUE_SIZE,
                std::bind(&TurtleControllerNode::aliveTurtlesCallback, this, std::placeholders::_1));

    auto control_period_millis = this->freq2millis(control_frequency_.value);
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(control_period_millis)),
                                             std::bind(&TurtleControllerNode::controlTurtleLoop, this));

    turtle_catch_srv_client_ = this->create_client<catch_turtle_obj>("catch_turtle");

    // Show start message:
    RCLCPP_INFO(this->get_logger(), "TurtleControllerNode has been started with node name \"%s\" "
                " and following parameters:\n"
                "\t\t -- %s: \"%s\"\n"
                "\t\t -- %s: %.3f ",
                this->get_name(),
                control_turtle_name_.name.data(), control_turtle_name_.value.c_str(),
                control_frequency_.name.data(), control_frequency_.value
                );
};

void TurtleControllerNode::controlTurtleLoop()
{
    control(control_turtle_.state, catch_turtle_);
    control_turtle_.publisher->publish(control_turtle_.state.twist);
}

void TurtleControllerNode::control(turtle_state_t& ctrl_turtle_state, const catch_turtle_t& catch_turtle)
{
    auto twist_next = geometry_msgs::msg::Twist();
    ctrl_turtle_state.time = rclcpp::Clock().now();
    [[ maybe_unused ]] const auto dt = (ctrl_turtle_state.time - ctrl_turtle_state.time).seconds();

    if (!catch_turtle_.is_active) {
        ctrl_turtle_state.twist = twist_next;
        return;
    }

    [[ maybe_unused ]] auto rad2deg = [](const auto &rad) -> auto { return (rad * 180.0) / M_PI; };
    [[ maybe_unused ]] auto transform_turtlesim_angle = [](const auto a) -> auto {
        auto new_a = a;
        if (new_a > M_PI) {
            new_a -= 2.0 * M_PI;
        } else if (a < -M_PI) {
            new_a += 2.0 * M_PI;
        }
        return new_a;
    };

    const auto dx = catch_turtle.turtle.x - ctrl_turtle_state.pose.x;
    const auto dy = catch_turtle.turtle.y - ctrl_turtle_state.pose.y;
    const auto d = std::sqrt(dx * dx + dy * dy);  // distance difference (e.g. distance error)

    const auto angle = std::atan2(dy, dx);  // angle between two turtles
    const auto angle_err = transform_turtlesim_angle(angle - ctrl_turtle_state.pose.theta);

    const auto v = 1.0 * d;
    const auto omega = 4.0 * angle_err;

    twist_next.linear.x = v;
    twist_next.angular.z = omega;

    #if 0
    RCLCPP_INFO(this->get_logger(), "---> %s: [ x: %.3f y: %.3f theta: %.3f° ]", catch_turtle.turtle.name.c_str(),
        catch_turtle.turtle.x, catch_turtle.turtle.y, rad2deg(catch_turtle.turtle.theta));
    RCLCPP_INFO(this->get_logger(), "---> %s: [ x: %.3f y: %.3f theta: %.3f° ] d: %.3f; "
        "angle: %.3f° angle_err: %.3f°", control_turtle_name_.value.c_str(), ctrl_turtle_state.pose.x, ctrl_turtle_state.pose.y,
        rad2deg(ctrl_turtle_state.pose.theta), d, rad2deg(angle), rad2deg(angle_err));
    #endif

    constexpr auto distance_tolerance = 0.5;
    if (d <= distance_tolerance) {
        catchTurtle(catch_turtle.turtle.name);
    }
    ctrl_turtle_state.twist = twist_next;
}

void TurtleControllerNode::controlTurtlePoseChangeCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
    control_turtle_.state.pose.x = msg->x;
    control_turtle_.state.pose.y = msg->y;
    control_turtle_.state.pose.theta = msg->theta;
}

void TurtleControllerNode::aliveTurtlesCallback(const turtle_array_obj::SharedPtr msg)
{
    const auto alive_turtles_array = msg->turtle_array;

    const auto idx = alive_turtles_array.size() - 1;  // just select last one
    catch_turtle_.turtle.name = msg->turtle_array.at(idx).name;
    catch_turtle_.turtle.x = msg->turtle_array.at(idx).x;
    catch_turtle_.turtle.y = msg->turtle_array.at(idx).y;
    catch_turtle_.turtle.theta = msg->turtle_array.at(idx).theta;
    catch_turtle_.is_active = true;

    RCLCPP_INFO(this->get_logger(), "Catching turtle [%s]...", catch_turtle_.turtle.name.c_str());
}

void TurtleControllerNode::catchTurtle(const std::string_view& name)
{
    auto req = std::make_shared<catch_turtle_obj::Request>();
    req->name = std::move(name);

    auto client = this->create_client<catch_turtle_obj>("catch_turtle");
    while(!client->wait_for_service(SERVICE_WAIT_DELAY_MSEC));

    auto future = client->async_send_request(req);
    try {
        auto future_status = future.wait_for(SERVICE_RESPONSE_WAIT_TIME_MSEC);
        if (future_status == std::future_status::ready) {
            auto response = future.get();
            (void)response;
            // RCLCPP_INFO(this->get_logger(), "Catch Turtle: Success - %d", response->success);
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), FAIL_SRV_MSG, e.what());
    }

    RCLCPP_INFO(this->get_logger(), "Catched turtle [%s]!", name.data());
}
