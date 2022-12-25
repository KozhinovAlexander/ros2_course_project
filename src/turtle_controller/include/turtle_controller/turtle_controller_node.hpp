#pragma once

#include "rclcpp/rclcpp.hpp"

#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "catch_them_all_interfaces/msg/turtle.hpp"
#include "catch_them_all_interfaces/msg/turtle_array.hpp"
#include "catch_them_all_interfaces/srv/catch_turtle.hpp"

namespace CatchThemAll {

    using turtle_obj = catch_them_all_interfaces::msg::Turtle;
    using turtle_array_obj = catch_them_all_interfaces::msg::TurtleArray;
    using catch_turtle_obj = catch_them_all_interfaces::srv::CatchTurtle;

    class TurtleControllerNode : public rclcpp::Node
    {
        public:
            TurtleControllerNode(const std::string& name);

        private:
            struct turtle_state_t {
                rclcpp::Time time;
                turtlesim::msg::Pose pose;
                geometry_msgs::msg::Twist twist;
            };

            struct catch_turtle_t {
                bool is_active;
                turtle_obj turtle;
            };
            catch_turtle_t catch_turtle_;

            void controlTurtleLoop();
            void control(turtle_state_t& ctrl_turtle_state, const catch_turtle_t& catch_turtle);

            void controlTurtlePoseChangeCallback(const turtlesim::msg::Pose::SharedPtr msg);
            void aliveTurtlesCallback(const turtle_array_obj::SharedPtr msg);
            void catchTurtle(const std::string_view& name);

            template <typename T>
            static double freq2millis(const T freq) {
                static_assert(std::is_arithmetic_v<T>, "ERROR: The type must be numeric!");
                return static_cast<double>(1000.0 / freq);
            }

            rclcpp::TimerBase::SharedPtr control_timer_ = nullptr;

            template <typename T>
            struct named_param_t
            {
                T value;
                const std::string_view name;
            };

            named_param_t<double> control_frequency_{1.0, "control_frequency"};
            named_param_t<std::string> control_turtle_name_{"turtle1", "control_turtle_name"};

            template <typename SubscriberT, typename PublisherT>
            struct control_turtle_t
            {
                typename rclcpp::Subscription<SubscriberT>::SharedPtr subscriber = nullptr;
                typename rclcpp::Publisher<PublisherT>::SharedPtr publisher = nullptr;
                turtle_state_t state;
            };
            control_turtle_t<turtlesim::msg::Pose, geometry_msgs::msg::Twist> control_turtle_;

            rclcpp::Subscription<turtle_array_obj>::SharedPtr alive_turtles_subscriber_ = nullptr;

            rclcpp::Client<catch_turtle_obj>::SharedPtr turtle_catch_srv_client_ = nullptr;

    };
}  // namespace CatchThemAll
