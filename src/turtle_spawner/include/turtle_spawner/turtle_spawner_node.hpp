#pragma once

#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "std_srvs/srv/empty.hpp"

#include "catch_them_all_interfaces/msg/turtle.hpp"
#include "catch_them_all_interfaces/msg/turtle_array.hpp"
#include "catch_them_all_interfaces/srv/catch_turtle.hpp"

namespace CatchThemAll {

    using turtle_obj = catch_them_all_interfaces::msg::Turtle;
    using turtle_array_obj = catch_them_all_interfaces::msg::TurtleArray;
    using catch_turtle_obj = catch_them_all_interfaces::srv::CatchTurtle;

    class TurtleSpawnerNode : public rclcpp::Node
    {
        public:
            TurtleSpawnerNode(const std::string& name);

            void init();
            void reset();
            void kill(const std::string_view& turtle_name);

        private:
            void spawnTurtle();
            void spawn(const std::string_view& name, const double x, const double y,
                       const double theta);

            void spawn(const turtle_obj& turtle) {
                spawn(turtle.name, turtle.x, turtle.y, turtle.theta);
            }

            void publishAliveTurtles();

            void catchTurtle(const catch_turtle_obj::Request::SharedPtr request,
                             const catch_turtle_obj::Response::SharedPtr response);

            template <typename T>
            static double freq2millis(const T freq) {
                static_assert(std::is_arithmetic_v<T>, "ERROR: The type must be numeric!");
                return static_cast<double>(1000.0 / freq);
            }

            template <typename T>
            struct named_param_t
            {
                T value;
                const std::string_view name;
            };

            rclcpp::TimerBase::SharedPtr timer_;

            named_param_t<double> pub_freq_{0.25, "publish_frequency"};
            named_param_t<int> max_spawn_turtles_{0, "max_spawn_turtles"};

            rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_reset_ = nullptr;

            rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_spawn_ = nullptr;
            rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_kill_ = nullptr;

            std::unordered_map<std::string, turtle_obj> turtles_;
            size_t turtles_counter_ = 0;

            rclcpp::Publisher<turtle_array_obj>::SharedPtr turtle_array_publisher_ = nullptr;
            rclcpp::Service<catch_turtle_obj>::SharedPtr turtle_catch_service_ = nullptr;

    };
}  // namespace CatchThemAll
