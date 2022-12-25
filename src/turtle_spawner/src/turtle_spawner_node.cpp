
#include <chrono>
#include <string>
#include <random>

#include "turtle_spawner/turtle_spawner_node.hpp"

#define POS_MIN     1.0
#define POS_MAX     10.0

#define THETA_MIN   -1.0
#define THETA_MAX   +1.0

#define MIN_NUM_TURTLES     2

#define SERVICE_WAIT_DELAY_MSEC             2000ms
#define SERVICE_RESPONSE_WAIT_TIME_MSEC     250ms

#define ALIVE_TURTLES_QUEUE_SIZE    20


using namespace CatchThemAll;
using namespace std::chrono_literals;

constexpr auto FAIL_SRV_MSG = "Failed to get response from the server! %s";


TurtleSpawnerNode::TurtleSpawnerNode(const std::string& name)
    : Node(std::move(name))
{
    turtles_.clear();

    this->declare_parameter(pub_freq_.name.data(), pub_freq_.value);
    pub_freq_.value = this->get_parameter(pub_freq_.name.data()).as_double();
    auto pub_millis = this->freq2millis(pub_freq_.value);

    this->declare_parameter(max_spawn_turtles_.name.data(), max_spawn_turtles_.value);
    max_spawn_turtles_.value = this->get_parameter(max_spawn_turtles_.name.data()).as_int();
    // Drop negative values - spawn at one two turtle:
    max_spawn_turtles_.value = max_spawn_turtles_.value > 0 ? max_spawn_turtles_.value : 1;
    assert(max_spawn_turtles_.value > 0);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(pub_millis)),
                                     std::bind(&TurtleSpawnerNode::spawnTurtle, this));

    turtle_array_publisher_ = this->create_publisher<turtle_array_obj>("alive_turtles",
                                ALIVE_TURTLES_QUEUE_SIZE);

    turtle_catch_service_ = this->create_service<catch_turtle_obj>("catch_turtle",
                                std::bind(&TurtleSpawnerNode::catchTurtle, this,
                                          std::placeholders::_1, std::placeholders::_2));

    client_reset_ = this->create_client<std_srvs::srv::Empty>("reset");
    client_spawn_ = this->create_client<turtlesim::srv::Spawn>("spawn");
    client_kill_ = this->create_client<turtlesim::srv::Kill>("kill");

    // Show start message:
    RCLCPP_INFO(this->get_logger(), "TurtleSpawnerNode has been started with name: \"%s\"",
                this->get_name());
};

 void TurtleSpawnerNode::init()
 {
    reset();
 }

void TurtleSpawnerNode::reset()
{
    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    while(!client_reset_->wait_for_service(SERVICE_WAIT_DELAY_MSEC));

    RCLCPP_INFO(this->get_logger(), "Resetting turtlesim...");
    auto future = client_reset_->async_send_request(req);

    try {
        auto future_status = future.wait_for(SERVICE_RESPONSE_WAIT_TIME_MSEC);
        if (future_status == std::future_status::timeout) {
            RCLCPP_INFO(this->get_logger(), "Resetting turtlesim: done!");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), FAIL_SRV_MSG, e.what());
    }
}

void TurtleSpawnerNode::kill(const std::string_view& turtle_name)
{
    auto req = std::make_shared<turtlesim::srv::Kill::Request>();
    req->name = std::move(turtle_name);

    while(!client_kill_->wait_for_service(SERVICE_WAIT_DELAY_MSEC));

    RCLCPP_INFO(this->get_logger(), "Killing turtle with the name \"%s\"...", turtle_name.data());
    auto future = client_kill_->async_send_request(req);
    try {
        auto future_status = future.wait_for(SERVICE_RESPONSE_WAIT_TIME_MSEC);
        if (future_status == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Killed turtle with the name \"%s\": done!",
                turtle_name.data());
        } else {
            RCLCPP_INFO(this->get_logger(), "Killed turtle with the name \"%s\": MAYBE done!",
                turtle_name.data());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), FAIL_SRV_MSG, e.what());
    }
}

void TurtleSpawnerNode::spawn(const std::string_view& name, const double x, const double y,
                              const double theta)
{
    // Create random turtle parameters:
    auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
    req->name = std::move(name);
    req->x = x;
    req->y = y;
    req->theta = theta;

    while(!client_spawn_->wait_for_service(SERVICE_WAIT_DELAY_MSEC));

    RCLCPP_INFO(this->get_logger(), "Spawn a turtle with the name \"%s\" at "
                "[x: %.3f, y: %.3f, theta: %.3f]...", req->name.c_str(), req->x, req->y, req->theta);
    auto future = client_spawn_->async_send_request(req);
    try {
        auto future_status = future.wait_for(SERVICE_RESPONSE_WAIT_TIME_MSEC);
        if (future_status == std::future_status::ready) {
            // Somehow the future status becomes never ready but timeouted instead
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Spawn a turtle with the name \"%s\" at "
                        "[x: %.3f, y: %.3f, theta: %.3f]: done - WITH Reponse", response->name.c_str(),
                        req->x, req->y, req->theta);
        } else {
            RCLCPP_INFO(this->get_logger(), "Spawn a turtle with the name \"%s\" at "
                        "[x: %.3f, y: %.3f, theta: %.3f]: done - NO Reponse", req->name.c_str(),
                        req->x, req->y, req->theta);
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), FAIL_SRV_MSG, e.what());
    }
}

void TurtleSpawnerNode::spawnTurtle()
{
    // Create random generator:
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> pos_gen(POS_MIN, POS_MAX);
    std::uniform_real_distribution<> theta_gen(THETA_MIN, THETA_MAX);

    if(turtles_.size() > (static_cast<size_t>(max_spawn_turtles_.value) - 1)) return;

    // Spawn turtle:
    turtle_obj t;
    do {
        t.name = std::string("catch_turtle_") + std::to_string(turtles_counter_);
        turtles_counter_++;
    } while(turtles_.find(t.name) != turtles_.end());

    t.x = pos_gen(gen);
    t.y = pos_gen(gen);
    t.theta = theta_gen(gen);
    turtles_.insert({t.name, t});
    spawn(turtles_[t.name]);

    publishAliveTurtles();
}

void TurtleSpawnerNode::publishAliveTurtles()
{
    auto msg = turtle_array_obj();
    auto turtle_arr = std::vector<turtle_obj>();
    std::transform(turtles_.begin(), turtles_.end(), std::back_inserter(turtle_arr),
                   [](const auto &e) { return e.second; });
    msg.set__turtle_array(std::move(turtle_arr));
    turtle_array_publisher_->publish(msg);
}

void TurtleSpawnerNode::catchTurtle(const catch_turtle_obj::Request::SharedPtr request,
                                    const catch_turtle_obj::Response::SharedPtr response)
{
    const auto catch_turtle_name = request->name;
    if(turtles_.find(catch_turtle_name) != turtles_.end()) {
        const auto kill_turtle = turtles_.extract(catch_turtle_name);
        kill(kill_turtle.key());
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Catched turtle with name \"%s\"", catch_turtle_name.c_str());
    } else {
        response->success = false;
        RCLCPP_WARN(this->get_logger(), "Unknown turtle with name \"%s\"", catch_turtle_name.c_str());
    }

}
