#include <chrono>
#include <functional>
#include <memory>
#include <string>



#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/spawn.hpp>

class TurtleControlNode : public rclcpp::Node
{
public:
    TurtleControlNode() : Node("turtle_control_node")
    {
        // Create a client to call the spawn service
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");

        // Wait for the service to be available
        while (!spawn_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the spawn service to be available...");
        }

        // Create a request to spawn a new turtle
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();

        // implement here, so the turtle spawns in the middle of the śimulation





        

        // Call the service to spawn the turtle
        auto result = spawn_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Successfully spawned turtle2");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn turtle2");
        }

        // Create a publisher to control the turtle's velocity
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

        // Create a timer to publish velocity commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtleControlNode::publish_velocity, this));
    }

private:
    void publish_velocity()
    {

            // implement here


    }

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleControlNode>());
    rclcpp::shutdown();
    return 0;
}