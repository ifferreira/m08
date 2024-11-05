#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "cg_interfaces/srv/move_cmd.hpp"

class MoveCommandClient : public rclcpp::Node
{
public:
    MoveCommandClient()
    : Node("move_command_client")
    {
        client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
    }

    void send_request(const std::string& direction)
    {
		auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;

        auto future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded!");
            RCLCPP_INFO(this->get_logger(), "Success: %d", response->success);
            RCLCPP_INFO(this->get_logger(), "Robot Position: (%d, %d)", response->robot_pos[0], response->robot_pos[1]);
            RCLCPP_INFO(this->get_logger(), "Target Position: (%d, %d)", response->target_pos[0], response->target_pos[1]);
            RCLCPP_INFO(this->get_logger(), "Available directions: Left: %s, Down: %s, Up: %s, Right: %s",
                        response->left.c_str(), response->down.c_str(), response->up.c_str(), response->right.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int i = 0;
    while (i < 10) 
    {
        auto client_node = std::make_shared<MoveCommandClient>();

        std::string direction = "right";
        client_node->send_request(direction);
        i++;
    }

    rclcpp::shutdown();
    return 0;
}

