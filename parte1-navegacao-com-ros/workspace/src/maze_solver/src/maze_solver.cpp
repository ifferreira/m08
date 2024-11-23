#include <iostream>
#include <vector>
#include <queue>
#include <memory>
#include <cmath>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

class AStarSolver : public rclcpp::Node
{
public:
    AStarSolver() : Node("a_star_solver")
    {
        client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
        move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

        while (!client_->wait_for_service(std::chrono::seconds(1)) || !move_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the services to become available...");
        }

        call_service();
    }

private:
    void call_service()
    {
        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();

        auto response_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = response_future.get();
            process_response(response);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call the service.");
        }
    }

    void process_response(const std::shared_ptr<cg_interfaces::srv::GetMap::Response> &response)
    {
        std::vector<unsigned char> flattened_map;
        for (const auto &str : response->occupancy_grid_flattened) {
            flattened_map.insert(flattened_map.end(), str.begin(), str.end());
        }
        std::vector<uint32_t> shape = {response->occupancy_grid_shape[0], response->occupancy_grid_shape[1]};

        RCLCPP_INFO(this->get_logger(), "Received a map of size %dx%d", shape[0], shape[1]);

        auto path = run_a_star(flattened_map, shape);

        if (!path.empty())
        {
            std::cout << "Path found: ";
            for (const auto &p : path)
            {
                std::cout << "(" << p.first << "," << p.second << ") ";
            }
            std::cout << std::endl;

            move_along_path(path);
        }
        else
        {
            std::cout << "No path found." << std::endl;
        }
    }

    std::vector<std::pair<int, int>> run_a_star(const std::vector<unsigned char> &occupancy_grid, const std::vector<uint32_t> &shape)
    {
        const int width = shape[0];
        const int height = shape[1];
        //const int start_x = 2, start_y = 2;
        const int goal_x = 17, goal_y = 17;

        int start_x = -1, start_y = -1;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (occupancy_grid[y * width + x] == 'r') {
                    start_x = x;
                    start_y = y;
                    break;
                }
            }
        if (start_x != -1) break;
        }

        struct Node
        {
            int x, y;
            float cost, priority;
            bool operator<(const Node &other) const { return priority > other.priority; }
        };

        std::priority_queue<Node> open_list;
        std::vector<bool> closed_list(width * height, false);
        std::vector<std::pair<int, int>> came_from(width * height, {-1, -1});
        auto index = [width](int x, int y) { return y * width + x; };

        auto heuristic = [](int x1, int y1, int x2, int y2) -> double {
            return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        };

        open_list.push({start_x, start_y, 0.0f, static_cast<float>(heuristic(start_x, start_y, goal_x, goal_y))});

        const std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

        while (!open_list.empty())
        {
            auto current = open_list.top();
            open_list.pop();

            if (current.x == goal_x && current.y == goal_y)
            {
                std::vector<std::pair<int, int>> path;
                for (auto at = std::make_pair(goal_x, goal_y); at != std::make_pair(-1, -1); at = came_from[index(at.first, at.second)])
                {
                    path.push_back(at);
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            if (closed_list[index(current.x, current.y)])
                continue;

            closed_list[index(current.x, current.y)] = true;

            for (const auto &dir : directions)
            {
                int nx = current.x + dir.first;
                int ny = current.y + dir.second;

                if (nx >= 0 && ny >= 0 && nx < width && ny < height && !closed_list[index(nx, ny)] &&
                    occupancy_grid[index(nx, ny)] != 'b')
                {
                    float new_cost = current.cost + 1;
                    float priority = new_cost + static_cast<float>(heuristic(nx, ny, goal_x, goal_y));
                    open_list.push({nx, ny, new_cost, priority});
                    came_from[index(nx, ny)] = {current.x, current.y};
                }
            }
        }

        return {};
    }

    void move_along_path(const std::vector<std::pair<int, int>> &path)
    {
        for (size_t i = 1; i < path.size(); ++i)
        {
            std::string direction;
            int dx = path[i].first - path[i - 1].first;
            int dy = path[i].second - path[i - 1].second;

            if (dx == 1 && dy == 0) direction = "right";
            else if (dx == -1 && dy == 0) direction = "left";
            else if (dx == 0 && dy == 1) direction = "down";
            else if (dx == 0 && dy == -1) direction = "up";

            send_move_command(direction);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void send_move_command(const std::string &direction)
    {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;

        auto future = move_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Moved %s. Success: %d", direction.c_str(), response->success);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move %s", direction.c_str());
        }
    }

    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto solver = std::make_shared<AStarSolver>();
    rclcpp::spin(solver);

    rclcpp::shutdown();
    return 0;
}

