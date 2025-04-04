#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <sys/select.h>
#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::string;
using std::vector;

// This class handles navigation goals
class SimpleNavigator : public rclcpp::Node
{
public:
  SimpleNavigator();

  bool go_to_location(const string &location_name);
  bool get_user_confirmation(const string &location_name, int timeout_sec);
  void run_navigation_loop();

private:
  // Action client for navigation
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  
  // Map of location names to coordinates (x, y, orientation z, w)
  std::map<string, std::tuple<float, float, float, float>> known_locations_;
};

// Constructor
SimpleNavigator::SimpleNavigator()
: Node("simple_navigator")
{
  // Create action client
  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");

  // Wait until server is ready
  while (!nav_client_->wait_for_action_server(2s)) {
    RCLCPP_INFO(get_logger(), "Waiting for the navigation server...");
  }

  // Define known positions with x, y, z, w
  known_locations_ = {
    {"kitchen", {4.28, -0.67, 0.7071, 0.7071}},
    {"table1", {3.40, 3.48, 0.7071, 0.7071}},
    {"table2", {-3.11, 3.38, -0.99, -0.03}},
    {"table3", {-2.88, -0.79, -0.99, 0.12}},
    {"home", {0.0, 0.0, 0.0, 1.57}}
  };
}

// Send robot to a location
bool SimpleNavigator::go_to_location(const string &location_name)
{
  if (known_locations_.find(location_name) == known_locations_.end()) {
    RCLCPP_ERROR(get_logger(), "Unknown location: %s", location_name.c_str());
    return false;
  }

  auto [x, y, z, w] = known_locations_[location_name];

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = this->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.z = z;
  goal.pose.pose.orientation.w = w;

  RCLCPP_INFO(get_logger(), "Navigating to %s (x=%.2f, y=%.2f)...", location_name.c_str(), x, y);

  // Send the goal and wait for response
  auto goal_future = nav_client_->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to send goal to server");
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Goal accepted. Waiting for result...");

  auto result_future = nav_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to get result");
    return false;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(get_logger(), "Successfully reached %s!", location_name.c_str());
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to reach %s", location_name.c_str());
    return false;
  }
}

// Ask the user to confirm the delivery
bool SimpleNavigator::get_user_confirmation(const string &location_name, int timeout_sec)
{
  RCLCPP_INFO(get_logger(), "Waiting for confirmation at %s (timeout: %d seconds)", location_name.c_str(), timeout_sec);
  std::cout << "Confirm delivery at " << location_name << "? (yes/no): ";
  fflush(stdout);  // Ensure prompt is shown

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);

  struct timeval tv;
  tv.tv_sec = timeout_sec;
  tv.tv_usec = 0;

  int retval = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
  if (retval == -1) {
    perror("select()");
    return false;
  } else if (retval == 0) {
    RCLCPP_WARN(get_logger(), "No input. Timeout at %s.", location_name.c_str());
    return false;
  } else {
    std::string input;
    std::getline(std::cin, input);
    if (input == "yes") {
      RCLCPP_INFO(get_logger(), "Confirmed at %s", location_name.c_str());
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "Not confirmed at %s", location_name.c_str());
      return false;
    }
  }
}

// Main control loop
void SimpleNavigator::run_navigation_loop()
{
  while (rclcpp::ok()) {
    std::cout << "Enter table numbers (e.g., 1 2 3): ";
    string input;
    std::getline(std::cin, input);
    std::istringstream stream(input);
    vector<string> tables_to_visit;
    string token;

    while (stream >> token) {
      string table_name = "table" + token;
      if (known_locations_.count(table_name)) {
        tables_to_visit.push_back(table_name);
      } else {
        RCLCPP_WARN(get_logger(), "Invalid table number: %s", token.c_str());
      }
    }

    if (tables_to_visit.empty()) {
      RCLCPP_ERROR(get_logger(), "No valid table numbers entered.");
      continue;
    }

    // Go to kitchen first
    if (!go_to_location("kitchen")) continue;

    // Wait for user to confirm pickup
    if (!get_user_confirmation("kitchen", 15)) {
      go_to_location("home");
      continue;
    }

    bool order_cancelled = false;

    // Go to each table
    for (const auto &table : tables_to_visit) {
      if (!go_to_location(table)) continue;

      if (!get_user_confirmation(table, 15)) {
        RCLCPP_INFO(get_logger(), "Order at %s was cancelled!", table.c_str());
        order_cancelled = true;
      }
    }

    // If any order was cancelled, go back to kitchen
    if (order_cancelled) {
      RCLCPP_INFO(get_logger(), "Going back to kitchen due to cancellation.");
      go_to_location("kitchen");
    }

    // Finally go home
    go_to_location("home");
  }
}

// Main function
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto navigator_node = std::make_shared<SimpleNavigator>();
  navigator_node->run_navigation_loop();
  rclcpp::shutdown();
  return 0;
}
