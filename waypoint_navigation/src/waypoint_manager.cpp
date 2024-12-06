#include <rclcpp/rclcpp.hpp>
#include "rclcpp/executor.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include <vector>
#include <memory>


using namespace std::chrono_literals;

class WaypointNavigator : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using WaypointFollowerGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
    using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

    WaypointNavigator()
        : Node("waypoint_navigator"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_), server_timeout_(5*1000) {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&WaypointNavigator::joystick_cb, this, std::placeholders::_1));

        // non-spining node
        client_node_ = std::make_shared<rclcpp::Node>("_");
        // client
        waypoint_follower_action_client_ = rclcpp_action::create_client<FollowWaypoints>(client_node_,"follow_waypoints");
        navigation_action_client_ = rclcpp_action::create_client<NavigateToPose>(client_node_, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "WaypointNavigator node started.");
    }

private:
    // --------------Variables--------------
    // The NavigateToPose action client
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoint_follower_action_client_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_action_client_;

   
    NavigateToPose::Goal navigation_goal_{};
    FollowWaypoints::Goal waypoint_follower_goal_{};

    // Goal-related state
    WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
    NavigationGoalHandle::SharedPtr navigation_goal_handle_;
   
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Timeout value when waiting for action servers to respnd
    std::chrono::milliseconds server_timeout_;
    rclcpp::Node::SharedPtr client_node_;
    
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;

    // Debouncing
    rclcpp::Time last_button_0_press_ {this->now()};  // last press time button 0
    rclcpp::Time last_button_1_press_ {this->now()};  // last press time button 1
    rclcpp::Time last_button_2_press_ {this->now()};  // last press time button 2
    rclcpp::Time last_button_3_press_ {this->now()};  // last press time button 3
    
    rclcpp::Duration debounce_duration_ {std::chrono::milliseconds(500)};  // time tolerance (ms)

    // --------------Functions--------------
    void joystick_cb(const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto now = this->now();
        // Button 0 triggers the waypoint saving (button X)
        if (msg->buttons[0] == 1 && (now - last_button_0_press_) > debounce_duration_) {
            last_button_0_press_ = now;
            geometry_msgs::msg::PoseStamped current_pose;
            if (getRobotPose(current_pose)) {
                waypoints_.push_back(current_pose);
                RCLCPP_INFO(this->get_logger(), "Waypoint recorded at position: [%.2f, %.2f]",
                            current_pose.pose.position.x, current_pose.pose.position.y);
            }
        }

        // Button 1 deletes already saved waypoints (button Circle)
        if (msg->buttons[1] == 1 && (now - last_button_1_press_) > debounce_duration_) {
            last_button_1_press_ = now;
            RCLCPP_INFO(this->get_logger(), "Waypoints deleted.");
            waypoints_.clear();
        }

        // Button 2 stops the execution of waypoints (button Square)
        if (msg->buttons[2] == 1 && (now - last_button_2_press_) > debounce_duration_){
            last_button_2_press_ = now;
            RCLCPP_INFO(this->get_logger(), "Waypoint execution cancelled.");
            cancelWaypointExecution();
        }

        // Button 3 triggers the start of waypoint following (button Triangle)
        if (msg->buttons[3] == 1 && (now - last_button_3_press_) > debounce_duration_) {
            last_button_3_press_ = now;

            if (waypoints_.size()){
                RCLCPP_INFO(this->get_logger(), "Starting waypoint navigation...");
                startWaypointFollowing(waypoints_);
            } else {
                RCLCPP_INFO(this->get_logger(), "No waypoint is chosen!");
            }     
        }
       
    } // joystick_cb

    bool getRobotPose(geometry_msgs::msg::PoseStamped &pose) {
        try {
            // Transform from "map" to "base_link" to get the robot's pose in the map frame
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero);
            
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation = transform.transform.rotation;

            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
            return false;
        }
    }

    void startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses){
        auto is_action_server_ready = waypoint_follower_action_client_->wait_for_action_server(std::chrono::seconds(5));
        if (!is_action_server_ready) {
            RCLCPP_ERROR(this->get_logger(), "follow_waypoints action server is not available."
                " Is the initial pose set?");
            return;
        }

        // Send goal poses
        waypoint_follower_goal_.poses = poses;
        

        RCLCPP_DEBUG(this->get_logger(), "Sending a path of %zu waypoints:", waypoint_follower_goal_.poses.size());
        for (auto waypoint : waypoint_follower_goal_.poses) {
        RCLCPP_DEBUG(this->get_logger(), "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
        }

        // Enable result awareness by providing an empty lambda function
        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

        send_goal_options.result_callback = [this](auto) {waypoint_follower_goal_handle_.reset();
        };
        send_goal_options.feedback_callback = [this](WaypointFollowerGoalHandle::SharedPtr /*goal_handle*/,
        const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback) {feedback->current_waypoint;
        };

        auto future_goal_handle = waypoint_follower_action_client_->async_send_goal(waypoint_follower_goal_, send_goal_options);


        if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
            return;
        }

        // Get the goal handle and save so that we can check on completion in the timer callback
        waypoint_follower_goal_handle_ = future_goal_handle.get();
        if (!waypoint_follower_goal_handle_) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }
                
        std::cout << "Waiting for compl ..." << std::endl;

    } // startWaypointFollowing()

    void cancelWaypointExecution(){

        if (navigation_goal_handle_) {
            auto future_cancel = navigation_action_client_->async_cancel_goal(navigation_goal_handle_);

            if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
            rclcpp::FutureReturnCode::SUCCESS)
            {
            RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel goal");
            } else {
            navigation_goal_handle_.reset();
            }
        }

        if (waypoint_follower_goal_handle_) {
            auto future_cancel =
            waypoint_follower_action_client_->async_cancel_goal(waypoint_follower_goal_handle_);

            if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
            rclcpp::FutureReturnCode::SUCCESS)
            {
            RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel waypoint follower");
            } else {
            waypoint_follower_goal_handle_.reset();
            }
        }

    } // cancelWaypointExecution

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WaypointNavigator>();
    rclcpp::executors::SingleThreadedExecutor executor_;

    //rclcpp::spin(node);
    executor_.add_node(node);
    executor_.spin();

    rclcpp::shutdown();
    return 0;
}
