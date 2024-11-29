#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>
#include <memory>




class WaypointNavigator : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WaypointNavigator()
        : Node("waypoint_recorder"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&WaypointNavigator::joystick_cb, this, std::placeholders::_1));

        // action client for navigation
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "WaypointNavigator node started.");
    }

private:
    void joystick_cb(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Button 0 triggers the waypoint saving (button X)
        if (msg->buttons[0] == 1) {
            geometry_msgs::msg::PoseStamped current_pose;
            if (getRobotPose(current_pose)) {
                waypoints_.push_back(current_pose);
                RCLCPP_INFO(this->get_logger(), "Waypoint recorded at position: [%.2f, %.2f]",
                            current_pose.pose.position.x, current_pose.pose.position.y);
            }
        }
        // Button 3 triggers the start of waypoint following (button Triangle)
        if (msg->buttons[3] == 1) {
        RCLCPP_INFO(this->get_logger(), "Starting waypoint navigation...");
        followWaypoints();
        }
    }

    bool getRobotPose(geometry_msgs::msg::PoseStamped &pose) {
        try {
            // Listen for the transform from "map" to "base_link" to get the robot's pose in the map frame
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

    // Updating followWaypoints() to send waypoints as goals to the action server and receive feedback about the robot's progress and final status
    void followWaypoints() {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Navigation action server not available.");
            return;
        }

        for (const auto &waypoint : waypoints_) {
            RCLCPP_INFO(this->get_logger(), "Navigating to waypoint at (%.2f, %.2f)", 
                    waypoint.pose.position.x, waypoint.pose.position.y);

        auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose = waypoint;

            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            send_goal_options.feedback_callback =
                std::bind(&WaypointNavigator::feedback_cb, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback =
                std::bind(&WaypointNavigator::result_cb, this, std::placeholders::_1);

            auto goal_handle_future = nav_client_->async_send_goal(goal_msg, send_goal_options);

            // Wait for the robot to finish navigating to this waypoint
            rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future);
        }


        RCLCPP_INFO(this->get_logger(), "Finished following all waypoints.");
    }

    void feedback_cb(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Feedback: Robot is at position: [%.2f, %.2f]",
                    feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
    }

    void result_cb(const GoalHandleNavigateToPose::WrappedResult &result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Successfully reached the waypoint!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Waypoint navigation aborted.");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Waypoint navigation canceled.");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
            break;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    /* tf2_ros::Buffer Stores transformations and provides lookup functions to get the current position and orientation of the robot.
     The tf_buffer_ object keeps a history of recent transforms between different frames. It's used to track the transformation from the map frame to
     the base_link frame (the robot's position).
     Initializing tf_buffer_ with get_clock() allows it to sync with the ROS2 system clock for accurate timing of transformations.
     tf_buffer_ Stores and provides access to frame transformations.
    */
    tf2_ros::Buffer tf_buffer_;

    /* tf_listener_ Subscribes to the tf and tf_static topics, where transforms between frames are published, and updates tf_buffer_ with these transforms.
     tf_listener_ keeps tf_buffer_ up-to-date with the latest frame data from the ROS2 system.
     */
    tf2_ros::TransformListener tf_listener_;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigator>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
