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
//#include "nav2_core/waypoint_task_executor.hpp"
#include <vector>
#include <memory>


using namespace std::chrono_literals;

class WaypointNavigator : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using WaypointFollowerGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;

    WaypointNavigator()
        : Node("waypoint_navigator"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_), server_timeout_(100) {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&WaypointNavigator::joystick_cb, this, std::placeholders::_1));

        // clients
        navigation_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        waypoint_follower_action_client_ = rclcpp_action::create_client<FollowWaypoints>(this,"follow_waypoints");

        

        RCLCPP_INFO(this->get_logger(), "WaypointNavigator node started.");
    }

private:
    void joystick_cb(const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto now = this->now();
        // Button 0 triggers the waypoint saving (button X)
        if (msg->buttons[0] == 1 && (now - last_button_0_press_) > debounce_duration_) {
            last_button_0_press_ = now;  // Update last press time
            geometry_msgs::msg::PoseStamped current_pose;
            if (getRobotPose(current_pose)) {
                waypoints_.push_back(current_pose);
                RCLCPP_INFO(this->get_logger(), "Waypoint recorded at position: [%.2f, %.2f]",
                            current_pose.pose.position.x, current_pose.pose.position.y);
            }
        }
        // Button 3 triggers the start of waypoint following (button Triangle)
        if (msg->buttons[3] == 1 && (now - last_button_3_press_) > debounce_duration_) {
            last_button_3_press_ = now;
            RCLCPP_INFO(this->get_logger(), "Starting waypoint navigation...");
            //followWaypoints();
            startWaypointFollowing(waypoints_);
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

#if 0
    // Updating followWaypoints() to send waypoints as goals to the action server and receive feedback about the robot's progress and final status
    void followWaypoints() {
        if (!navigation_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Navigation action server not available.");
            return;
        }

        for (const auto &waypoint : waypoints_) {
            RCLCPP_INFO(this->get_logger(), "Navigating to waypoint at (%.2f, %.2f)", 
                    waypoint.pose.position.x, waypoint.pose.position.y);

            //auto navigation_goal_ = NavigateToPose::Goal();
            //navigation_goal_.pose = waypoint;

            // configure callbacks for monitoring the progress of a goal sent to action server
            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            send_goal_options.feedback_callback =
                std::bind(&WaypointNavigator::feedback_cb, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback =
                std::bind(&WaypointNavigator::result_cb, this, std::placeholders::_1);

            // send_goal_options.goal_response_callback =
            // std::bind(&WaypointFollower::goalResponse_cb, this, std::placeholders::_1);
            

            auto future_goal_handle_ = navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
            //auto result_future = future_goal_handle_.get()->async_get_result();
            rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle_); // result_future?
        }

        RCLCPP_INFO(this->get_logger(), "Finished following all waypoints.");
    } // followWaypoints()
#endif

    void startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses){
        auto is_action_server_ready = waypoint_follower_action_client_->wait_for_action_server(std::chrono::seconds(5));
        if (!is_action_server_ready) {
            RCLCPP_ERROR(this->get_logger(), "follow_waypoints action server is not available."
                " Is the initial pose set?");
            return;
        }
        std::cout << "In the startWaypointFollowing function." << std::endl;
        // Send the goal poses
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

        //if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle, server_timeout_) != rclcpp::FutureReturnCode::SUCCESS) //why?
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
        //     return;
        // }

        // Get the goal handle and save so that we can check on completion in the timer callback
        waypoint_follower_goal_handle_ = future_goal_handle.get();
        if (!waypoint_follower_goal_handle_) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }

    } // startWaypointFollowing()

#if 0
    void feedback_cb( GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
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
#endif
    // // handle the server's response to the goal request
    // void goalResponse_cb(const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr & goal){
    //     if (!goal) {
    //         RCLCPP_ERROR(get_logger(), "navigate_to_pose action client failed to send goal to server.");
    //         current_goal_status_.status = ActionStatus::FAILED;
    //     }
    // }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_action_client_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoint_follower_action_client_;

    // waypoint_follower_goal_ = FollowWaypoints::Goal(); //590 why?
    // nav_through_poses_goal_ = NavigateThroughPoses::Goal(); //591 why?
    NavigateToPose::Goal navigation_goal_{};
    FollowWaypoints::Goal waypoint_follower_goal_{};

    // Goal-related state
    WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;


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

    // Timeout value when waiting for action servers to respnd
    std::chrono::milliseconds server_timeout_;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;

    // Debouncing
    rclcpp::Time last_button_0_press_ {this->now()};  // last press time button 0
    rclcpp::Time last_button_3_press_ {this->now()};  // last press time button 3
    rclcpp::Duration debounce_duration_ {std::chrono::milliseconds(500)};  // time tolerance (ms)


// protected:
//     rclcpp::executors::SingleThreadedExecutor executor_;


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
