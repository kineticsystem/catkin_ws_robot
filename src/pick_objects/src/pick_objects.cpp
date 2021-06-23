#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

#include "pick_objects/NavigationTarget.h"

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static pick_objects::NavigationTarget target;

void processNavigationTarget(const pick_objects::NavigationTargetConstPtr &msg) {
    target = *msg;
}

move_base_msgs::MoveBaseGoal from(const pick_objects::NavigationTarget &target) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = target.x;
    goal.target_pose.pose.position.y = target.y;
    goal.target_pose.pose.orientation.w = 1.0;
    return goal;
}

int main(int argc, char** argv) {

    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects_node");
    ros::NodeHandle n;
    ros::Rate r(1);

    // Read goals.
    ros::Subscriber marker_sub = n.subscribe<pick_objects::NavigationTarget>("/navigation_targets", 10, processNavigationTarget);

    // Tell the action client that we want to spin a thread by default.
    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    while (ros::ok()) {

        ros::spinOnce();

        if (target.type == "pick-up") {
            target.type = "";

            ROS_INFO("Go to pickup zone  (x:%f,y:%f)", target.x, target.y);
            ac.sendGoal(from(target));
            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("The robot reached the pickup zone.");
            } else {
                ROS_INFO("The robot failed to reach the pickup zone.");
            }

            ros::Duration(5.0).sleep();

        } else if (target.type == "drop-off") {
            target.type = "";

            ROS_INFO("Go to drop zone  (x:%f,y:%f)", target.x, target.y);
            ac.sendGoal(from(target));
            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("The robot reached the drop zone.");
            } else {
                ROS_INFO("The robot failed to reach the drop zone.");
            }

            ros::Duration(5.0).sleep();
        }
    }

    return 0;
}
