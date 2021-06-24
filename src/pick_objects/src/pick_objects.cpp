#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

#include "pick_objects/NavigationCommand.h"
#include "pick_objects/OperationStatus.h"

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static pick_objects::NavigationCommand cmd;

void processNavigationCommand(const pick_objects::NavigationCommandConstPtr &msg) {
    cmd = *msg;
}

// Create a message about an item loaded or deployed.
pick_objects::OperationStatus createStatus(std::string type, double x, double y) {
    pick_objects::OperationStatus status;
    status.type = type;
    status.x = x;
    status.y = y;
    return status;
}

// Convert a simple navigation command into a robot navigation command with pose and orientation.
move_base_msgs::MoveBaseGoal from(const pick_objects::NavigationCommand &cmd) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = cmd.x;
    goal.target_pose.pose.position.y = cmd.y;
    goal.target_pose.pose.orientation.w = 1.0;
    return goal;
}

int main(int argc, char** argv) {

    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects_node");
    ros::NodeHandle n;
    ros::Rate r(1);

    // Read goals.
    ros::Subscriber target_sub = n.subscribe<pick_objects::NavigationCommand>("/navigation_command", 10, processNavigationCommand);

    // Notify status.
    ros::Publisher status_pub = n.advertise<pick_objects::OperationStatus>("/operation_status", 1);

    // Tell the action client that we want to spin a thread by default.
    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    while (ros::ok()) {
        ros::spinOnce();

        if (cmd.type == "pick-up") {
            cmd.type = "";

            status_pub.publish(createStatus("deployed", cmd.x, cmd.y));

            ROS_INFO("Go to pickup zone  (x:%f,y:%f)", cmd.x, cmd.y);
            ac.sendGoal(from(cmd));
            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                status_pub.publish(createStatus("loaded", cmd.x, cmd.y));
                ROS_INFO("The robot reached the pickup zone.");
            } else {
                ROS_INFO("The robot failed to reach the pickup zone.");
            }

            ros::Duration(5.0).sleep();

        } else if (cmd.type == "drop-off") {
            cmd.type = "";

            status_pub.publish(createStatus("loaded", cmd.x, cmd.y));

            ROS_INFO("Go to drop zone  (x:%f,y:%f)", cmd.x, cmd.y);
            ac.sendGoal(from(cmd));
            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                status_pub.publish(createStatus("deployed", cmd.x, cmd.y));
                ROS_INFO("The robot reached the drop off zone.");
            } else {
                ROS_INFO("The robot failed to reach the drop zone.");
            }

            ros::Duration(5.0).sleep();
        }

        r.sleep();
    }

    return 0;
}
