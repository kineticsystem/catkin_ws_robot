#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>

#include "pick_objects/OperationStatus.h"

static pick_objects::OperationStatus status;

// Create a message to display a marget at a given position.
visualization_msgs::Marker createAddMarkerMessage(double x, double y) {

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
}

// Create a message to delete all markers.
visualization_msgs::Marker createDeletionMarkersMessage() {

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.action = visualization_msgs::Marker::DELETEALL;

    return marker;
}

// Receive information about a item being loaded or deployed.
void processOperationStatus(const pick_objects::OperationStatusConstPtr &msg) {
    status = *msg;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "marker_node");
    ros::NodeHandle n;
    ros::Rate r(1);

    // Generate markers.
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    // Read goals.
    ros::Subscriber status_sub = n.subscribe<pick_objects::OperationStatus>("/operation_status", 10, processOperationStatus);

    while (ros::ok()) {
        ros::spinOnce();

        if (status.type == "deployed") {
            marker_pub.publish(createAddMarkerMessage(status.x, status.y));
            ROS_INFO("Package deployed at (x:%f,y:%f)", status.x, status.y);
            status.type = "";
        } else if (status.type == "loaded") {
            marker_pub.publish(createDeletionMarkersMessage());
            ROS_INFO("Package loaded at (x:%f,y:%f)", status.x, status.y);
            status.type = "";
        }

        r.sleep();
    }
}
