#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pick_objects/NavigationTarget.h"

static nav_msgs::Odometry odom;
static pick_objects::NavigationTarget target;

geometry_msgs::Pose pose(double x, double y, double yaw) {

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    geometry_msgs::Quaternion orientation = tf2::toMsg(quaternion);

    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation = orientation;
    return pose;
}

// Calculate the distance between two points.
double distance(double x0, double y0, double x1, double y1) {
    return std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
}

void addMarker(double x, double y, const ros::Publisher &marker_pub) {

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

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);
}

void deleteMarker(const ros::Publisher &marker_pub) {

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.action = visualization_msgs::Marker::DELETEALL;

    marker_pub.publish(marker);
}

void processOdometry(const nav_msgs::Odometry::ConstPtr& msg) {
    odom = *msg;
}

void processNavigationTarget(const pick_objects::NavigationTargetConstPtr &msg) {
    target = *msg;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "marker_node");
    ros::NodeHandle n;
    ros::Rate r(1);

    // Generate markers.
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    // Read goals.
    ros::Subscriber target_sub = n.subscribe<pick_objects::NavigationTarget>("/navigation_targets", 10, processNavigationTarget);

    // Read robot position.
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, processOdometry);

    while (ros::ok()) {
        ros::spinOnce();

        double d = distance(odom.pose.pose.position.x, odom.pose.pose.position.y, target.x, target.y);

        if (target.type == "pick-up" && d > 20) {
            addMarker(target.x, target.y, marker_pub);
            target.type = "picking_up";
        }

        if (target.type == "picking_up" && (d < 20)) {
            deleteMarker(marker_pub);
            target.type = "";
        }

        r.sleep();
    }
}
