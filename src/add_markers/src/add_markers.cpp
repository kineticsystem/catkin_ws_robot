#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static nav_msgs::Odometry odom;

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

visualization_msgs::Marker generateMarker(double x, double y) {

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
}

void processOdometry(const nav_msgs::Odometry::ConstPtr& msg) {
    odom = *msg;
}

int main( int argc, char** argv )
{
    std::pair<double, double> dropZone{-1.325962, 1.325962};
    std::vector<std::pair<double, double>> pickupZones{
        {1.325962, 1.325962},
        {1.325962, -1.325962},
        {-1.0, -1.0}
    };

    ros::init(argc, argv, "marker_node");
    ros::NodeHandle n;
    ros::Rate r(1);

    // Generate markers/goals.
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Read robot position.
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, processOdometry);

    while (ros::ok()) {

        visualization_msgs::Marker marker = generateMarker(0.0, 0.0);
   
        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);
 
        r.sleep();
    }
}
