#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/qos.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <functional>


geometry_msgs::TransformStamped convert(const geometry_msgs::msg::TransformStamped& tf_ros2) {
    geometry_msgs::TransformStamped tf_ros1;
    tf_ros1.header.stamp.sec = tf_ros2.header.stamp.sec;
    tf_ros1.header.stamp.nsec = tf_ros2.header.stamp.nanosec;
    tf_ros1.header.frame_id = tf_ros2.header.frame_id;
    tf_ros1.child_frame_id = tf_ros2.child_frame_id;
    tf_ros1.transform.translation.x = tf_ros2.transform.translation.x;
    tf_ros1.transform.translation.y = tf_ros2.transform.translation.y;
    tf_ros1.transform.translation.z = tf_ros2.transform.translation.z;
    tf_ros1.transform.rotation.x = tf_ros2.transform.rotation.x;
    tf_ros1.transform.rotation.y = tf_ros2.transform.rotation.y;
    tf_ros1.transform.rotation.z = tf_ros2.transform.rotation.z;
    tf_ros1.transform.rotation.w = tf_ros2.transform.rotation.w;
    return tf_ros1;
}


void bridge_tf_static(tf2_msgs::msg::TFMessage::SharedPtr new_tfs, ros::Publisher& pub) {
    static tf2_msgs::TFMessage accum_tfs;
    for (const auto& new_tf : new_tfs->transforms) {
        bool found = false;
        for (auto& accum_tf : accum_tfs.transforms) {
            if (new_tf.header.frame_id == accum_tf.header.frame_id &&
                    new_tf.child_frame_id == accum_tf.child_frame_id) {
                accum_tf = convert(new_tf);
                found = true;
                break;
            }
        }
        if (!found) {
            accum_tfs.transforms.push_back(convert(new_tf));
        }
    }
    pub.publish(accum_tfs);
}


int main(int argc, char * argv[]) {
    // ROS 1 node
    ros::init(argc, argv, "ros_tf_static_bridge");
    ros::NodeHandle ros1_node;

    // ROS 2 node
    rclcpp::init(argc, argv);
    auto ros2_node = rclcpp::Node::make_shared("ros_tf_static_bridge");

    // ROS 1 publisher
    auto pub = ros1_node.advertise<tf2_msgs::TFMessage>("/tf_static", 100, true);

    // ROS 2 subscriber
    auto sub = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        ros2_node,
        "/tf_static",
        tf2_ros::StaticListenerQoS(),
        [&pub](tf2_msgs::msg::TFMessage::SharedPtr new_tfs) { bridge_tf_static(new_tfs, pub); });

    std::cout << "Spinning...\n";

    // ROS 1 asynchronous spinner
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    // ROS 2 spinning loop
    rclcpp::executors::SingleThreadedExecutor executor;
    while (ros1_node.ok() && rclcpp::ok()) {
        executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
    }
}