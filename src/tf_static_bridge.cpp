#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/qos.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <mutex>
#include <functional>


class StaticTransformsBridge {
public:
    StaticTransformsBridge(ros::NodeHandle ros1_node, rclcpp::Node::SharedPtr ros2_node) :
            ros1_node_(ros1_node), ros2_node_(ros2_node) {
        // ROS 1 -> ROS 2
        ros1_sub_ = ros1_node_.subscribe(
            "/tf_static", 100, &StaticTransformsBridge::ros1_to_ros2, this);
        ros2_pub_ = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(
            ros2_node_, "/tf_static", tf2_ros::StaticBroadcasterQoS());

        // ROS 2 -> ROS 1
        ros2_sub_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
            ros2_node_, "/tf_static", tf2_ros::StaticListenerQoS(),
            std::bind(&StaticTransformsBridge::ros2_to_ros1, this, std::placeholders::_1, std::placeholders::_2));
        ros1_pub_ = ros1_node_.advertise<tf2_msgs::TFMessage>(
            "/tf_static", 100, true);
    }
    ~StaticTransformsBridge() = default;

    void spin() {
        // ROS 1 asynchronous spinner
        ros::AsyncSpinner async_spinner(1);
        async_spinner.start();

        // ROS 2 spinning loop
        rclcpp::executors::SingleThreadedExecutor executor;
        while (ros1_node_.ok() && rclcpp::ok()) {
            executor.spin_node_once(ros2_node_, std::chrono::milliseconds(1000));
        }
    }

    void ros1_to_ros2(const ros::MessageEvent<tf2_msgs::TFMessage>& ros1_message_event) {
        const auto& connection_header = ros1_message_event.getConnectionHeaderPtr();
        if (!connection_header) {
            ROS_WARN("Dropping ROS 1 static transforms without connection header.");
            return;
        }

        auto caller_node_name_it = connection_header->find("callerid");
        if (caller_node_name_it != connection_header->end()) {
            // do not republish messages from this node
            if (caller_node_name_it->second == ros::this_node::getName()) {
                return;
            }
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const auto& ros1_new_static_transforms = ros1_message_event.getConstMessage();
        add_static_transforms(*ros1_new_static_transforms);
        ros2_pub_->publish(ros2_static_transforms_);
    }

    void ros2_to_ros1(tf2_msgs::msg::TFMessage::SharedPtr ros2_new_static_transforms,
            const rclcpp::MessageInfo& ros2_message_info) {
        bool result = false;
        auto ret = rmw_compare_gids_equal(
            &ros2_message_info.get_rmw_message_info().publisher_gid,
            &ros2_pub_->get_gid(),
            &result);
        if (ret == RMW_RET_OK) {
            if (result) {
                // do not republish messages from this node
                return;
            }
        } else {
            RCLCPP_WARN(ros2_node_->get_logger(),
                "Dropping ROS 2 static transforms since cloud not compare publishers gids.");
        }

        std::lock_guard<std::mutex> lock(mutex_);
        add_static_transforms(*ros2_new_static_transforms);
        ros1_pub_.publish(ros1_static_transforms_);
    }

private:
    void add_static_transforms(const tf2_msgs::TFMessage& ros1_new_static_transforms);
    void add_static_transforms(const tf2_msgs::msg::TFMessage& ros2_new_static_transforms);

    static geometry_msgs::msg::TransformStamped convert(const geometry_msgs::TransformStamped& ros1_tf);
    static geometry_msgs::TransformStamped convert(const geometry_msgs::msg::TransformStamped& ros2_tf);

private:
    ros::NodeHandle ros1_node_;
    rclcpp::Node::SharedPtr ros2_node_;

    // ROS 1 -> ROS 2
    ros::Subscriber ros1_sub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr ros2_pub_;

    // ROS 2 -> ROS 1
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr ros2_sub_;
    ros::Publisher ros1_pub_;

    std::mutex mutex_;
    tf2_msgs::TFMessage ros1_static_transforms_;
    tf2_msgs::msg::TFMessage ros2_static_transforms_;
};


void StaticTransformsBridge::add_static_transforms(const tf2_msgs::TFMessage& ros1_new_static_transforms) {
    for (const auto& ros1_new_tf : ros1_new_static_transforms.transforms) {
        bool found = false;
        for (auto& ros2_tf : ros2_static_transforms_.transforms) {
            if (ros1_new_tf.header.frame_id == ros2_tf.header.frame_id &&
                    ros1_new_tf.child_frame_id == ros2_tf.child_frame_id) {
                ros2_tf = convert(ros1_new_tf);
                found = true;
                break;
            }
        }
        if (!found) {
            ros2_static_transforms_.transforms.push_back(convert(ros1_new_tf));
        }
    }
}


void StaticTransformsBridge::add_static_transforms(const tf2_msgs::msg::TFMessage& ros2_new_static_transforms) {
    for (const auto& ros2_new_tf : ros2_new_static_transforms.transforms) {
        bool found = false;
        for (auto& ros1_tf : ros1_static_transforms_.transforms) {
            if (ros2_new_tf.header.frame_id == ros1_tf.header.frame_id &&
                    ros2_new_tf.child_frame_id == ros1_tf.child_frame_id) {
                ros1_tf = convert(ros2_new_tf);
                found = true;
                break;
            }
        }
        if (!found) {
            ros1_static_transforms_.transforms.push_back(convert(ros2_new_tf));
        }
    }
}


geometry_msgs::msg::TransformStamped StaticTransformsBridge::convert(const geometry_msgs::TransformStamped& ros1_tf) {
    geometry_msgs::msg::TransformStamped ros2_tf;
    ros2_tf.header.stamp.sec = ros1_tf.header.stamp.sec;
    ros2_tf.header.stamp.nanosec = ros1_tf.header.stamp.nsec;
    ros2_tf.header.frame_id = ros1_tf.header.frame_id;
    ros2_tf.child_frame_id = ros1_tf.child_frame_id;
    ros2_tf.transform.translation.x = ros1_tf.transform.translation.x;
    ros2_tf.transform.translation.y = ros1_tf.transform.translation.y;
    ros2_tf.transform.translation.z = ros1_tf.transform.translation.z;
    ros2_tf.transform.rotation.x = ros1_tf.transform.rotation.x;
    ros2_tf.transform.rotation.y = ros1_tf.transform.rotation.y;
    ros2_tf.transform.rotation.z = ros1_tf.transform.rotation.z;
    ros2_tf.transform.rotation.w = ros1_tf.transform.rotation.w;
    return ros2_tf;
}


geometry_msgs::TransformStamped StaticTransformsBridge::convert(const geometry_msgs::msg::TransformStamped& ros2_tf) {
    geometry_msgs::TransformStamped ros1_tf;
    ros1_tf.header.stamp.sec = ros2_tf.header.stamp.sec;
    ros1_tf.header.stamp.nsec = ros2_tf.header.stamp.nanosec;
    ros1_tf.header.frame_id = ros2_tf.header.frame_id;
    ros1_tf.child_frame_id = ros2_tf.child_frame_id;
    ros1_tf.transform.translation.x = ros2_tf.transform.translation.x;
    ros1_tf.transform.translation.y = ros2_tf.transform.translation.y;
    ros1_tf.transform.translation.z = ros2_tf.transform.translation.z;
    ros1_tf.transform.rotation.x = ros2_tf.transform.rotation.x;
    ros1_tf.transform.rotation.y = ros2_tf.transform.rotation.y;
    ros1_tf.transform.rotation.z = ros2_tf.transform.rotation.z;
    ros1_tf.transform.rotation.w = ros2_tf.transform.rotation.w;
    return ros1_tf;
}


int main(int argc, char * argv[]) {
    // ROS 1 node
    ros::init(argc, argv, "ros_tf_static_bridge");
    ros::NodeHandle ros1_node;

    // ROS 2 node
    rclcpp::init(argc, argv);
    auto ros2_node = rclcpp::Node::make_shared("ros_tf_static_bridge");

    StaticTransformsBridge static_transforms_bridge(ros1_node, ros2_node);
    std::cout << "Spinning...\n";
    static_transforms_bridge.spin();
}