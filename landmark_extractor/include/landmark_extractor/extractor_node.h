#ifndef SRC_EXTRACTOR_NODE_H
#define SRC_EXTRACTOR_NODE_H

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>

#include <landmark_extractor/extractor.h>

class ExtractorNode {
public:
    explicit ExtractorNode(ros::NodeHandle &nh);

    void msgCallback(octomap_msgs::Octomap::ConstPtr msg);

    ~ExtractorNode();

private:
    std::string camera_frame_id_;
    std::string octomap_topic_;
    std::string octomap_frame_id_;
    std::string landmark_topic_;

    int search_size_;
    float search_radius_;
    int strategy_;

    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::MessageFilter<octomap_msgs::Octomap> *tf_filter_;
    message_filters::Subscriber<octomap_msgs::Octomap> *octomap_sub_;
    ros::Publisher landmark_pub_;

    Extractor *extractor_;
};

#endif //SRC_EXTRACTOR_NODE_H
