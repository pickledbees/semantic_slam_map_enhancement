//
// Created by han on 16/2/21.
//

#ifndef SRC_LANDMARKEXTRACTORNODE_H
#define SRC_LANDMARKEXTRACTORNODE_H

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <map_localiser/LandmarkExtractor.h>

class LandmarkExtractorNode {
public:
    explicit LandmarkExtractorNode(ros::NodeHandle &nh);

    void extractionCallback(octomap_msgs::OctomapConstPtr msg);

    void reset();

    ~LandmarkExtractorNode();

private:
    std::string camera_frame_id_;
    std::string octomap_topic_;
    std::string octomap_frame_id_;
    std::string landmark_topic_;

    int buffer_size_;
    double search_radius_;
    int strategy_;

    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::MessageFilter<octomap_msgs::Octomap> *tf_filter_;
    message_filters::Subscriber<octomap_msgs::Octomap> *octomap_sub_;
    ros::Publisher landmark_pub_;

    LandmarkExtractor *extractor_;
};

#endif //SRC_LANDMARKEXTRACTORNODE_H
