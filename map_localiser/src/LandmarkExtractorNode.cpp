//
// Created by han on 16/2/21.
//

#include <map_localiser/LandmarkExtractorNode.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "Landmark Extractor Node");
    ros::NodeHandle nh;
    LandmarkExtractorNode node(nh);
    ros::spin();
    return 0;
}

LandmarkExtractorNode::LandmarkExtractorNode(ros::NodeHandle &nh) : nh_(nh), tf_listener_(tf_buffer_) {
    nh_.getParam("/extractor/camera_frame_id", camera_frame_id_);
    nh_.getParam("/extractor/octomap_topic", octomap_topic_);
    nh_.getParam("/extractor/octomap_frame_id", octomap_frame_id_);
    nh_.getParam("/extractor/landmarks_topic", landmark_topic_);
    nh_.getParam("/extractor/buffer_size", buffer_size_);
    nh_.getParam("/extractor/search_radius", search_radius_);
    nh_.getParam("/extractor/strategy", strategy_);

    //basic checks
    if (buffer_size_ < 1 || search_radius_ < 1)
        ROS_FATAL("buffer_size or search_radius specified less than 1");

    octomap_sub_ = new message_filters::Subscriber<octomap_msgs::Octomap>(nh_, octomap_topic_, 10);
    tf_filter_ = new tf2_ros::MessageFilter<octomap_msgs::Octomap>(*octomap_sub_, tf_buffer_, octomap_frame_id_, 10, 0);
    landmark_pub_ = nh_.advertise<map_localiser::ExtractorLandmarks>(landmark_topic_, 10);
    extractor_ = new LandmarkExtractor(buffer_size_, search_radius_, strategy_);
    tf_filter_->registerCallback([this](auto &&PH1) { extractionCallback(std::forward<decltype(PH1)>(PH1)); });
}

void LandmarkExtractorNode::extractionCallback(octomap_msgs::OctomapConstPtr msg) {
    auto tf = tf_buffer_.lookupTransform(octomap_frame_id_, camera_frame_id_, msg.get()->header.stamp);
    auto l = extractor_->extract(msg, tf);
    //ROS_INFO("found %zu landmarks", l.landmarks.size());
    l.header.frame_id = octomap_frame_id_;
    l.header.stamp = ros::Time::now();
    landmark_pub_.publish(l);
}

LandmarkExtractorNode::~LandmarkExtractorNode() {
    delete octomap_sub_;
    delete tf_filter_;
    delete extractor_;
}


