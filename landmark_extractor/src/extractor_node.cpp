#include <landmark_extractor/extractor_node.h>
#include <landmark_extractor/ExtractorLandmarks.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "landmark_extractor_node");
    ros::NodeHandle nh;
    ExtractorNode extractorNode(nh);
    ros::spin();
}

ExtractorNode::ExtractorNode(ros::NodeHandle &nh) : nh_(nh), tf_listener_(tf_buffer_) {
    ROS_INFO("Initialising extractor node");

    nh_.getParam("/extractor/camera_frame_id", camera_frame_id_);
    nh_.getParam("/extractor/octomap_topic", octomap_topic_);
    nh_.getParam("/extractor/octomap_frame_id", octomap_frame_id_);
    nh_.getParam("/extractor/search_size", search_size_);
    nh_.getParam("/extractor/search_radius", search_radius_);
    nh_.getParam("/extractor/strategy", strategy_);
    nh_.getParam("/extractor/landmark_topic", landmark_topic_);

    octomap_sub_ = new message_filters::Subscriber<octomap_msgs::Octomap>(nh_, octomap_topic_, 10);
    tf_filter_ = new tf2_ros::MessageFilter<octomap_msgs::Octomap>(*octomap_sub_, tf_buffer_, octomap_frame_id_,
                                                                   10, 0);
    landmark_pub_ = nh_.advertise<landmark_extractor::ExtractorLandmarks>(landmark_topic_, 10);

    extractor_ = new Extractor(search_size_);
    tf_filter_->registerCallback(boost::bind(&ExtractorNode::msgCallback, this, _1));

    ROS_INFO("Extractor node ready");
}

void ExtractorNode::msgCallback(octomap_msgs::Octomap::ConstPtr msg) {
    geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(octomap_frame_id_, camera_frame_id_,
                                                                    msg.get()->header.stamp); //get latest time stamp
    extractor_->extract(search_radius_, strategy_, msg, tf);
    Landmark *landmarks = extractor_->getLandmarks();
    int found = extractor_->getFound();

    landmark_extractor::ExtractorLandmarks landmarksMsg;
    while (found) {
        landmark_extractor::ExtractorLandmark l = landmarks[--found].toMsg();
        landmarksMsg.landmarks.push_back(l);
    }
    landmarksMsg.header.stamp = ros::Time::now();
    landmarksMsg.header.frame_id = octomap_frame_id_;

    landmark_pub_.publish(landmarksMsg);
}

ExtractorNode::~ExtractorNode() {
    delete tf_filter_;
    delete octomap_sub_;
    delete extractor_;
}