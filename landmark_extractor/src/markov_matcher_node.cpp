//
// Created by han on 21/1/21.
//

#include <landmark_extractor/markov_matcher_node.h>
#include <landmark_extractor/MatchResultsMsg.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "matcher_node");
    ros::NodeHandle nh;
    MatcherNode matcherNode(nh);
    ros::spin();
}

MatcherNode::MatcherNode(const ros::NodeHandle &nh) : nh_(nh) {
    double min_correlation_threshold;
    int max_buffer_size, top;
    nh_.getParam("matcher/top", top);
    nh_.getParam("matcher/min_correlation_threshold", min_correlation_threshold);
    nh_.getParam("matcher/max_buffer_size", max_buffer_size);
    nh_.getParam("matcher/floorplan_file_path", floorplan_file_path_);
    nh_.getParam("matcher/landmarks_topic", landmarks_topic_);
    nh_.getParam("matcher/match_results_topic", match_results_topic_);

    ROS_INFO("Loading floorplan image from: %s, this may take a while", floorplan_file_path_.c_str());
    if (!fp_.populateFromImage(floorplan_file_path_)) {
        ROS_WARN("Failed to load floorplan image from :%s, floorplan is uninitialised", floorplan_file_path_.c_str());
    } else {
        //produce summary of floorplan
        ROS_INFO("Floorplan loaded\n%s", fp_.getSummary().c_str());
    }

    matcher_ = new MarkovMatcher(top, min_correlation_threshold, max_buffer_size);

    sub_ = nh_.subscribe(landmarks_topic_, 10, &MatcherNode::messageCallback, this);
    pub_ = nh_.advertise<landmark_extractor::MatchResultsMsg>(match_results_topic_, 10);
}

void MatcherNode::messageCallback(landmark_extractor::ExtractorLandmarks landmarks) {
    MatchResults results = matcher_->match(landmarks, fp_);

    //TODO: remove the need for conversion / convert to constant pointer
    landmark_extractor::MatchResultsMsg msg;
    msg.header.stamp = ros::Time::now();
    for (const auto &chainPair : results.chains) {
        landmark_extractor::ChainCorrelationPairMsg chainPairMsg;
        chainPairMsg.correlation = chainPair.second;
        for (const auto &coord : chainPair.first) {
            landmark_extractor::FloorPlanCoordMsg coordMsg;
            coordMsg.x = coord.x;
            coordMsg.y = coord.y;
            coordMsg.hash = coord.hash;
            coordMsg.label = LabelService::getLabel(coord.hash);
            chainPairMsg.chain.push_back(coordMsg);
        }
        msg.chains.push_back(chainPairMsg);
    }

    msg.landmarks = results.landmarks;

    pub_.publish(msg);
}

MatcherNode::~MatcherNode() {
    delete matcher_;
}