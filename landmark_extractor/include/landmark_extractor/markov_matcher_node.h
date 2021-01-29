//
// Created by han on 21/1/21.
//

#ifndef SRC_MARKOV_MATCHER_NODE_H
#define SRC_MARKOV_MATCHER_NODE_H

#include <ros/ros.h>
#include <landmark_extractor/ExtractorLandmarks.h>
#include <landmark_extractor/markov_matcher.h>

class MatcherNode {
public:
    explicit MatcherNode(const ros::NodeHandle &nh);

    void messageCallback(landmark_extractor::ExtractorLandmarks landmarks);

    ~MatcherNode();

private:
    std::string landmarks_topic_;
    std::string floorplan_file_path_;
    std::string match_results_topic_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    SemanticFloorPlan fp_;
    MarkovMatcher *matcher_;
};

#endif //SRC_MARKOV_MATCHER_NODE_H
