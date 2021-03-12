//
// Created by han on 18/2/21.
//

#ifndef SRC_COORDINATESMATCHERNODE_H
#define SRC_COORDINATESMATCHERNODE_H

#include <ros/ros.h>
#include <map_localiser/ExtractorLandmarks.h>
#include <map_localiser/Floorplan.h>
#include <map_localiser/CoordinatesMatcher.h>

class CoordinatesMatcherNode {
public:
    explicit CoordinatesMatcherNode(const ros::NodeHandle &nh);

    void matchCallback(map_localiser::ExtractorLandmarks landmarks);

    ~CoordinatesMatcherNode();

private:
    std::string landmarks_topic_;
    std::string match_result_topic_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    Floorplan fp_;
    CoordinatesMatcher *matcher_;
};


#endif //SRC_COORDINATESMATCHERNODE_H
