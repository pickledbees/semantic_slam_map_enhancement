//
// Created by han on 16/2/21.
//

#ifndef SRC_LANDMARKEXTRACTOR_H
#define SRC_LANDMARKEXTRACTOR_H

#include <ros/ros.h>
#include <map_localiser/ExtractorLandmarks.h>
#include <geometry_msgs/TransformStamped.h>
#include <octomap_msgs/Octomap.h>

class LandmarkExtractor {
public:
    LandmarkExtractor();

    LandmarkExtractor(size_t bufferSize, double searchRadius, int strategy);

    map_localiser::ExtractorLandmarks
    extract(octomap_msgs::OctomapConstPtr &msg, const geometry_msgs::TransformStamped &tf);

private:
    map_localiser::ExtractorLandmarks
    extractNearest(octomap_msgs::OctomapConstPtr &msg, const geometry_msgs::TransformStamped &tf
    );

    map_localiser::ExtractorLandmarks
    extractUnique(octomap_msgs::OctomapConstPtr &msg, const geometry_msgs::TransformStamped &tf
    );

    size_t buffer_size_ = 10;
    double search_radius_ = 10000;
    int strategy_ = 1;
};


#endif //SRC_LANDMARKEXTRACTOR_H
