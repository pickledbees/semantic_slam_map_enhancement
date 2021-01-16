//
// Created by han on 3/1/21.
//

#ifndef SRC_SEMANTICOCCUPANCYGRIDGENERATOR_H
#define SRC_SEMANTICOCCUPANCYGRIDGENERATOR_H

#include <octomap_msgs/Octomap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_listener.h>
#include <octomap_msgs/Octomap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap/ColorOcTree.h>

class SemanticOccupancyGridGenerator {
private:
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf_listener;
public:
    void insertSemantics(nav_msgs::OccupancyGridConstPtr grid, octomap_msgs::OctomapConstPtr octomap);
};


#endif //SRC_SEMANTICOCCUPANCYGRIDGENERATOR_H
