#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>

#define OCTOMAP_TOPIC "octomap_full"
#define OCCUPANCY_GRID_TOPIC "/projected_map"
#define OCCUPANCY_GRID_FRAME_ID "map"
#define TIME_DIFF 10

void insertSemantics(octomap_msgs::Octomap::ConstPtr octomap, nav_msgs::OccupancyGrid::ConstPtr grid) {
    ROS_INFO("%s, %s", octomap->header.frame_id.c_str(), grid->header.frame_id.c_str());

    //octomap is in /world, grid is in map
    //get world to map static transform
    //iterate over grid and find corresponding semantic in octomap
}

int main(int argc, char **argv) {
    //subscribe to projected maps topic
    //subscribe to octree topic
    //use message filter to sync the messages

    ros::init(argc, argv, "projector");
    ros::NodeHandle nh;

    typedef octomap_msgs::Octomap OctomapMsg;
    typedef nav_msgs::OccupancyGrid OccGridMsg;

    message_filters::Subscriber<OctomapMsg> octomap_sub(nh, OCTOMAP_TOPIC, 5);
    message_filters::Subscriber<OccGridMsg> grid_sub(nh, OCCUPANCY_GRID_TOPIC, 5);

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf_listener(buffer);
    tf2_ros::MessageFilter<OccGridMsg> grid_filter(buffer, OCCUPANCY_GRID_FRAME_ID, 10, nh);

    typedef message_filters::sync_policies::ApproximateTime<OctomapMsg, OccGridMsg> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(TIME_DIFF), octomap_sub, grid_filter);

    sync.registerCallback(boost::bind(&insertSemantics, _1, _2));
    ros::spin();
}

#undef OCTOMAP_TOPIC
#undef OCCUPANCY_GRID_TOPIC
#undef TIME_DIFF