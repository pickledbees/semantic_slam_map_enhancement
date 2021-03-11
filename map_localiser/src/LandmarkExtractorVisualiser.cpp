//
// Created by han on 25/2/21.
//

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <map_localiser/ExtractorLandmarks.h>
#include <map_localiser/LabelService.h>

class Visualiser {
public:
    explicit Visualiser(const ros::NodeHandle &nh) : nh_(nh) {
        nh_.getParam("/extractor/landmarks_topic", landmark_topic_);
        nh_.getParam("/extractor/visualiser/point_scale", point_scale_);
        nh_.getParam("/extractor/visualiser/text_scale", text_scale_);
        nh_.getParam("/extractor/visualiser/namespace", marker_array_topic_);
        nh_.getParam("/extractor/visualiser/marker_array_topic", marker_array_topic_);

        pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_array_topic_, 10);
        sub_ = nh_.subscribe(landmark_topic_, 10, &Visualiser::visualise, this);
    }

    void visualise(map_localiser::ExtractorLandmarks landmarks) {
        int id = 0;
        ros::Time time = ros::Time::now();

        visualization_msgs::MarkerArray markers;

        visualization_msgs::Marker deleter;
        deleter.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(deleter);

        visualization_msgs::Marker points;
        points.type = visualization_msgs::Marker::POINTS;
        points.header.frame_id = landmarks.header.frame_id;
        points.ns = namespace_;
        points.id = id++;
        points.header.stamp = time;
        points.scale.x = point_scale_;
        points.scale.y = point_scale_;
        points.color.r = 1;
        points.color.a = 1;

        for (auto &landmark : landmarks.landmarks) {
            visualization_msgs::Marker text;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.header.frame_id = landmarks.header.frame_id;
            text.ns = namespace_;
            text.id = id++;
            text.header.stamp = time;
            text.scale.z = text_scale_;
            text.color.r = 1;
            text.color.g = 1;
            text.color.b = 1;
            text.color.a = 1;
            text.text = LabelService::hashToLabel(landmark.hash);

            geometry_msgs::Point p;
            p.x = landmark.x;
            p.y = landmark.y;
            p.z = landmark.z;
            points.points.push_back(p);

            text.pose.position.x = p.x;
            text.pose.position.y = p.y;
            text.pose.position.z = p.z;
            markers.markers.push_back(text);
        }

        markers.markers.push_back(points);
        pub_.publish(markers);
    }

private:
    std::string landmark_topic_, marker_array_topic_;
    double point_scale_{}, text_scale_{};
    std::string namespace_;

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "landmark_extractor_visualiser");
    ros::NodeHandle nh;
    Visualiser v(nh);
    ros::spin();
    return 0;
}