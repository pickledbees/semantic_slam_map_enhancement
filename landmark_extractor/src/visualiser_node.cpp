#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <landmark_extractor/ExtractorLandmarks.h>
#include <landmark_extractor/extractor.h>
#include <unordered_map>

class Visualiser {
public:
    explicit Visualiser(ros::NodeHandle &nh) : nh_(nh) {
        nh_.getParam("/extractor/landmark_topic", landmark_topic_);
        nh_.getParam("/extractor/visualiser_point_scale", point_scale_);
        nh_.getParam("/extractor/visualiser_text_scale", text_scale_);
        nh_.getParam("/extractor/visualiser_namespace", namespace_);
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
        sub_ = nh_.subscribe(landmark_topic_, 10, &Visualiser::visualise, this);

        insertLabel(0, 0, 0, "background");
        insertLabel(0, 0, 64, "nightstand");
        insertLabel(0, 0, 128, "bed");
        insertLabel(0, 0, 192, "bathtub");
        insertLabel(0, 64, 0, "curtain");
        insertLabel(0, 64, 128, "floormat");
        insertLabel(0, 128, 0, "floor");
        insertLabel(0, 128, 64, "sink");
        insertLabel(0, 128, 128, "sofa");
        insertLabel(0, 192, 0, "pillow");
        insertLabel(0, 192, 128, "ceiling");
        insertLabel(64, 0, 0, "door");
        insertLabel(64, 0, 128, "counter");
        insertLabel(64, 64, 0, "fridge");
        insertLabel(64, 64, 128, "shower_curtain");
        insertLabel(64, 128, 0, "bookshelf");
        insertLabel(64, 128, 128, "desk");
        insertLabel(64, 192, 0, "paper");
        insertLabel(64, 192, 128, "whiteboard");
        insertLabel(128, 0, 0, "wall");
        insertLabel(128, 0, 64, "toilet");
        insertLabel(128, 0, 128, "chair");
        insertLabel(128, 0, 192, "bag");
        insertLabel(128, 64, 0, "dresser");
        insertLabel(128, 64, 128, "clothes");
        insertLabel(128, 128, 0, "cabinet");
        insertLabel(128, 128, 64, "lamp");
        insertLabel(128, 128, 128, "table");
        insertLabel(128, 192, 0, "mirror");
        insertLabel(128, 192, 128, "books");
        insertLabel(192, 0, 0, "window");
        insertLabel(192, 0, 128, "blinds");
        insertLabel(192, 64, 0, "tv");
        insertLabel(192, 64, 128, "box");
        insertLabel(192, 128, 0, "picture");
        insertLabel(192, 128, 128, "shelves");
        insertLabel(192, 192, 0, "towel");
        insertLabel(192, 192, 128, "person");
    }

    void visualise(landmark_extractor::ExtractorLandmarks landmarksMsg) {

        int id = 0;
        ros::Time time = ros::Time::now();

        //initialise markers
        visualization_msgs::MarkerArray markers;

        visualization_msgs::Marker deleter;
        deleter.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(deleter);

        visualization_msgs::Marker points;
        points.type = visualization_msgs::Marker::POINTS;
        points.header.frame_id = landmarksMsg.header.frame_id;
        points.ns = namespace_;
        points.id = id++;
        points.header.stamp = time;
        points.scale.x = point_scale_;
        points.scale.y = point_scale_;
        points.color.r = 1;
        points.color.a = 1;

        for (auto &landmark : landmarksMsg.landmarks) {
            visualization_msgs::Marker text;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.header.frame_id = landmarksMsg.header.frame_id;
            text.ns = namespace_;
            text.id = id++;
            text.header.stamp = time;
            text.scale.z = text_scale_;
            text.color.r = 1;
            text.color.g = 1;
            text.color.b = 1;
            text.color.a = 1;
            text.text = getLabel(landmark.r, landmark.g, landmark.b);

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
    void insertLabel(uint8_t r, uint8_t g, uint8_t b, std::string label) {
        int key = Extractor::keyFromRGB(r, g, b);
        label_map_.insert({key, label});
    }

    std::string getLabel(uint8_t r, uint8_t g, uint8_t b) {
        int key = Extractor::keyFromRGB(r, g, b);
        auto pair = label_map_.find(key);
        return pair != label_map_.end() ? pair->second : "unknown";
    }

    std::string landmark_topic_;

    float point_scale_;
    float text_scale_;
    std::string namespace_;

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    std::unordered_map<int, std::string> label_map_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "landmark_extractor_visualiser");
    ros::NodeHandle nh;

    Visualiser v(nh);

    ros::spin();
}