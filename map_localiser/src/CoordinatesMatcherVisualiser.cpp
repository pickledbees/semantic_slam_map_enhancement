//
// Created by han on 25/2/21.
//

#include <ros/ros.h>
#include <map_localiser/LabelService.h>
#include <map_localiser/MatchResult.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stack>
#include <cstdint>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class Visualiser {
    struct Pixel {
        int x, y;
        uint8_t r, g, b;
    };
public:
    explicit Visualiser(ros::NodeHandle &nh) : nh_(nh) {
        loadImage();

        nh.getParam("/matcher/visualiser/image_topic", image_topic_);
        nh.getParam("/matcher/match_result_topic", match_result_topic_);
        image_transport::ImageTransport it(nh_);
        pub_ = it.advertise(image_topic_, 1);
        sub_ = nh.subscribe(match_result_topic_, 1, &Visualiser::visualise, this);
    }

    void loadImage() {
        nh_.getParam("/matcher/floorplan_file_path", file_path_);

        ROS_INFO("reading image file");

        image_ = cv::imread(file_path_, cv::IMREAD_COLOR);

        if (image_.empty()) {
            ROS_WARN("could not load image");
            return;
        }

        if (image_.channels() != 3) {
            ROS_WARN("image not in RGB");
            return;
        }

        ROS_INFO("loaded image into visualiser");
    }

    void visualise(map_localiser::MatchResult result) {
        if (image_.empty()) {
            ROS_WARN("results received but no image to load");
            return;
        }

        if (result.chains.empty()) return;

        std::stack<Pixel> altered;
        auto coords = result.chains.back().elements;
        for (const auto &coord : coords) {
            auto &pixel = image_.at<cv::Vec3b>(cv::Point(coord.x, coord.y));

            //cache
            Pixel alteredPixel{};
            alteredPixel.x = coord.x;
            alteredPixel.y = coord.y;
            alteredPixel.x = coord.x;
            alteredPixel.y = coord.y;
            alteredPixel.r = pixel[2];
            alteredPixel.g = pixel[1];
            alteredPixel.b = pixel[0];
            altered.push(alteredPixel);

            //mark
            pixel[2] = 0;
            pixel[1] = 255;
            pixel[0] = 255;
        }

        //publish
        cv_bridge::CvImagePtr ptr(new cv_bridge::CvImage);
        ptr->encoding = "bgr8";
        ptr->header.stamp = ros::Time::now();
        ptr->header.frame_id = "/match_results";
        ptr->image = image_;
        pub_.publish(ptr->toImageMsg());

        //restore
        while (!altered.empty()) {
            auto alteredPixel = altered.top();
            altered.pop();

            auto &pixel = image_.at<cv::Vec3b>(cv::Point(alteredPixel.x, alteredPixel.y));
            pixel[0] = alteredPixel.b;
            pixel[1] = alteredPixel.g;
            pixel[2] = alteredPixel.r;
        }
    }

private:
    std::string image_topic_;
    std::string match_result_topic_;
    std::string file_path_;
    cv::Mat image_;

    ros::NodeHandle nh_;
    image_transport::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "matcher_visualiser");
    ros::NodeHandle nh;
    Visualiser v(nh);
    ros::spin();
    return 0;
}