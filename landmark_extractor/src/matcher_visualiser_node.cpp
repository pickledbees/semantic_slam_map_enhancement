//
// Created by han on 8/2/21.
//

#include <ros/ros.h>
#include <landmark_extractor/label_service.h>
#include <landmark_extractor/MatchResultsMsg.h>
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
        std::string imageTopic;
        std::string matchResultsTopic;
        nh_.getParam("matcher/visualiser_image_topic", imageTopic);
        nh_.getParam("matcher/match_results_topic", matchResultsTopic);

        loadImage();

        image_transport::ImageTransport it(nh_);
        pub_ = it.advertise(imageTopic, 1);
        sub_ = nh_.subscribe(matchResultsTopic, 1, &Visualiser::visualise, this);   //get the latest message always
    }

    void loadImage() {
        nh_.getParam("matcher/floorplan_file_path", file_path_);
        ROS_INFO("loading image into matcher visualiser from '%s'...", file_path_.c_str());

        if (image_loaded_) {
            ROS_INFO("Image from '%s' already loaded", file_path_.c_str());
            return;
        }

        //read in image
        image_ = cv::imread(file_path_, cv::IMREAD_COLOR);

        if (image_.empty()) {
            ROS_WARN("could not load image");
            return;
        }

        if (image_.channels() != 3) {
            ROS_WARN("image not RGB");
            return;
        }

        image_loaded_ = true;
        ROS_INFO("image loaded into matcher visualiser");
    }

    void visualise(landmark_extractor::MatchResultsMsg results) {

        if (!image_loaded_) {
            ROS_WARN("results received but image not loaded to render markers");
            return;
        }

        if (results.chains.empty()) return;

        //draw detection on image
        std::stack<Pixel> altered;
        auto coords = results.chains.back().chain;
        for (const auto &coord : coords) {
            auto &pixel = image_.at<cv::Vec3b>(cv::Point(coord.x, coord.y));

            //cache pixel original value
            Pixel alteredPixel{};
            alteredPixel.x = coord.x;
            alteredPixel.y = coord.y;
            alteredPixel.r = pixel[2];
            alteredPixel.g = pixel[1];
            alteredPixel.b = pixel[0];
            altered.push(alteredPixel);

            //mark it red
            pixel[2] = 255;
            pixel[1] = 0;
            pixel[0] = 0;

        }

        //publish as image
        cv_bridge::CvImagePtr ptr(new cv_bridge::CvImage);
        ptr->encoding = "bgr8";
        ptr->header.stamp = ros::Time::now();
        ptr->header.frame_id = "/match_results";
        ptr->image = image_;
        pub_.publish(ptr->toImageMsg());

        //revert changes
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
    bool image_loaded_ = false;
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

