//
// Created by han on 4/1/21.
//

#ifndef SRC_EXTRACTOR_H
#define SRC_EXTRACTOR_H

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <landmark_extractor/extractor_landmark.h>
#include <geometry_msgs/TransformStamped.h>
#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>

class Extractor {
public:
    enum Strategy {
        NEAREST,
        UNIQUE,
    };

    Extractor(int bufferSize);

    void
    extract(float radius, int strategy, octomap_msgs::Octomap::ConstPtr &msg,
            const geometry_msgs::TransformStamped &tf);

    Landmark *getLandmarks() const;

    int getFound() const;

    static int keyFromRGB(uint8_t r, uint8_t g, uint8_t b);

    ~Extractor();

private:
    void insertLandmark(uint8_t r, uint8_t g, uint8_t b);

    bool isLandmark(uint8_t r, uint8_t g, uint8_t b);

    void extractNearest(float radius, octomap_msgs::Octomap::ConstPtr &msg,
                        const geometry_msgs::TransformStamped &tf);

    void extractUnique(float radius, octomap_msgs::Octomap::ConstPtr &msg,
                       const geometry_msgs::TransformStamped &tf);

    Landmark *landmarks_buffer_;
    int found_;
    const int buffer_size_;

    std::priority_queue<Landmark, std::vector<Landmark>, std::less<>> pq_;      //sorted with top as the furthest distance (for easy sorting)
    std::unordered_map<int, Landmark> map_;

    std::unordered_set<int> landmark_colors_;
};

#endif //SRC_EXTRACTOR_H
