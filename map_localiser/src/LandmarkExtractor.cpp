//
// Created by han on 16/2/21.
//

#include <map_localiser/LandmarkExtractor.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <map_localiser/LabelService.h>
#include <map_localiser/ExtractorLandmarkWrapper.h>
#include <queue>

double inline distance(double x1, double y1, double z1, double x2, double y2, double z2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

LandmarkExtractor::LandmarkExtractor(size_t bufferSize, double searchRadius, int strategy) : buffer_size_(bufferSize),
                                                                                             search_radius_(
                                                                                                     searchRadius),
                                                                                             strategy_(strategy) {}

map_localiser::ExtractorLandmarks
LandmarkExtractor::extract(octomap_msgs::OctomapConstPtr &msg, const geometry_msgs::TransformStamped &tf) {
    map_localiser::ExtractorLandmarks l;
    switch (strategy_) {
        case 0:
            l = extractNearest(msg, tf);
        case 1:
        default:
            l = extractUnique(msg, tf);
    }
    return l;
}

map_localiser::ExtractorLandmarks
LandmarkExtractor::extractNearest(octomap_msgs::OctomapConstPtr &msg, const geometry_msgs::TransformStamped &tf) {
    auto *octree = dynamic_cast<octomap::ColorOcTree *>(octomap_msgs::msgToMap(*msg));

    float cx = tf.transform.translation.x;
    float cy = tf.transform.translation.y;
    float cz = tf.transform.translation.z;

    std::priority_queue<ExtractorLandmarkWrapper, std::vector<ExtractorLandmarkWrapper>, std::less<>> q;

    for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; it++) {
        auto color = it->getColor();
        int hash = LabelService::RGBToHash(color.r, color.g, color.b);
        if (!LabelService::isLandmark(hash)) continue;
        double d = distance(it.getX(), it.getY(), it.getZ(), cx, cy, cz);
        if (d > search_radius_) continue;

        q.push(ExtractorLandmarkWrapper(it.getX(), it.getY(), it.getZ(), hash, d));
        if (q.size() > buffer_size_) q.pop();
    }

    map_localiser::ExtractorLandmarks l;
    while (!q.empty()) {
        l.landmarks.push_back(q.top().getLandmark());
        q.pop();
    }

    delete octree;

    return l;
}

map_localiser::ExtractorLandmarks
LandmarkExtractor::extractUnique(octomap_msgs::OctomapConstPtr &msg, const geometry_msgs::TransformStamped &tf) {
    auto *octree = dynamic_cast<octomap::ColorOcTree *>(octomap_msgs::msgToMap(*msg));

    float cx = tf.transform.translation.x;
    float cy = tf.transform.translation.y;
    float cz = tf.transform.translation.z;

    std::unordered_map<int, ExtractorLandmarkWrapper> map;
    std::priority_queue<ExtractorLandmarkWrapper, std::vector<ExtractorLandmarkWrapper>, std::less<>> q;

    for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; it++) {
        auto color = it->getColor();
        int hash = LabelService::RGBToHash(color.r, color.g, color.b);
        if (!LabelService::isLandmark(hash)) continue;
        double d = distance(it.getX(), it.getY(), it.getZ(), cx, cy, cz);
        if (d > search_radius_) continue;

        auto pair = map.find(hash);
        if (pair == map.end() || (pair->second.getDepth() >= it.getDepth() && pair->second.getDistance() > d))
            map.insert({hash, ExtractorLandmarkWrapper(it.getX(), it.getY(), it.getZ(), hash, d, it.getDepth())});
    }

    for (const auto &pair : map) q.push(pair.second);
    while (q.size() > buffer_size_) q.pop();

    map_localiser::ExtractorLandmarks l;
    while (!q.empty()) {
        l.landmarks.push_back(q.top().getLandmark());
        q.pop();
    }

    delete octree;

    return l;
}
