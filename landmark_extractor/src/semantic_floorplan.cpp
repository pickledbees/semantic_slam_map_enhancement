//
// Created by han on 21/1/21.
//

#include <landmark_extractor/semantic_floorplan.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

bool SemanticFloorPlan::hasLabel(int label) {
    return map_.find(label) != map_.end();
}

void SemanticFloorPlan::insertCoord(FloorPlanCoord &coord) {
    if (map_.find(coord.label) == map_.end()) map_.insert({coord.label, std::vector<FloorPlanCoord>()});
    map_[coord.label].push_back(coord);
}

bool SemanticFloorPlan::populateFromImage(const std::string &filePath) {
    cv::Mat image = cv::imread(filePath, cv::IMREAD_COLOR);
    if (image.empty()) return false;

    //only work with RGB images
    if (image.channels() != 3) return false;

    for (auto it = image.begin<cv::Vec3b>(), end = image.end<cv::Vec3b>(); it != end; ++it) {
        //perform rgb checks (create whitelist of RGB items);
        auto pixel = *it;
        //cv image is in BGR
        if (isLandmark(pixel[2], pixel[1], pixel[0])) {
            FloorPlanCoord coord{};
            coord.x = it.pos().x;
            coord.y = it.pos().y;
            coord.label = RGBToLabel(pixel[2], pixel[1], pixel[0]);
            insertCoord(coord);
        }
    }
    return true;
}

std::vector<FloorPlanCoord> SemanticFloorPlan::getCoords(int label) {
    return map_.find(label) == map_.end() ? std::vector<FloorPlanCoord>() : map_[label];
}

int SemanticFloorPlan::RGBToLabel(uint8_t r, uint8_t g, uint8_t b) {
    return r << 16 | g << 8 | b;
}

bool SemanticFloorPlan::isLandmark(uint8_t r, uint8_t g, uint8_t b) {
    return false;
}
