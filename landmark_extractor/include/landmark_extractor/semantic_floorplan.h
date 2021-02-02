//
// Created by han on 21/1/21.
//

#ifndef SRC_SEMANTIC_FLOORPLAN_H
#define SRC_SEMANTIC_FLOORPLAN_H

#include <vector>
#include <cstdint>
#include <unordered_map>
#include <landmark_extractor/label_service.h>
#include <opencv2/core.hpp>

struct FloorPlanCoord {
    double x;
    double y;
    int label;
};

class SemanticFloorPlan {
public:
    bool hasLabel(uint8_t r, uint8_t g, uint8_t b);

    void insertCoord(FloorPlanCoord coord);

    bool populateFromImage(const std::string &filePath);    //image must be rgb, and only landmarks should be coloured

    std::vector<FloorPlanCoord> getCoords(uint8_t r, uint8_t g, uint8_t b);

private:
    FloorPlanCoord getCentre(int xStart, int yStart, cv::Mat &image, cv::Mat &visited);

    std::unordered_map<int, std::vector<FloorPlanCoord>> map_;
};

#endif //SRC_SEMANTIC_FLOORPLAN_H
