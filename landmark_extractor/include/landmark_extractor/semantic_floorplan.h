//
// Created by han on 21/1/21.
//

#ifndef SRC_SEMANTIC_FLOORPLAN_H
#define SRC_SEMANTIC_FLOORPLAN_H

#include <vector>
#include <cstdint>
#include <unordered_map>

struct FloorPlanCoord {
    double x;
    double y;
    int label;
};

class SemanticFloorPlan {
public:
    bool hasLabel(int label);

    void insertCoord(FloorPlanCoord &coord);

    bool populateFromImage(const std::string &filePath);

    std::vector<FloorPlanCoord> getCoords(int label);

    static int RGBToLabel(uint8_t r, uint8_t g, uint8_t b);
private:
    bool isLandmark(uint8_t r, uint8_t g, uint8_t b);

    std::unordered_map<int, std::vector<FloorPlanCoord>> map_;
};

#endif //SRC_SEMANTIC_FLOORPLAN_H
