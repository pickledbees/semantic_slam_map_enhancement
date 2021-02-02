//
// Created by han on 21/1/21.
//

#include <landmark_extractor/semantic_floorplan.h>
#include <landmark_extractor/label_service.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stack>

bool SemanticFloorPlan::hasLabel(uint8_t r, uint8_t g, uint8_t b) {
    return map_.find(LabelService::RGBToIntHash(r, g, b)) != map_.end();
}

void SemanticFloorPlan::insertCoord(FloorPlanCoord coord) {
    if (map_.find(coord.label) == map_.end()) map_.insert({coord.label, std::vector<FloorPlanCoord>()});
    map_[coord.label].push_back(coord);
}

FloorPlanCoord SemanticFloorPlan::getCentre(int xStart, int yStart, cv::Mat &image, cv::Mat &visited) {
    std::stack<cv::Point> stack;
    stack.push(cv::Point(xStart, yStart));

    double sumX = 0, sumY = 0;
    int x, y, count = 0, boundX = image.cols - 1, boundY = image.rows - 1;
    cv::Point p;
    cv::Vec3b pixel;
    while (!stack.empty()) {
        x = stack.top().x;
        y = stack.top().y;
        stack.pop();

        pixel = image.at<cv::Vec3b>(x, y);

        if (!LabelService::isLandmark(pixel[2], pixel[1], pixel[0]) || visited.at<unsigned char>(x, y) == 1) continue;

        sumX += x;
        sumY += y;
        count++;
        visited.at<unsigned char>(x, y) = 1;

        if (x > 0) stack.push(cv::Point(x - 1, y));
        if (x < boundX) stack.push(cv::Point(x + 1, y));
        if (y > 0) stack.push(cv::Point(x, y - 1));
        if (y < boundY) stack.push(cv::Point(x, y + 1));
    }

    FloorPlanCoord coord{};
    coord.x = (int) (sumX / count);
    coord.y = (int) (sumY / count);
    coord.label = LabelService::RGBToIntHash(pixel[2], pixel[1], pixel[0]);

    return coord;
}

//with blob detection, requires c-style access
bool SemanticFloorPlan::populateFromImage(const std::string &filePath) {
    cv::Mat image = cv::imread(filePath, cv::IMREAD_COLOR);
    if (image.empty()) return false;

    //only work with RGB images
    if (image.channels() != 3) return false;

    cv::Mat visited = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    std::vector<FloorPlanCoord> landmarks;

    for (auto it = image.begin<cv::Vec3b>(), end = image.end<cv::Vec3b>(); it < end; it++) {
        auto pixel = *it;
        if (LabelService::isLandmark(pixel[2], pixel[1], pixel[0]) &&
            visited.at<unsigned char>(it.pos().x, it.pos().y) == 0) {
            auto coord = getCentre(it.pos().x, it.pos().y, image, visited);
            insertCoord(coord);
        }
    }

    return true;
}

std::vector<FloorPlanCoord> SemanticFloorPlan::getCoords(uint8_t r, uint8_t g, uint8_t b) {
    return map_.find(LabelService::RGBToIntHash(r, g, b)) == map_.end() ? std::vector<FloorPlanCoord>()
                                                                        : map_[LabelService::RGBToIntHash(r, g, b)];
}
