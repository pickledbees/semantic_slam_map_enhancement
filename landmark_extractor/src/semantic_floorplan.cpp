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
    if (map_.find(coord.hash) == map_.end()) map_.insert({coord.hash, std::vector<FloorPlanCoord>()});
    map_[coord.hash].push_back(coord);
}

FloorPlanCoord
SemanticFloorPlan::getCentre(int colStart, int rowStart, cv::Mat &image, cv::Mat &visited, int extractionMode) {
    if (extractionMode == 1) {
        FloorPlanCoord coord{};
        auto pixel = image.at<cv::Vec3b>(rowStart, colStart);

        coord.x = colStart;
        coord.y = rowStart;
        coord.hash = LabelService::RGBToIntHash(pixel[2], pixel[1], pixel[0]);
        return coord;
    } else {
        std::stack<cv::Point> stack;
        stack.push(cv::Point(colStart, rowStart));

        auto pixel = image.at<cv::Vec3b>(rowStart, colStart);
        int hash = LabelService::RGBToIntHash(pixel[2], pixel[1], pixel[0]);

        double sumCol = 0, sumRow = 0;
        int col, row, count = 0, boundCols = image.cols - 1, boundRows = image.rows - 1;
        while (!stack.empty()) {
            col = stack.top().x;
            row = stack.top().y;
            stack.pop();

            pixel = image.at<cv::Vec3b>(row, col);

            if (visited.at<unsigned char>(row, col) == 0 &&
                LabelService::RGBToIntHash(pixel[2], pixel[1], pixel[0]) == hash) {
                sumCol += col;
                sumRow += row;
                count++;
                visited.at<unsigned char>(row, col) = 1;

                if (col > 0) stack.push(cv::Point(col - 1, row));
                if (col < boundCols) stack.push(cv::Point(col + 1, row));
                if (row > 0) stack.push(cv::Point(col, row - 1));
                if (row < boundRows) stack.push(cv::Point(col, row + 1));
            }
        }

        FloorPlanCoord coord{};
        coord.x = sumCol / count;
        coord.y = sumRow / count;
        coord.hash = hash;

        return coord;
    }
}

bool SemanticFloorPlan::populateFromImage(const std::string &filePath, int extractionMode) {
    cv::Mat image = cv::imread(filePath, cv::IMREAD_COLOR);

    if (image.empty()) return false;

    //only work with RGB images
    if (image.channels() != 3) return false;

    cv::Mat visited = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    std::vector<FloorPlanCoord> landmarks;

    for (auto it = image.begin<cv::Vec3b>(), end = image.end<cv::Vec3b>(); it < end; it++) {
        auto pixel = *it;
        if (visited.at<unsigned char>(it.pos().y, it.pos().x) == 0 &&
            LabelService::isLandmark(pixel[2], pixel[1], pixel[0])) {
            auto coord = getCentre(it.pos().x, it.pos().y, image, visited, extractionMode);
            insertCoord(coord);
        }
    }

    return true;
}

std::vector<FloorPlanCoord> SemanticFloorPlan::getCoords(uint8_t r, uint8_t g, uint8_t b) {
    return map_.find(LabelService::RGBToIntHash(r, g, b)) == map_.end() ? std::vector<FloorPlanCoord>()
                                                                        : map_[LabelService::RGBToIntHash(r, g, b)];
}

std::string SemanticFloorPlan::getSummary() {
    std::ostringstream summary;
    summary << "FLOORPLAN SUMMARY" << std::endl;

    //get number of landmarks
    int num = 0;
    for (auto v : map_) {
        num += v.second.size();
    }

    summary << "* Total no. landmarks: " << num << std::endl;
    summary << "* Total no. types: " << map_.size() << std::endl;
    summary << "TYPE COUNTS" << std::endl;

    for (auto v : map_) {
        summary << "** " << LabelService::getLabel(v.first) << ": " << v.second.size() << std::endl;
    }

    return summary.str();
}
