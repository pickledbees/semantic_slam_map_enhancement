//
// Created by han on 18/2/21.
//

#include <map_localiser/Floorplan.h>
#include <map_localiser/LabelService.h>
#include <stack>

Floorplan::Coord::Coord(double x, double y, int hash) : x_(x), y_(y), hash_(hash) {}

void Floorplan::loadFromFile(const std::string &filePath, bool aggregate) {
    cv::Mat image = cv::imread(filePath, cv::IMREAD_COLOR);

    if (image.empty()) {
        has_error_ = true;
        error_ = "image at '" + filePath + "' not found";
    }

    if (image.channels() != 3) {
        has_error_ = true;
        error_ = "image is not in RGB, will not load";
    }

    auto it = image.begin<cv::Vec3b>(), end = image.end<cv::Vec3b>();
    if (aggregate) {
        cv::Mat visited = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
        for (; it < end; it++) {
            auto pixel = *it;
            int hash = LabelService::RGBToHash(pixel[2], pixel[1], pixel[0]);
            if (visited.at<unsigned char>(it.pos().y, it.pos().x) == 0 &&
                LabelService::isLandmark(hash)) {
                insert(getCentre(it.pos().x, it.pos().y, image, visited));
            }
        }
    } else {
        for (; it < end; it++) {
            auto pixel = *it;
            int hash = LabelService::RGBToHash(pixel[2], pixel[1], pixel[0]);
            insert(Floorplan::Coord(it.pos().x, it.pos().y, hash));
        }
    }

    error_ = "no error";
    has_error_ = false;
    loaded_ = true;
}

void Floorplan::insert(Floorplan::Coord coord) {
    if (map_.find(coord.hash_) == map_.end()) map_.insert({coord.hash_, std::vector<Floorplan::Coord>()});
    map_[coord.hash_].push_back(coord);
}

bool Floorplan::isLoaded() const {
    return loaded_;
}

bool Floorplan::hasError() const {
    return has_error_;
}

std::string Floorplan::getError() {
    return has_error_ ? error_ : "no error";
}

bool Floorplan::hasHash(int hash) {
    return map_.find(hash) != map_.end();
}

std::vector<Floorplan::Coord> Floorplan::getCoords(int hash) {
    auto pair = map_.find(hash);
    return pair != map_.end() ? pair->second : std::vector<Floorplan::Coord>();
}

std::string Floorplan::getSummary() {
    std::ostringstream summary;
    summary << "FLOORPLAN SUMMARY" << std::endl;

    //get number of landmarks
    int num = 0;
    for (const auto &v : map_) {
        num += v.second.size();
    }

    summary << "* Total no. landmarks: " << num << std::endl;
    summary << "* Total no. types: " << map_.size() << std::endl;
    summary << "TYPE COUNTS" << std::endl;

    for (const auto &v : map_) {
        summary << "** " << LabelService::hashToLabel(v.first) << ": " << v.second.size() << std::endl;
    }

    return summary.str();
}

Floorplan::Coord Floorplan::getCentre(int colStart, int rowStart, cv::Mat &image, cv::Mat &visited) {
    std::stack<cv::Point> stack;
    stack.push(cv::Point(colStart, rowStart));

    auto pixel = image.at<cv::Vec3b>(rowStart, colStart);
    int hash = LabelService::RGBToHash(pixel[2], pixel[1], pixel[0]);

    double sumCol = 0, sumRow = 0;
    int col, row, count = 0, boundCols = image.cols - 1, boundRows = image.rows - 1;

    //dfs to accumulate centre
    if (visited.at<unsigned char>(row, col) == 0 && LabelService::RGBToHash(pixel[2], pixel[1], pixel[0]) == hash) {
        sumCol += col;
        sumRow += row;
        count++;
        visited.at<unsigned char>(row, col) = 1;

        if (col > 0) stack.push(cv::Point(col - 1, row));
        if (col < boundCols) stack.push(cv::Point(col + 1, row));
        if (row > 0) stack.push(cv::Point(col, row - 1));
        if (row < boundRows) stack.push(cv::Point(col, row + 1));
    }

    Floorplan::Coord coord(sumCol / count, sumRow / count, hash);
    return coord;
}
