//
// Created by han on 18/2/21.
//

#ifndef SRC_FLOORPLAN_H
#define SRC_FLOORPLAN_H

#include <map_localiser/ExtractorLandmark.h>
#include <unordered_map>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

class Floorplan {
public:
    class Coord {
    public:
        double x_, y_;
        int hash_;

        Coord(double x, double y, int hash);
    };

    void loadFromFile(const std::string &filePath, bool aggregate);

    void insert(Floorplan::Coord coord);

    bool isLoaded() const;

    bool hasError() const;

    std::string getError();

    bool hasHash(int hash);

    std::vector<Coord> getCoords(int hash);

    std::string getSummary();

private:
    bool loaded_ = false;
    bool has_error_ = false;
    std::string error_;

    std::unordered_map<int, std::vector<Coord>> map_;

    Floorplan::Coord getCentre(int colStart, int rowStart, cv::Mat &image, cv::Mat &visited);
};

#endif //SRC_FLOORPLAN_H
