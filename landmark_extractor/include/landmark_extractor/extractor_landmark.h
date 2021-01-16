//
// Created by han on 5/1/21.
//

#ifndef SRC_EXTRACTOR_LANDMARK_H
#define SRC_EXTRACTOR_LANDMARK_H

#include <cstdint>
#include <string>
#include <landmark_extractor/ExtractorLandmark.h>

class Landmark {
public:
    uint8_t r_;
    uint8_t g_;
    uint8_t b_;
    double x_;
    double y_;
    double z_;
    double distance_;
    unsigned int depth_;

    Landmark();

    Landmark(uint8_t r, uint8_t g, uint8_t b, double x, double y, double z, double distance, unsigned int depth);

    friend bool operator<(const Landmark &a, const Landmark &b);

    friend bool operator>(const Landmark &a, const Landmark &b);

    landmark_extractor::ExtractorLandmark toMsg() const;
};


#endif //SRC_EXTRACTOR_LANDMARK_H
