//
// Created by han on 5/1/21.
//

#include <landmark_extractor/extractor_landmark.h>

Landmark::Landmark() = default;

Landmark::Landmark(uint8_t r, uint8_t g, uint8_t b, double x, double y, double z, double distance, unsigned int depth)
        : r_(r),
          g_(g),
          b_(b),
          x_(x),
          y_(y),
          z_(z),
          distance_(
                  distance),
          depth_(depth) {
}

bool operator<(const Landmark &a, const Landmark &b) {
    return a.distance_ < b.distance_;
}

bool operator>(const Landmark &a, const Landmark &b) {
    return a.distance_ > b.distance_;
}

landmark_extractor::ExtractorLandmark Landmark::toMsg() const {
    landmark_extractor::ExtractorLandmark l;
    l.r = r_;
    l.g = g_;
    l.b = b_;
    l.x = x_;
    l.y = y_;
    l.z = z_;
    l.distance = distance_;
    return l;
}
