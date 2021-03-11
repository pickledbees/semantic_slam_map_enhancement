//
// Created by han on 18/2/21.
//

#include <map_localiser/ExtractorLandmarkWrapper.h>

ExtractorLandmarkWrapper::ExtractorLandmarkWrapper(double x, double y, double z, int hash, double distance,
                                                   unsigned int depth) : x_(x), y_(y), z_(z), hash_(hash),
                                                                         distance_(distance), depth_(depth) {
}

bool operator<(const ExtractorLandmarkWrapper &l1, const ExtractorLandmarkWrapper &l2) {
    return l1.distance_ < l2.distance_;
}

bool operator>(const ExtractorLandmarkWrapper &l1, const ExtractorLandmarkWrapper &l2) {
    return l1.distance_ > l2.distance_;
}

double ExtractorLandmarkWrapper::getDistance() const {
    return distance_;
}

unsigned int ExtractorLandmarkWrapper::getDepth() const {
    return depth_;
}

map_localiser::ExtractorLandmark ExtractorLandmarkWrapper::getLandmark() const {
    map_localiser::ExtractorLandmark l;
    l.x = x_;
    l.y = y_;
    l.z = z_;
    l.hash = hash_;
    return l;
}
