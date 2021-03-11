//
// Created by han on 18/2/21.
//

#ifndef SRC_EXTRACTORLANDMARKWRAPPER_H
#define SRC_EXTRACTORLANDMARKWRAPPER_H

#include <map_localiser/ExtractorLandmark.h>

class ExtractorLandmarkWrapper {
public:
    ExtractorLandmarkWrapper(double x, double y, double z, int hash, double distance, unsigned int depth = 0);

    friend bool operator<(const ExtractorLandmarkWrapper &l1, const ExtractorLandmarkWrapper &l2);

    friend bool operator>(const ExtractorLandmarkWrapper &l1, const ExtractorLandmarkWrapper &l2);

    double getDistance() const;

    unsigned int getDepth() const;

    map_localiser::ExtractorLandmark getLandmark() const;

    double x_, y_, z_, distance_;
    int hash_;
    unsigned int depth_;
private:
};


#endif //SRC_EXTRACTORLANDMARKWRAPPER_H
