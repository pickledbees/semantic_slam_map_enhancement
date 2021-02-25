//
// Created by han on 18/2/21.
//

#ifndef SRC_COORDINATESMATCHER_H
#define SRC_COORDINATESMATCHER_H

#include <map_localiser/MatchResult.h>
#include <map_localiser/ExtractorLandmarks.h>
#include <map_localiser/Floorplan.h>

class CoordinatesMatcher {
public:
    CoordinatesMatcher(int top, double minCorrThreshold, size_t minChainLength, size_t maxBufferSize, int strategy);

    map_localiser::MatchResult match(const map_localiser::ExtractorLandmarks &landmark, Floorplan &fp) const;

private:
    int top_ = 10;
    double min_correlation_threshold_ = -0.3;
    size_t min_chain_length_ = 2;
    size_t max_buffer_size_ = 30;
    int strategy_ = 0;
};


#endif //SRC_COORDINATESMATCHER_H
