//
// Created by han on 21/1/21.
//

#ifndef SRC_MARKOV_MATCHER_H
#define SRC_MARKOV_MATCHER_H

#include <landmark_extractor/ExtractorLandmarks.h>
#include <landmark_extractor/ExtractorLandmark.h>
#include <landmark_extractor/semantic_floorplan.h>
#include <vector>
#include <queue>

struct MatchResults {
    std::vector<landmark_extractor::ExtractorLandmark> landmarks;
    std::vector<std::pair<std::vector<FloorPlanCoord>, double>> chains;
};

class MarkovMatcherChain {
public:
    explicit MarkovMatcherChain(FloorPlanCoord head);

    //MarkovMatcherChain(const MarkovMatcherChain &c);

    std::vector<FloorPlanCoord> coords;
    double correlation = 1;
    struct {
        double x = 0, y = 0, xy = 0, xSquare = 0, ySquare = 0;
    } sum;

    friend bool operator<(const MarkovMatcherChain &c1, const MarkovMatcherChain &c2);

    friend bool operator>(const MarkovMatcherChain &c1, const MarkovMatcherChain &c2);

    void appendCoord(int *pattern, std::vector<landmark_extractor::ExtractorLandmark> &landmark, FloorPlanCoord next);

    //TODO: clean up debug functions
    std::string getSummary() const;

    std::string getChainString() const;
};

class MarkovMatcher {
public:
    MarkovMatcher(int top, double minCorrelationThreshold, int maxBufferSize);

    MatchResults match(landmark_extractor::ExtractorLandmarks &landmarksMsg, SemanticFloorPlan &fp) const;

private:
    int top_ = 3;                           //top results to take
    double min_correlation_threshold_ = 0;  //iteratively discard chains that fall below this threshold
    int max_buffer_size = -1;               //max size of buffer to calculate chains
};

#endif //SRC_MARKOV_MATCHER_H
