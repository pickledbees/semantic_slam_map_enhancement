//
// Created by han on 21/1/21.
//

#include <landmark_extractor/markov_matcher.h>
#include <landmark_extractor/label_service.h>
#include <unordered_set>
#include <queue>

inline double
distanceBetweenLandmarks(landmark_extractor::ExtractorLandmark &l1, landmark_extractor::ExtractorLandmark &l2) {
    return sqrt(pow(l1.x - l2.x, 2) + pow(l1.y - l2.y, 2));
}

inline double distanceBetweenFloorPlanCoords(FloorPlanCoord *c1, FloorPlanCoord *c2) {
    return sqrt(pow(c1->x - c2->x, 2) + pow(c1->y - c2->y, 2));
}

MarkovMatcherChain::MarkovMatcherChain(FloorPlanCoord *head) {
    coords.push_back(head);
}

bool operator<(const MarkovMatcherChain &c1, const MarkovMatcherChain &c2) {
    return c1.correlation < c2.correlation;
}

bool operator>(const MarkovMatcherChain &c1, const MarkovMatcherChain &c2) {
    return c1.correlation > c2.correlation;
}

//performs correlation calculation optimised with memoization from previous chains
void MarkovMatcherChain::appendCoord(int *pattern, std::vector<landmark_extractor::ExtractorLandmark> &landmarks,
                                     FloorPlanCoord *next) {
    coords.push_back(next);
    int size = coords.size();
    //compute dx and dy from difference of last 2
    double dx = distanceBetweenLandmarks(landmarks[pattern[size - 1]], landmarks[pattern[size - 2]]);
    double dy = distanceBetweenFloorPlanCoords(coords[size - 1], coords[size - 2]);
    sum.x = sum.x + dx;
    sum.y = sum.y + dy;
    sum.xy = sum.xy + dx * dy;
    sum.xSquare = sum.xSquare + dx * dx;
    sum.ySquare = sum.ySquare + dy * dy;
    correlation = (size * sum.xy - sum.x * sum.y) /
                  sqrt((size * sum.xSquare - sum.x * sum.x) * (size * sum.ySquare - sum.y * sum.y));
}

MarkovMatcher::MarkovMatcher(int top, double minCorrelationThreshold, int maxBufferSize) : top_(top),
                                                                                           min_correlation_threshold_(
                                                                                                   minCorrelationThreshold),
                                                                                           max_buffer_size(
                                                                                                   maxBufferSize) {}

MatchResults MarkovMatcher::match(landmark_extractor::ExtractorLandmarks &landmarksMsg,
                                  SemanticFloorPlan &fp) const {
    /*
     * line up landmarks + remove unidentified landmarks to form pattern
     *
     * initialise chains by selecting all candidates for the first landmark in pattern
     *
     * build chains
     *  for each landmark in pattern (starting from second)
     *      find candidates for landmark
     *      for each chain
     *          for each candidate
     *              append candidate to chain and calculate correlation
     *              if correlation < threshold, put chain back into circulation for consideration for next landmark
     *      trim number of chains by discarding the lowest X chains which have the lowest correlation
     *  if multiple chains remain, extract top few chains with highest correlation
     */

    //filter out only those applicable detections
    int *indexes = new int[landmarksMsg.landmarks.size()];  //indexes of useful landmarks
    int i, j;
    for (i = 0, j = 0; i < landmarksMsg.landmarks.size(); i++) {
        landmark_extractor::ExtractorLandmark &l = landmarksMsg.landmarks[j];
        if (fp.hasLabel(l.r, l.g, l.b)) {
            indexes[j] = i;
            j++;
        }
    }

    //j is know indicator of how many elements there are in the pattern
    if (j < 1) return {};    //cannot perform match if filtered < 2;

    //line up the landmarks (may remove since not really required)
    int *pattern = new int[j];  //indexes of landmarks in order
    int prev, k, closest;
    prev = k = 0;
    double min, distance;
    std::unordered_set<int> skip;   //indexes of index that have been assigned to pattern
    while (skip.size() != j) {      //while skip does not encompass all the indexes
        min = DBL_MAX;
        for (i = 0; i < j; i++) {
            if (skip.find(i) != skip.end() || prev == i) continue;
            distance = distanceBetweenLandmarks(landmarksMsg.landmarks[indexes[prev]],
                                                landmarksMsg.landmarks[indexes[i]]);
            if (distance < min) {
                min = distance;
                closest = i;
            }
        }
        skip.insert(closest);
        pattern[k++] = indexes[closest];
        prev = closest;
    }

    std::priority_queue<MarkovMatcherChain, std::vector<MarkovMatcherChain>, std::greater<>> chains; //chains sorted in ascending order of correlation
    landmark_extractor::ExtractorLandmark l;
    std::vector<FloorPlanCoord> candidates;

    //initialise chains
    l = landmarksMsg.landmarks[pattern[0]];
    candidates = fp.getCoords(l.r, l.g, l.b);
    for (auto candidate : candidates) {
        MarkovMatcherChain chain(&candidate);
        chains.push(chain);
    }

    //build chains
    std::queue<MarkovMatcherChain> temp;
    for (i = 1; i < j; i++) {   //iterate over pattern
        l = landmarksMsg.landmarks[i];
        candidates = fp.getCoords(l.r, l.g, l.b);

        while (!chains.empty()) {         //iterate over chains
            MarkovMatcherChain chain = chains.top();
            chains.pop();

            for (auto coord : candidates) { //iterate over candidates
                auto newChain = chain;      //copy chain
                newChain.appendCoord(pattern, landmarksMsg.landmarks, &coord);
                if (newChain.correlation >= min_correlation_threshold_) {
                    temp.push(newChain);
                }
            }
        }

        while (!temp.empty()) { //sort chains by correlation
            chains.push(temp.front());
            temp.pop();
        }

        //remove excess (throw away the lowest correlations chains)
        if (max_buffer_size > 0) {
            while (chains.size() > max_buffer_size) {
                chains.pop();
            }
        }
    }

    MatchResults results;
    while (!chains.empty()) {
        results.chains.emplace_back(chains.top().coords, chains.top().correlation);
        chains.pop();
    }
    for (i = 0; i < j; i++) results.landmarks.push_back(landmarksMsg.landmarks[pattern[i]]);

    delete[] indexes;
    delete[] pattern;

    return results;
}

