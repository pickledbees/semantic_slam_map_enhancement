//
// Created by han on 18/2/21.
//

#include <map_localiser/CoordinatesMatcher.h>
#include <map_localiser/CorrelationChain.h>
#include <queue>

CoordinatesMatcher::CoordinatesMatcher(int top, double minCorrThreshold, size_t minChainLength, size_t maxBufferSize,
                                       int strategy) : top_(
        top), min_correlation_threshold_(minCorrThreshold), min_chain_length_(minChainLength),
                                                       max_buffer_size_(maxBufferSize), strategy_(strategy) {}

map_localiser::MatchResult
CoordinatesMatcher::match(const map_localiser::ExtractorLandmarks &landmarks, Floorplan &fp) const {
    //filter applicable landmarks
    std::vector<map_localiser::ExtractorLandmark> pattern;
    std::copy_if(landmarks.landmarks.begin(), landmarks.landmarks.end(), std::back_inserter(pattern),
                 [&f = fp](map_localiser::ExtractorLandmark l) { return f.hasHash(l.hash); });

    //cannot perform any meaningful correlation if below min chain length
    if (pattern.size() < min_chain_length_ || pattern.size() < 3) return {};

    std::priority_queue<CorrelationChain, std::vector<CorrelationChain>, std::greater<>> q; //sorts with lowest correlation on top
    map_localiser::ExtractorLandmark l;
    std::vector<Floorplan::Coord> candidates;

    l = pattern[0];
    candidates = fp.getCoords(l.hash);
    for (const auto &c : candidates) {
        CorrelationChain chain(c.x_, c.y_, c.hash_, pattern);
        q.push(chain);
    }

    std::queue<CorrelationChain> temp;
    for (int i = 1; i < pattern.size(); i++) {
        l = pattern[i];
        candidates = fp.getCoords(l.hash);

        while (!q.empty()) {
            CorrelationChain chain = q.top();
            q.pop();

            for (const auto &c : candidates) {
                auto newChain = chain;
                newChain.appendCoord(c.x_, c.y_, c.hash_);
                if (newChain.getCorrelation() >= min_correlation_threshold_) temp.push(newChain);
            }
        }

        while (!temp.empty()) {
            q.push(temp.front());
            temp.pop();
        }

        if (max_buffer_size_ > 0) {
            while (q.size() > max_buffer_size_) q.pop();
        }
    }

    map_localiser::MatchResult result;
    result.pattern = pattern;
    while (!q.empty()) {
        result.chains.push_back(q.top().toMatchedChain());
        q.pop();
    }

    return result;
}
