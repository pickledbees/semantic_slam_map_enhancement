//
// Created by han on 24/2/21.
//

#include <map_localiser/CorrelationChain.h>

CorrelationChain::Element::Element(double x, double y, int hash) : x_(x), y_(y), hash_(hash) {}

CorrelationChain::CorrelationChain(double x, double y, int hash,
                                   std::vector<map_localiser::ExtractorLandmark> &pattern) : pattern_(pattern) {
    elements_.emplace_back(x, y, hash);
}

CorrelationChain::CorrelationChain(const CorrelationChain &c) : pattern_(c.pattern_) {
    sumX_ = c.sumX_;
    sumY_ = c.sumY_;
    sumXX_ = c.sumXX_;
    sumYY_ = c.sumYY_;
    sumXY_ = c.sumXY_;
    correlation_ = c.correlation_;
    elements_ = c.elements_;
}

CorrelationChain &CorrelationChain::operator=(const CorrelationChain &c) {
    sumX_ = c.sumX_;
    sumY_ = c.sumY_;
    sumXX_ = c.sumXX_;
    sumYY_ = c.sumYY_;
    sumXY_ = c.sumXY_;
    correlation_ = c.correlation_;
    elements_ = c.elements_;
    return *this;
}

inline double
distanceBetweenLandmarks(const map_localiser::ExtractorLandmark &a, const map_localiser::ExtractorLandmark &b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

inline double
distanceBetweenCoords(const CorrelationChain::Element &a, const CorrelationChain::Element &b) {
    return sqrt(pow(a.x_ - b.x_, 2) + pow(a.y_ - b.y_, 2));
}

void CorrelationChain::appendCoord(double x, double y, int hash) {
    size_t n = elements_.size() + 1;
    if (n > pattern_.size()) return;    //do not perform match if n is out of bounds

    elements_.emplace_back(x, y, hash);

    //get sums
    auto p = pattern_[n - 1];
    auto e = elements_[n - 1];

    //perform correlation with other points
    for (int i = 0; i < n - 1; i++) {
        double dx = distanceBetweenLandmarks(p, pattern_[i]);
        double dy = distanceBetweenCoords(e, elements_[i]);
        sumX_ += dx;
        sumY_ += dy;
        sumXY_ += dx * dy;
        sumXX_ += dx * dx;
        sumYY_ += dy * dy;
        //maybe cache the distances to speed up execution?
    }

    correlation_ = (n * sumXY_ - sumX_ * sumY_) / sqrt((n * sumXX_ - sumX_ * sumX_) * (n * sumYY_ - sumY_ * sumY_));
}

bool CorrelationChain::operator<(const CorrelationChain &c) const {
    return correlation_ < c.correlation_;
}

bool CorrelationChain::operator>(const CorrelationChain &c) const {
    return correlation_ > c.correlation_;
}

double CorrelationChain::getCorrelation() const {
    return correlation_;
}

map_localiser::MatchedChain CorrelationChain::toMatchedChain() const {
    int n = elements_.size();
    double m = (n * sumXY_ - sumX_ * sumY_) / (n * sumXX_ - sumX_ * sumX_);
    double b = sumY_ / n - m * sumX_ / n;

    /*
     * each landmark in the pattern is matched to a floorplan coord
     * each landmark can produce a set of x values, with each x value having a corresponding 'ideal' y value
     * each x value has a y value derived from the floorplan coord calcs
     * each x value is derived from calculating the distance of that landmark to the other landmarks
         * its matched y value is the distance between corresponding floorplan coords
         * its ideal y value is derived from the line
         * the error of that x value is the distance between the matched and ideal y values
     * the error of that match is the average error of its x values
     */

    map_localiser::MatchedChain chain;
    chain.correlation = correlation_;

    //error calculation
    for (int i = 0; i < pattern_.size(); i++) {
        double totalError = 0;
        auto p = pattern_[i];
        auto e = elements_[i];
        for (int j = 0; j < pattern_.size(); j++) {
            if (j == i) continue;
            double x = distanceBetweenLandmarks(p, pattern_[j]);
            double predY = m * x + b;
            double y = distanceBetweenCoords(e, elements_[j]);
            totalError = totalError + abs(predY - y);
        }
        map_localiser::MatchedChainElement mce;
        mce.x = e.x_;
        mce.y = e.y_;
        mce.hash = e.hash_;
        mce.error = totalError / (double) (pattern_.size() - 1);
    }

    return chain;
}