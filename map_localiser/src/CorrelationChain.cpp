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

    if (n > pattern_.size()) return;
    elements_.emplace_back(x, y, hash);

    double dx = distanceBetweenLandmarks(pattern_[n - 1], pattern_[n - 2]);
    double dy = distanceBetweenCoords(elements_[n - 1], elements_[n - 2]);
    sumX_ += dx;
    sumY_ += dy;
    sumXY_ += dx * dy;
    sumXX_ += dx * dx;
    sumYY_ += dy * dy;
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

    map_localiser::MatchedChain chain;
    chain.correlation = correlation_;
    for (const auto &e : elements_) {
        map_localiser::MatchedChainElement mce;
        mce.x = e.x_;
        mce.y = e.y_;
        mce.hash = e.hash_;
        mce.error = calculateError(m * e.x_ + b, e.y_);
        chain.elements.push_back(mce);
    }

    return chain;
}

//TODO: maybe try using gaussian error?
double CorrelationChain::calculateError(double yPred, double yActual) const {
    return yPred - yActual;
}
