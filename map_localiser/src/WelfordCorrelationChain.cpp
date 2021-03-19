//
// Created by han on 18/3/21.
//

#include <map_localiser/WelfordCorrelationChain.h>

//extract only the lateral distances
inline double
distanceBetweenLandmarks(const map_localiser::ExtractorLandmark &a, const map_localiser::ExtractorLandmark &b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.z - b.z, 2));
}

inline double
distanceBetweenCoords(const WelfordCorrelationChain::Element &a, const WelfordCorrelationChain::Element &b) {
    return sqrt(pow(a.x_ - b.x_, 2) + pow(a.y_ - b.y_, 2));
}

WelfordCorrelationChain::WelfordCorrelationChain(double x, double y, int hash,
                                                 std::vector<map_localiser::ExtractorLandmark> &pattern) : pattern_(
        pattern) {
    elements_.emplace_back(x, y, hash);
}

void WelfordCorrelationChain::appendCoord(double x, double y, int hash) {
    elements_.emplace_back(x, y, hash);
    int len = elements_.size();
    if (len > 1) {
        auto p = pattern_[len - 1];
        auto e = elements_[len - 1];
        for (int i = 0; i < len - 1; i++) {
            double _x = distanceBetweenLandmarks(p, pattern_[i]);
            double _y = distanceBetweenCoords(e, elements_[i]);
            cov_.insert(_x, _y);
        }
    }

    if (len > 2) {
        correlation_ = cov_.covarianceXY() / sqrt(cov_.varianceX() * cov_.varianceY());
    }
}

bool WelfordCorrelationChain::operator<(const WelfordCorrelationChain &c) const {
    return correlation_ < c.correlation_;
}

bool WelfordCorrelationChain::operator>(const WelfordCorrelationChain &c) const {
    return correlation_ > c.correlation_;
}

WelfordCorrelationChain &WelfordCorrelationChain::operator=(const WelfordCorrelationChain &c) {
    cov_ = c.cov_;
    elements_ = c.elements_;
    correlation_ = c.correlation_;
    return *this;
}

double WelfordCorrelationChain::getCorrelation() const {
    return correlation_;
}

map_localiser::MatchedChain WelfordCorrelationChain::toMatchedChain() const {
    map_localiser::MatchedChain chain;
    for (auto &e : elements_) {
        map_localiser::MatchedChainElement mce;
        mce.x = e.x_;
        mce.y = e.y_;
        mce.hash = e.hash_;
        mce.error = 0; //TODO: calculate the error
        chain.elements.push_back(mce);
    }
    chain.correlation = correlation_;

    return chain;
}

std::string WelfordCorrelationChain::getSummary() const {
    std::ostringstream stream;
    stream << "len=" << elements_.size() << " covXY=" << cov_.covarianceXY() << " varX=" << cov_.varianceX() << " varY="
           << cov_.varianceY() << " corr=" << correlation_;
    return stream.str();
}

WelfordCorrelationChain::Element::Element(double x, double y, int hash) : x_(x), y_(y), hash_(hash) {}
