//
// Created by han on 18/3/21.
//

#ifndef SRC_WELFORDCORRELATIONCHAIN_H
#define SRC_WELFORDCORRELATIONCHAIN_H

#include <map_localiser/MatchedChain.h>
#include <map_localiser/ExtractorLandmark.h>
#include <map_localiser/RunningCovariance.h>
#include <map_localiser/RunningVariance.h>

class WelfordCorrelationChain {
public:
    class Element {
    public:
        double x_, y_;
        int hash_;

        Element(double x, double y, int hash);
    };

    WelfordCorrelationChain(double x, double y, int hash, std::vector<map_localiser::ExtractorLandmark> &pattern);

    void appendCoord(double x, double y, int hash);

    bool operator<(const WelfordCorrelationChain &c) const;

    bool operator>(const WelfordCorrelationChain &c) const;

    WelfordCorrelationChain &operator=(const WelfordCorrelationChain &c);

    double getCorrelation() const;

    map_localiser::MatchedChain toMatchedChain() const;

    std::string getSummary() const;

private:
    std::vector<map_localiser::ExtractorLandmark> &pattern_;
    RunningCovariance cov_;
    std::vector<Element> elements_;
    double correlation_ = 1;
};


#endif //SRC_WELFORDCORRELATIONCHAIN_H
