//
// Created by han on 24/2/21.
//

#ifndef SRC_CORRELATIONCHAIN_H
#define SRC_CORRELATIONCHAIN_H

#include <map_localiser/MatchedChain.h>
#include <map_localiser/ExtractorLandmark.h>

class CorrelationChain {
public:
    class Element {
    public:
        double x_, y_;
        int hash_;

        Element(double x, double y, int hash);
    };

    CorrelationChain(double x, double y, int hash, std::vector<map_localiser::ExtractorLandmark> &pattern);

    CorrelationChain(const CorrelationChain &c);

    void appendCoord(double x, double y, int hash);

    bool operator<(const CorrelationChain &c) const;

    bool operator>(const CorrelationChain &c) const;

    CorrelationChain &operator=(const CorrelationChain &c);

    double getCorrelation() const;

    map_localiser::MatchedChain toMatchedChain() const;

    std::string getSummary() const;

private:
    std::vector<map_localiser::ExtractorLandmark> &pattern_;
    double sumX_ = 0, sumY_ = 0, sumXX_ = 0, sumYY_ = 0, sumXY_ = 0, n_ = 0, correlation_ = 1;
    std::vector<Element> elements_;
};


#endif //SRC_CORRELATIONCHAIN_H
