//
// Created by han on 18/3/21.
//

#ifndef SRC_RUNNINGVARIANCE_H
#define SRC_RUNNINGVARIANCE_H


class RunningVariance {
public:
    void insert(double x);

    double variance() const;

private:
    int n_ = 0;
    double old_m{}, new_m{}, old_s{}, new_s{};
};


#endif //SRC_RUNNINGVARIANCE_H
