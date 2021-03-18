//
// Created by han on 18/3/21.
//

#include <map_localiser/RunningVariance.h>

void RunningVariance::insert(double x) {
    n_++;
    if (n_ == 1) {
        old_m = new_m = x;
        old_s = 0;
    } else {
        new_m = old_m + (x - old_m) / n_;
        new_s = old_s + (x - old_m) * (x - new_m);

        old_m = new_m;
        old_s = new_s;
    }
}

double RunningVariance::variance() const {
    return n_ > 1 ? new_s / n_ - 1 : 0;
}
