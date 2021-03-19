//
// Created by han on 18/3/21.
//

#include <map_localiser/RunningCovariance.h>

void RunningCovariance::insert(double x, double y) {
    n_++;
    if (n_ == 1) {
        old_m_x = new_m_x = x;
        old_m_y = new_m_y = y;
        old_s_xy = 0;
        old_s_x = 0;
        old_s_y = 0;
    } else {
        new_m_x = old_m_x + (x - old_m_x) / n_;
        new_m_y = old_m_y + (y - old_m_y) / n_;
        new_s_xy = old_s_xy + (x - old_m_x) * (y - new_m_y);
        new_s_x = old_s_x + (x - old_m_x) * (x - new_m_x);
        new_s_y = old_s_y + (y - old_m_y) * (y - new_m_y);

        old_m_x = new_m_x;
        old_m_y = new_m_y;
        old_s_xy = new_s_xy;
        old_s_x = new_s_x;
        old_s_y = new_s_y;
    }
}

double RunningCovariance::covarianceXY() const {
    return n_ > 1 ? new_s_xy / (n_ - 1) : 0;
}

double RunningCovariance::varianceX() const {
    return n_ > 1 ? new_s_x / (n_ - 1) : 0;
}

double RunningCovariance::varianceY() const {
    return n_ > 1 ? new_s_y / (n_ - 1) : 0;
}
