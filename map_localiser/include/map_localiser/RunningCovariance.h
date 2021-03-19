//
// Created by han on 18/3/21.
//

#ifndef SRC_RUNNINGCOVARIANCE_H
#define SRC_RUNNINGCOVARIANCE_H

//maintains a running covariance score between 2 variables
class RunningCovariance {
public:
    void insert(double x, double y);

    double covarianceXY() const;

    double varianceX() const;

    double varianceY() const;

private:
    int n_ = 0;
    double old_m_x{}, new_m_x{}, old_m_y{}, new_m_y{},
            old_s_xy{}, new_s_xy{},
            old_s_x{}, new_s_x{},
            old_s_y{}, new_s_y{};
};


#endif //SRC_RUNNINGCOVARIANCE_H
