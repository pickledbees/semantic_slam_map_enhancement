//
// Created by han on 19/3/21.
//

#include <ros/ros.h>
#include <map_localiser/RunningCovariance.h>

#define ARR_SIZE 7

double mean(const double arr[], int n)
{
    double sum = 0;
    for(int i = 0; i < n; i++)
        sum = sum + arr[i];
    return sum / n;
}

// usual covariance calculation (non-running)
double covariance(double arr1[], double arr2[], int n)
{
    double sum = 0;
    for(int i = 0; i < n; i++)
        sum = sum + (arr1[i] - mean(arr1, n)) *
                    (arr2[i] - mean(arr2, n));
    return sum / (n - 1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test");

    ROS_INFO("test start");

    double X[ARR_SIZE] = {1,3,4,6,44,6,7};
    double Y[ARR_SIZE] = {3,4,5,0,89,3,66};

    RunningCovariance cov;
    for (int i = 0; i < ARR_SIZE; i++) {
        cov.insert(X[i], Y[i]);
    }

    ROS_INFO("out_covXY=%f actual_covXY=%f", cov.covarianceXY(), covariance(X, Y, ARR_SIZE));
    ROS_INFO("out_varX=%f actual_varX%f", cov.varianceX(), covariance(X, X, ARR_SIZE));
    ROS_INFO("out_varY=%f actual_varY%f", cov.varianceY(), covariance(Y, Y, ARR_SIZE));

    return 0;
}