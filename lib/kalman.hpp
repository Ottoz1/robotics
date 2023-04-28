#include <Eigen/Dense>

#pragma once

class KalmanFilter {
public:
    KalmanFilter(
        double dt,
        Eigen::VectorXd state,
        Eigen::MatrixXd state_covariance,
        Eigen::MatrixXd process_noise,
        Eigen::MatrixXd measurement_noise,
        Eigen::MatrixXd transition_matrix,
        Eigen::MatrixXd control_matrix,
    );

    void init(const Eigen::VectorXd &state, const Eigen::MatrixXd &state_covariance);
    void update(const Eigen::VectorXd &measurement);
}