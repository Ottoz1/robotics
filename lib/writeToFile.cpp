#include "writeToFile.hpp"

void append_odometry(VectorXf pos, MatrixXf cov) {
    ofstream log("odometry.txt", ios_base::app | ios_base::out);
    VectorXf flattened_cov = Map<VectorXf>(cov.data(), cov.cols() * cov.rows());
    VectorXf row(1 + pos.size() + flattened_cov.size());

    time_t now = time(0) - 1683888246;
    row(0) = static_cast<float>(now);
    row.segment(1, pos.size()) = pos;

    // Add covariance elements to the row
    for (int i = 0; i < cov.rows(); i++) {
        for (int j = 0; j < cov.cols(); j++) {
            row(1 + pos.size() + i * cov.cols() + j) = cov(i, j);
        }
    }

    //cout << "Odom time: " << now << endl;

    // Write row to file
    log << row.transpose() << endl;
}

void append_cox(VectorXf pos, MatrixXf cov) {
    ofstream log("cox.txt", ios_base::app | ios_base::out);
    VectorXf flattened_cov = Map<VectorXf>(cov.data(), cov.cols() * cov.rows());
    VectorXf row(1 + pos.size() + flattened_cov.size());

    time_t now = time(0) - 1683888246;
    row(0) = static_cast<float>(now);
    row.segment(1, pos.size()) = pos;

    // Add covariance elements to the row
    for (int i = 0; i < cov.rows(); i++) {
        for (int j = 0; j < cov.cols(); j++) {
            row(1 + pos.size() + i * cov.cols() + j) = cov(i, j);
        }
    }

    cout << "Cox time: " << now << endl;

    // Write row to file
    log << row.transpose() << endl;
}

void append_kalman(VectorXf pos, MatrixXf cov) {
    ofstream log("kalman.txt", ios_base::app | ios_base::out);
    VectorXf flattened_cov = Map<VectorXf>(cov.data(), cov.cols() * cov.rows());
    VectorXf row(1 + pos.size() + flattened_cov.size());

    time_t now = time(0) - 1683888246;
    row(0) = static_cast<float>(now);
    row.segment(1, pos.size()) = pos;

    // Add covariance elements to the row
    for (int i = 0; i < cov.rows(); i++) {
        for (int j = 0; j < cov.cols(); j++) {
            row(1 + pos.size() + i * cov.cols() + j) = cov(i, j);
        }
    }

    cout << "Kalm time: " << now << endl;

    // Write row to file
    log << row.transpose() << endl;
}
