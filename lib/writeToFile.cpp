#include "writeToFile.hpp"

time_t start;

void init_filewriter(){
    start = time(0);
}

void append_odometry(VectorXf pos, MatrixXf cov) {
    ofstream log("odometry.txt", ios_base::app | ios_base::out);
    VectorXf flattened_cov = Map<VectorXf>(cov.data(), cov.cols() * cov.rows());
    VectorXf row(1 + pos.size() + flattened_cov.size());

    time_t now = time(0) - start;
    row(0) = static_cast<float>(now);
    row.segment(1, pos.size()) = pos;

    // Add covariance elements to the row
    for (int i = 0; i < cov.rows(); i++) {
        for (int j = 0; j < cov.cols(); j++) {
            row(1 + pos.size() + i * cov.cols() + j) = cov(i, j);
        }
    }

    // Loop through the row and check if any elements are nan
    for (int i = 0; i < row.size(); i++) {
        if (row(i) != row(i)) {
            return;
        }
    }

    // Loop through the row and write every element to the file with a space in between
    for (int i = 0; i < row.size(); i++) {
        log << row(i) << " ";
    }
    log << endl;
}

void append_cox(VectorXf pos, MatrixXf cov) {
    ofstream log("cox.txt", ios_base::app | ios_base::out);
    VectorXf flattened_cov = Map<VectorXf>(cov.data(), cov.cols() * cov.rows());
    VectorXf row(1 + pos.size() + flattened_cov.size());

    time_t now = time(0) - start;
    row(0) = static_cast<float>(now);
    row.segment(1, pos.size()) = pos;

    // Add covariance elements to the row
    for (int i = 0; i < cov.rows(); i++) {
        for (int j = 0; j < cov.cols(); j++) {
            row(1 + pos.size() + i * cov.cols() + j) = cov(i, j);
        }
    }

    // Loop through the row and check if any elements are nan
    for (int i = 0; i < row.size(); i++) {
        if (row(i) != row(i)) {
            return;
        }
    }

    // Loop through the row and write every element to the file with a space in between
    for (int i = 0; i < row.size(); i++) {
        log << row(i) << " ";
    }
    log << endl;
}

void append_kalman(VectorXf pos, MatrixXf cov) {
    ofstream log("kalman.txt", ios_base::app | ios_base::out);
    VectorXf flattened_cov = Map<VectorXf>(cov.data(), cov.cols() * cov.rows());
    VectorXf row(1 + pos.size() + flattened_cov.size());

    time_t now = time(0) - start;
    row(0) = static_cast<float>(now);
    row.segment(1, pos.size()) = pos;

    // Add covariance elements to the row
    for (int i = 0; i < cov.rows(); i++) {
        for (int j = 0; j < cov.cols(); j++) {
            row(1 + pos.size() + i * cov.cols() + j) = cov(i, j);
        }
    }

    // Loop through the row and check if any elements are nan
    for (int i = 0; i < row.size(); i++) {
        if (row(i) != row(i)) {
            return;
        }
    }

    // Loop through the row and write every element to the file with a space in between
    for (int i = 0; i < row.size(); i++) {
        log << row(i) << " ";
    }
    log << endl;
}
