#pragma once
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

class Transform {
public:
    Transform(){};
    Transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
        T_.setIdentity();
        T_.block<3, 3>(0, 0) = R;
        T_.block<3, 1>(0, 3) = t;
        T_(3, 3)             = 1;
    }
    Transform(const Eigen::Matrix4d& T) {
        T_ = T;
    }
    Transform(const cv::Mat& rvec, const cv::Mat& t) {
        T_.setIdentity();
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T_(i, j) = R.at<double>(i, j);
            }
        }
        for (int i = 0; i < 3; i++) {
            T_(i, 3) = t.at<double>(i, 0);
        }
    }

    Eigen::Matrix4d GetMatrix() const {
        return T_;
    }

    void GetPoseCV(cv::Mat& rvec, cv::Mat& t) {
        cv::Mat R;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R.at<double>(i, j) = T_(i, j);
            }
        }
        cv::Rodrigues(R, rvec);
        for (int i = 0; i < 3; i++) {
            t.at<double>(i, 0) = T_(i, 3);
        }
    }

    cv::Mat GetRvecCV() {
        cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R.at<double>(i, j) = T_(i, j);
            }
        }
        cv::Rodrigues(R, rvec);
        return rvec;
    }

    cv::Mat GetTvecCV() {
        cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
        for (int i = 0; i < 3; i++) {
            t.at<double>(i, 0) = T_(i, 3);
        }
        return t;
    }

    Transform Inverse() const {
        Eigen::Matrix4d T        = Eigen::Matrix4d::Identity();
        T.topLeftCorner<3, 3>()  = T_.topLeftCorner<3, 3>().transpose();
        T.topRightCorner<3, 1>() = -T_.topLeftCorner<3, 3>().transpose() * T_.topRightCorner<3, 1>();
        return Transform(T);
    }

    Transform operator*(const Transform& rhs) const {
        Eigen::Matrix4d T_new      = Eigen::Matrix4d::Identity();
        T_new.topLeftCorner(3, 3)  = T_.topLeftCorner(3, 3) * rhs.T_.topLeftCorner(3, 3);
        T_new.topRightCorner(3, 1) = T_.topLeftCorner(3, 3) * rhs.T_.topRightCorner(3, 1) + T_.topRightCorner(3, 1);

        return Transform(T_new);
    }

    Eigen::Vector3d TransformPoint(const Eigen::Vector3d& p) {
        return T_.topLeftCorner<3, 3>() * p + T_.topRightCorner<3, 1>();
    }

private:
    Eigen::Matrix4d T_;
};