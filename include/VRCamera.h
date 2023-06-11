#pragma once
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "BoardConfig.h"
#include "Transform.h"
#include "apriltags/TagDetector.h"

class VRCamera {
public:
    void SetCameraIntrinsics(const Eigen::Matrix3d& K, const Eigen::Vector4d& D) {
        K_    = K;
        D_    = D;
        K_cv_ = cv::Mat::zeros(3, 3, CV_64F);
        D_cv_ = cv::Mat::zeros(4, 1, CV_64F);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++)
                K_cv_.at<double>(i, j) = K_(i, j);
        }

        for (int i = 0; i < 4; i++)
            D_cv_.at<double>(i, 0) = D_(i, 0);
    }

    void SetName(const std::string& name) {
        name_ = name;
    }

    void SetCameraExtrinsics(const Eigen::Matrix4d& T_cb) {
        T_cb_ = Transform(T_cb);
    }

    void SetImage(const cv::Mat& image) {
        image_ = image;
    }

    void SetOutputDir(const std::string& output_dir) {
        output_dir_ = output_dir + "/";
    }

    void UndistortImage() {
        cv::fisheye::undistortImage(image_, image_undistorted_, K_cv_, D_cv_, K_cv_, cv::Size(image_.cols * 2, image_.rows * 2));
        cv::imwrite(output_dir_ + "[debug]" + name_ + "_undistorted.png", image_undistorted_);
    }

    void SetAprilTagDetector(AprilTags::TagDetector* tag_detector) {
        tag_detector_ = tag_detector;
    }

    cv::Mat GetKCV() {
        return K_cv_;
    }

    cv::Mat GetDCV() {
        return D_cv_;
    }

    void DetectTags() {
        tags_ = tag_detector_->extractTags(image_undistorted_);

        cv::Mat draw_image_undistored = image_undistorted_.clone();
        cv::cvtColor(draw_image_undistored, draw_image_undistored, CV_GRAY2BGR);
        for (int i = 0; i < tags_.size(); i++) {
            for (int j = 0; j < 4; j++) {
                cv::circle(draw_image_undistored, cv::Point(tags_[i].p[j].first, tags_[i].p[j].second), 1, cv::Scalar(0, 0, 255), -1);
            }
        }
        cv::imwrite(output_dir_ + "[debug]" + name_ + "_undistorted_tag_detection.png", draw_image_undistored);

        cv::Mat draw_image_distorted = image_.clone();
        cv::cvtColor(draw_image_distorted, draw_image_distorted, CV_GRAY2BGR);
        for (int i = 0; i < tags_.size(); i++) {
            AprilTags::TagDetection tag = tags_[i];
            std::vector<cv::Point2f> p_un_k;
            for (int j = 0; j < 4; j++) {
                cv::Point2f p;
                p.x = (tag.p[j].first - K_(0, 2)) / K_(0, 0);
                p.y = (tag.p[j].second - K_(1, 2)) / K_(1, 1);
                p_un_k.push_back(p);
            }
            std::vector<cv::Point2f> points_distorted;
            cv::fisheye::distortPoints(p_un_k, points_distorted, K_cv_, D_cv_);
            for (int j = 0; j < 4; j++) {
                cv::circle(draw_image_distorted, points_distorted[j], 1, cv::Scalar(0, 0, 255), -1);
            }

            // cv::cornerSubPix(image_, points_distorted, cv::Size(3, 3), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            for (int j = 0; j < 4; j++) {
                cv::circle(draw_image_distorted, points_distorted[j], 1, cv::Scalar(0, 255, 0), -1);
            }
            for (int j = 0; j < 4; j++) {
                tag.p[j].first  = points_distorted[j].x;
                tag.p[j].second = points_distorted[j].y;
            }
            tag.cxy.first  = (tag.p[0].first + tag.p[1].first + tag.p[2].first + tag.p[3].first) / 4;
            tag.cxy.second = (tag.p[0].second + tag.p[1].second + tag.p[2].second + tag.p[3].second) / 4;
            tags_distorted_.push_back(tag);
        }
        cv::imwrite(output_dir_ + "[debug]" + name_ + "_distorted_tag_detection.png", draw_image_distorted);
    }

    float SolvePnP() {
        std::vector<cv::Point3f> obj_pts;
        std::vector<cv::Point2f> img_pts;

        for (int i = 0; i < tags_distorted_.size(); i++) {
            AprilTags::TagDetection tag = tags_distorted_[i];
            int id_local                = tag.id % (cube_board_.board_config.tag_cols * cube_board_.board_config.tag_rows);
            int board_id                = tag.id / (cube_board_.board_config.tag_cols * cube_board_.board_config.tag_rows);
            int row                     = id_local / cube_board_.board_config.tag_cols;
            int col                     = id_local % cube_board_.board_config.tag_cols;
            cv::Point3f p[4];
            p[0].x = col * cube_board_.board_config.tag_size * (1 + cube_board_.board_config.tag_spacing);
            p[0].y = row * cube_board_.board_config.tag_size * (1 + cube_board_.board_config.tag_spacing);
            p[0].z = 0;
            p[1].x = p[0].x + cube_board_.board_config.tag_size;
            p[1].y = p[0].y;
            p[1].z = 0;
            p[2].x = p[0].x + cube_board_.board_config.tag_size;
            p[2].y = p[0].y + cube_board_.board_config.tag_size;
            p[2].z = 0;
            p[3].x = p[0].x;
            p[3].y = p[0].y + cube_board_.board_config.tag_size;
            p[3].z = 0;

            for (int j = 0; j < 4; j++) {
                Eigen::Vector3d p_c;
                p_c << p[j].x,
                    p[j].y, p[j].z;
                Eigen::Vector3d p_w;
                p_w = cube_board_.board_info[board_id].T_wc.TransformPoint(p_c);
                obj_pts.push_back(cv::Point3f(p_w(0), p_w(1), p_w(2)));
                img_pts.push_back(cv::Point2f(tag.p[j].first, tag.p[j].second));
            }
        }

        cv::fisheye::undistortPoints(img_pts, img_pts, K_cv_, D_cv_, K_cv_);
        cv::Mat rvec, t;
        cv::solvePnP(obj_pts, img_pts, K_cv_, cv::Mat(), rvec, t);

        T_cw_ = Transform(rvec, t);

        cv::Mat draw_image_undistored = image_undistorted_.clone();
        cv::cvtColor(draw_image_undistored, draw_image_undistored, CV_GRAY2BGR);
        // compute reprojection error
        double error = 0;
        std::vector<cv::Point2f> reprojected_points;
        cv::projectPoints(obj_pts, rvec, t, K_cv_, cv::Mat(), reprojected_points);
        float max_r_pixel = sqrt(float(image_.rows * image_.rows + image_.cols * image_.cols)) * 0.8f;
        int valid_count   = 0;
        for (int i = 0; i < reprojected_points.size(); i++) {
            float dx = reprojected_points[i].x - img_pts[i].x;
            float dy = reprojected_points[i].y - img_pts[i].y;
            float r  = sqrt(reprojected_points[i].x * reprojected_points[i].x + reprojected_points[i].y * reprojected_points[i].y);
            if (r < max_r_pixel) {
                valid_count++;
                error += sqrt(dx * dx + dy * dy);
            }
            cv::circle(draw_image_undistored, reprojected_points[i], 1, cv::Scalar(255, 0, 0), -1);
            cv::circle(draw_image_undistored, img_pts[i], 1, cv::Scalar(0, 0, 255), -1);
        }
        error /= valid_count;
        // printf("%s Reprojection error: %f\n", name_.c_str(), error);
        cv::imwrite(output_dir_ + "[debug]" + name_ + "_solve_pnp.png", draw_image_undistored);
        return error;
    }

    void DetectTagsFromReprojection() {
        std::vector<cv::Point3f> obj_pts;

        for (int board_id = 0; board_id < cube_board_.valid_board_count; ++board_id) {
            for (int row = 0; row < cube_board_.board_config.tag_rows; ++row) {
                for (int col = 0; col < cube_board_.board_config.tag_cols; ++col) {
                    cv::Point3f p[4];
                    p[0].x = col * cube_board_.board_config.tag_size * (1 + cube_board_.board_config.tag_spacing);
                    p[0].y = row * cube_board_.board_config.tag_size * (1 + cube_board_.board_config.tag_spacing);
                    p[0].z = 0;
                    p[1].x = p[0].x + cube_board_.board_config.tag_size;
                    p[1].y = p[0].y;
                    p[1].z = 0;
                    p[2].x = p[0].x + cube_board_.board_config.tag_size;
                    p[2].y = p[0].y + cube_board_.board_config.tag_size;
                    p[2].z = 0;
                    p[3].x = p[0].x;
                    p[3].y = p[0].y + cube_board_.board_config.tag_size;
                    p[3].z = 0;

                    for (int j = 0; j < 4; j++) {
                        Eigen::Vector3d p_c;
                        p_c << p[j].x,
                            p[j].y, p[j].z;
                        Eigen::Vector3d p_w;
                        p_w = cube_board_.board_info[board_id].T_wc.TransformPoint(p_c);
                        obj_pts.push_back(cv::Point3f(p_w(0), p_w(1), p_w(2)));
                    }
                }
            }
        }

        std::vector<cv::Point2f> img_pts;
        cv::fisheye::projectPoints(obj_pts, img_pts, T_cw_.GetRvecCV(), T_cw_.GetTvecCV(), K_cv_, D_cv_);

        cv::Mat draw_image_distored_from_reprojection = image_.clone();
        cv::cvtColor(draw_image_distored_from_reprojection, draw_image_distored_from_reprojection, CV_GRAY2BGR);
        for (int i = 0; i < img_pts.size(); i++) {
            cv::circle(draw_image_distored_from_reprojection, img_pts[i], 1, cv::Scalar(0, 0, 255), -1);
        }

        // cv::cornerSubPix(image_, img_pts, cv::Size(3, 3), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        for (int i = 0; i < img_pts.size(); i++) {
            cv::circle(draw_image_distored_from_reprojection, img_pts[i], 1, cv::Scalar(0, 255, 0), -1);
        }
        cv::imwrite(output_dir_ + "[debug]" + name_ + "_distorted_tag_detection_from_reprojection.png", draw_image_distored_from_reprojection);

        int image_rows                                         = image_.rows;
        int image_cols                                         = image_.cols;
        cv::Mat draw_image_distored_from_reprojection_selected = image_.clone();
        cv::cvtColor(draw_image_distored_from_reprojection_selected, draw_image_distored_from_reprojection_selected, CV_GRAY2BGR);
        for (int i = 0; i < img_pts.size(); i += 4) {
            AprilTags::TagDetection tag;
            tag.id       = i / 4;
            bool in_view = true;
            for (int j = 0; j < 4; j++) {
                tag.p[j].first  = img_pts[i + j].x;
                tag.p[j].second = img_pts[i + j].y;
                if (tag.p[j].first < 0 || tag.p[j].first >= image_cols || tag.p[j].second < 0 || tag.p[j].second >= image_rows) {
                    in_view = false;
                    break;
                }
            }
            Eigen::Vector3d p_c = T_cw_.TransformPoint(Eigen::Vector3d(obj_pts[i].x, obj_pts[i].y, obj_pts[i].z));
            if (p_c.z() < 0.01) {
                in_view = false;
            }

            if (!in_view)
                continue;
            tag.cxy.first  = (tag.p[0].first + tag.p[1].first + tag.p[2].first + tag.p[3].first) / 4;
            tag.cxy.second = (tag.p[0].second + tag.p[1].second + tag.p[2].second + tag.p[3].second) / 4;
            tags_distorted_from_reprojection_.push_back(tag);
        }

        for (int i = 0; i < tags_distorted_from_reprojection_.size(); ++i) {
            AprilTags::TagDetection tag = tags_distorted_from_reprojection_[i];
            for (int j = 0; j < 4; ++j) {
                cv::circle(draw_image_distored_from_reprojection_selected, cv::Point(tag.p[j].first, tag.p[j].second), 1, cv::Scalar(0, 0, 255), -1);
            }
            cv::putText(draw_image_distored_from_reprojection_selected, std::to_string(tag.id), cv::Point(tag.cxy.first, tag.cxy.second), cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(0, 0, 255), 1);
        }

        cv::imwrite(output_dir_ + "[debug]" + name_ + "_distorted_tag_detection_from_reprojection_selected.png", draw_image_distored_from_reprojection_selected);
    }

    std::vector<AprilTags::TagDetection> GetTags() {
        return tags_;
    }

    std::vector<AprilTags::TagDetection> GetTagsDistorted() {
        return tags_distorted_;
    }

    std::vector<AprilTags::TagDetection> GetTagsDistortedFromReprojection() {
        return tags_distorted_from_reprojection_;
    }

    float ValidateExtrinsic(VRCamera& another_cam) {
        std::vector<cv::Point2f> img_pts_another_cam;
        std::vector<std::pair<AprilTags::TagDetection, AprilTags::TagDetection>> same_tag_detections;
        std::vector<cv::Point3f> obj_pts;
        for (int i = 0; i < tags_distorted_from_reprojection_.size(); ++i) {
            for (int j = 0; j < another_cam.tags_distorted_from_reprojection_.size(); ++j) {
                if (tags_distorted_from_reprojection_[i].id == another_cam.tags_distorted_from_reprojection_[j].id) {
                    same_tag_detections.push_back(std::make_pair(tags_distorted_from_reprojection_[i], another_cam.tags_distorted_from_reprojection_[j]));
                    for (int k = 0; k < 4; ++k) {
                        img_pts_another_cam.push_back(cv::Point2f(another_cam.tags_distorted_from_reprojection_[j].p[k].first, another_cam.tags_distorted_from_reprojection_[j].p[k].second));
                    }

                    cv::Point3f p[4];
                    int board_id = tags_distorted_from_reprojection_[i].id / (cube_board_.board_config.tag_cols * cube_board_.board_config.tag_rows);
                    int id_local = tags_distorted_from_reprojection_[i].id % (cube_board_.board_config.tag_cols * cube_board_.board_config.tag_rows);
                    int row      = id_local / cube_board_.board_config.tag_cols;
                    int col      = id_local % cube_board_.board_config.tag_cols;
                    p[0].x       = col * cube_board_.board_config.tag_size * (1 + cube_board_.board_config.tag_spacing);
                    p[0].y       = row * cube_board_.board_config.tag_size * (1 + cube_board_.board_config.tag_spacing);
                    p[0].z       = 0;
                    p[1].x       = p[0].x + cube_board_.board_config.tag_size;
                    p[1].y       = p[0].y;
                    p[1].z       = 0;
                    p[2].x       = p[0].x + cube_board_.board_config.tag_size;
                    p[2].y       = p[0].y + cube_board_.board_config.tag_size;
                    p[2].z       = 0;
                    p[3].x       = p[0].x;
                    p[3].y       = p[0].y + cube_board_.board_config.tag_size;
                    p[3].z       = 0;

                    for (int k = 0; k < 4; k++) {
                        Eigen::Vector3d p_c;
                        p_c << p[k].x,
                            p[k].y, p[k].z;
                        Eigen::Vector3d p_w;
                        p_w = cube_board_.board_info[board_id].T_wc.TransformPoint(p_c);
                        obj_pts.push_back(cv::Point3f(p_w(0), p_w(1), p_w(2)));
                    }

                    break;
                }
            }
        }

        Transform Tc1c0 = another_cam.T_cb_ * T_cb_.Inverse();
        Transform Tc1w_ = Tc1c0 * T_cw_;

        std::vector<cv::Point2f> img_pts_another_cam_reprojected;
        cv::fisheye::projectPoints(obj_pts, img_pts_another_cam_reprojected, Tc1w_.GetRvecCV(), Tc1w_.GetTvecCV(), another_cam.K_cv_, another_cam.D_cv_);

        cv::Mat left_image_distored_from_reprojection_selected = image_.clone();
        cv::cvtColor(left_image_distored_from_reprojection_selected, left_image_distored_from_reprojection_selected, CV_GRAY2BGR);
        cv::Mat right_image_distored_from_reprojection_selected = another_cam.image_.clone();
        cv::cvtColor(right_image_distored_from_reprojection_selected, right_image_distored_from_reprojection_selected, CV_GRAY2BGR);

        cv::Mat right_image_result_for_user = another_cam.image_.clone();
        cv::cvtColor(right_image_result_for_user, right_image_result_for_user, CV_GRAY2BGR);
        float error = 0;
        for (int i = 0; i < img_pts_another_cam.size(); ++i) {
            float dx      = img_pts_another_cam[i].x - img_pts_another_cam_reprojected[i].x;
            float dy      = img_pts_another_cam[i].y - img_pts_another_cam_reprojected[i].y;
            float err_now = sqrt(dx * dx + dy * dy);
            error += err_now;
            auto color = err_now < 3 ? cv::Scalar(0, 255, 0) : err_now < 5 ? cv::Scalar(0, 255, 255)
                                                                           : cv::Scalar(0, 0, 255);
            cv::circle(right_image_result_for_user, img_pts_another_cam_reprojected[i], 1, color, -1);
            cv::circle(right_image_distored_from_reprojection_selected, img_pts_another_cam_reprojected[i], 1, cv::Scalar(255, 0, 0), -1);
        }
        error /= img_pts_another_cam.size();

        for (int i = 0; i < same_tag_detections.size(); ++i) {
            AprilTags::TagDetection tag_left  = same_tag_detections[i].first;
            AprilTags::TagDetection tag_right = same_tag_detections[i].second;
            for (int j = 0; j < 4; ++j) {
                cv::circle(left_image_distored_from_reprojection_selected, cv::Point(tag_left.p[j].first, tag_left.p[j].second), 1, cv::Scalar(0, 0, 255), -1);
                cv::circle(right_image_distored_from_reprojection_selected, cv::Point(tag_right.p[j].first, tag_right.p[j].second), 1, cv::Scalar(0, 0, 255), -1);
            }
        }

        cv::Mat merge;
        cv::hconcat(left_image_distored_from_reprojection_selected, right_image_distored_from_reprojection_selected, merge);
        cv::imwrite(output_dir_ + "[debug]" + name_ + "- " + another_cam.name_ + "extensive_result.png", merge);

        cv::Mat merge_for_user;
        cv::hconcat(left_image_distored_from_reprojection_selected, right_image_result_for_user, merge_for_user);
        cv::imwrite(output_dir_ + "[user]" + name_ + "- " + another_cam.name_ + "extensive_result.png", merge_for_user);
        return error;
    }

    void SetBoardConfig(const CubeBoard& cube_board) {
        cube_board_ = cube_board;
    }

    std::string DebugString() {
        std::stringstream ss;
        ss << "name: " << name_ << std::endl;
        ss << "K: " << K_ << std::endl;
        ss << "D: " << D_ << std::endl;
        ss << "T_cb: " << T_cb_.GetMatrix() << std::endl;
        return ss.str();
    }

    std::string GetName() {
        return name_;
    }

private:
    Eigen::Matrix3d K_;
    Eigen::Vector4d D_;
    cv::Mat K_cv_;
    cv::Mat D_cv_;

    Transform T_cb_;

    std::string name_;

    cv::Mat image_;
    cv::Mat image_undistorted_;

    AprilTags::TagDetector* tag_detector_;

    std::vector<AprilTags::TagDetection> tags_;
    std::vector<AprilTags::TagDetection> tags_distorted_;
    std::vector<AprilTags::TagDetection> tags_distorted_from_reprojection_;

    Transform T_cw_;

    CubeBoard cube_board_;

    std::string output_dir_{"./"};
};