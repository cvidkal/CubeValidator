#pragma once
#include <unordered_map>
#include <vector>

#include "VRCamera.h"

class VRBody {
public:
    void AddCamera(const VRCamera& camera, int camera_id) {
        cameras_[camera_id] = camera;
    }

    void SetImage(const cv::Mat& image, int camera_id) {
        cameras_[camera_id].SetImage(image);
    }

    void SetCubeBoard(const CubeBoard& cube_board) {
        for (auto& camera : cameras_) {
            camera.second.SetBoardConfig(cube_board);
        }
    }

    void SetAprilTagDetector(AprilTags::TagDetector* detector) {
        for (auto& camera : cameras_) {
            camera.second.SetAprilTagDetector(detector);
        }
    }

    void SetOutputDir(const std::string& output_dir) {
        output_dir_ = output_dir + "/";
        for (auto& camera : cameras_) {
            camera.second.SetOutputDir(output_dir);
        }
    }

    void ValidateExtrinsic() {
        FILE* fp = fopen((output_dir_+"[user]result.txt").c_str(), "w");
        printf("Intrinsic validation:\n");
        fprintf(fp, "Intrinsic validation:\n");

        for (auto& camera_pair : cameras_) {
            auto& camera = camera_pair.second;
            camera.UndistortImage();
            camera.DetectTags();
            float error = camera.SolvePnP();
            printf("\t %s reprojection error: %f\n", camera.GetName().c_str(), error);
            fprintf(fp, "\t %s reprojection error: %f\n", camera.GetName().c_str(), error);
            camera.DetectTagsFromReprojection();
        }

        printf("Extrinsic validation:\n");
        fprintf(fp, "Extrinsic validation:\n");

        for (auto& left : cameras_) {
            for (auto& right : cameras_) {
                if (left.first < right.first) {
                    float error = left.second.ValidateExtrinsic(right.second);
                    printf("\t %s - %s reprojection error: %f\n", left.second.GetName().c_str(), right.second.GetName().c_str(), error);
                    fprintf(fp, "\t %s - %s reprojection error: %f\n", left.second.GetName().c_str(), right.second.GetName().c_str(), error);
                }
            }
        }
    }

    std::string DebugString() {
        std::stringstream ss;
        for (auto& camera : cameras_) {
            ss << camera.second.DebugString() << std::endl;
        }

        return ss.str();
    }

private:
    std::unordered_map<int, VRCamera> cameras_;
    std::string output_dir_{"./"};
};