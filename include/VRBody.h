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

    void ValidateExtrinsic() {
        for (auto& camera_pair : cameras_) {
            auto& camera = camera_pair.second;
            camera.UndistortImage();
            camera.DetectTags();
            camera.SolvePnP();
            camera.DetectTagsFromReprojection();
        }

        FILE* fp = fopen("result.txt", "w");
        for (auto& left : cameras_) {
            for (auto& right : cameras_) {
                if (left.first < right.first) {
                    float error = left.second.ValidateExtrinsic(right.second);
                    printf("%s - %s reprojection error: %f\n", left.second.GetName().c_str(), right.second.GetName().c_str(), error);
                    fprintf(fp, "%s - %s reprojection error: %f\n", left.second.GetName().c_str(), right.second.GetName().c_str(), error);
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
};