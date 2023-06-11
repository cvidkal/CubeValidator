#pragma once
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "VRBody.h"
class Qualcomm_loader {
public:
    std::vector<std::string> split(const std::string& s, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }

    bool Load(std::string xml_path, VRBody& body) {
        boost::property_tree::ptree pt;
        boost::property_tree::read_xml(xml_path, pt);
        boost::property_tree::ptree cameras = pt.get_child("DeviceConfiguration");

        for (auto& camera_pair : cameras) {
            if (camera_pair.first == "Camera") {
                VRCamera vr_camera;
                auto& camera  = camera_pair.second;
                int camera_id = camera.get<int>("<xmlattr>.id");
                if (camera_id > 3) continue;
                std::string cam_name = camera.get<std::string>("<xmlattr>.name");

                // get Calibration child
                boost::property_tree::ptree calibration = camera.get_child("Calibration");
                // get size
                std::string size_str = calibration.get<std::string>("<xmlattr>.size");
                // get principal_point
                std::string principal_point_str = calibration.get<std::string>("<xmlattr>.principal_point");
                auto principal_point            = split(principal_point_str, ' ');
                float principal_point_x         = std::stof(principal_point[0]);
                float principal_point_y         = std::stof(principal_point[1]);
                // get focal_length
                std::string focal_length_str = calibration.get<std::string>("<xmlattr>.focal_length");
                auto focal_length            = split(focal_length_str, ' ');
                float focal_length_x         = std::stof(focal_length[0]);
                float focal_length_y         = std::stof(focal_length[1]);
                // get radial_distortion
                std::string radial_distortion_str = calibration.get<std::string>("<xmlattr>.radial_distortion");
                auto radial_distortion            = split(radial_distortion_str, ' ');
                float radial_distortion_1         = std::stof(radial_distortion[0]);
                float radial_distortion_2         = std::stof(radial_distortion[1]);
                float radial_distortion_3         = std::stof(radial_distortion[2]);
                float radial_distortion_4         = std::stof(radial_distortion[3]);

                // get Rig child
                boost::property_tree::ptree rig = camera.get_child("Rig");
                // get translation
                std::string translation_str = rig.get<std::string>("<xmlattr>.translation");
                auto translation            = split(translation_str, ' ');
                float translation_x         = std::stof(translation[0]);
                float translation_y         = std::stof(translation[1]);
                float translation_z         = std::stof(translation[2]);
                // get rowMajorRotationMat
                std::string rowMajorRotationMat_str = rig.get<std::string>("<xmlattr>.rowMajorRotationMat");
                auto rowMajorRotationMat            = split(rowMajorRotationMat_str, ' ');
                float rowMajorRotationMat_1         = std::stof(rowMajorRotationMat[0]);
                float rowMajorRotationMat_2         = std::stof(rowMajorRotationMat[1]);
                float rowMajorRotationMat_3         = std::stof(rowMajorRotationMat[2]);
                float rowMajorRotationMat_4         = std::stof(rowMajorRotationMat[3]);
                float rowMajorRotationMat_5         = std::stof(rowMajorRotationMat[4]);
                float rowMajorRotationMat_6         = std::stof(rowMajorRotationMat[5]);
                float rowMajorRotationMat_7         = std::stof(rowMajorRotationMat[6]);
                float rowMajorRotationMat_8         = std::stof(rowMajorRotationMat[7]);
                float rowMajorRotationMat_9         = std::stof(rowMajorRotationMat[8]);

                Eigen::Matrix3d K;
                K << focal_length_x, 0, principal_point_x,
                    0, focal_length_y, principal_point_y,
                    0, 0, 1;
                Eigen::Vector4d D;
                D << radial_distortion_1, radial_distortion_2, radial_distortion_3, radial_distortion_4;

                vr_camera.SetCameraIntrinsics(K, D);
                Eigen::Matrix4d T;
                T << rowMajorRotationMat_1, rowMajorRotationMat_2, rowMajorRotationMat_3, translation_x,
                    rowMajorRotationMat_4, rowMajorRotationMat_5, rowMajorRotationMat_6, translation_y,
                    rowMajorRotationMat_7, rowMajorRotationMat_8, rowMajorRotationMat_9, translation_z,
                    0, 0, 0, 1;
                vr_camera.SetCameraExtrinsics(T);
                vr_camera.SetName(cam_name);
                body.AddCamera(vr_camera, camera_id);
            }
        }
        return true;
    }

private:
};