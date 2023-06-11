#pragma once
#include "Transform.h"
#include "opencv2/opencv.hpp"

struct BoardConfig {
    float tag_size;
    float tag_spacing;
    int tag_rows;
    int tag_cols;
};
struct BoardInfo {
    int board_id;
    Transform T_wc;
};

struct CubeBoard {
    BoardConfig board_config;
    BoardInfo board_info[3];
    int valid_board_count = 0;
    void WriteAllPoints(const std::string& filename) {
        std::ofstream fout(filename);
        for (int i = 0; i < valid_board_count; i++) {
            for (int j = 0; j < board_config.tag_rows; j++) {
                for (int k = 0; k < board_config.tag_cols; k++) {
                    Eigen::Vector3d p[4];
                    p[0].x() = k * (board_config.tag_size * (1 + board_config.tag_spacing));
                    p[0].y() = j * (board_config.tag_size * (1 + board_config.tag_spacing));
                    p[0].z() = 0;
                    p[1].x() = p[0].x() + board_config.tag_size;
                    p[1].y() = p[0].y();
                    p[1].z() = 0;
                    p[2].x() = p[0].x() + board_config.tag_size;
                    p[2].y() = p[0].y() + board_config.tag_size;
                    p[2].z() = 0;
                    p[3].x() = p[0].x();
                    p[3].y() = p[0].y() + board_config.tag_size;
                    p[3].z() = 0;

                    for (int l = 0; l < 1; l++) {
                        Eigen::Vector3d p_w = board_info[i].T_wc.TransformPoint(p[l]);
                        fout << p_w.x() << " " << p_w.y() << " " << p_w.z() << " " << std::endl;
                    }
                }
            }
        }
        fout.close();
    }

    void LoadBoardConfig(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        fs["tag_size"] >> board_config.tag_size;
        fs["tag_spacing"] >> board_config.tag_spacing;
        fs["tag_rows"] >> board_config.tag_rows;
        fs["tag_cols"] >> board_config.tag_cols;
        fs["valid_board_count"] >> valid_board_count;
        cv::Mat rvec0, tvec0, rvec1, tvec1, rvec2, tvec2;
        rvec0 = cv::Mat_<double>(3, 1);
        tvec0 = cv::Mat_<double>(3, 1);
        rvec1 = cv::Mat_<double>(3, 1);
        tvec1 = cv::Mat_<double>(3, 1);
        rvec2 = cv::Mat_<double>(3, 1);
        tvec2 = cv::Mat_<double>(3, 1);
        fs["rvec0"] >> rvec0;
        fs["tvec0"] >> tvec0;
        fs["rvec1"] >> rvec1;
        fs["tvec1"] >> tvec1;
        fs["rvec2"] >> rvec2;
        fs["tvec2"] >> tvec2;
        board_info[0].board_id = 0;
        board_info[1].board_id = 1;
        board_info[2].board_id = 2;
        board_info[0].T_wc     = Transform(rvec0, tvec0);
        board_info[1].T_wc     = Transform(rvec1, tvec1);
        board_info[2].T_wc     = Transform(rvec2, tvec2);

        fs.release();
    }

    void DumpBoardConfig(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        fs << "tag_size" << board_config.tag_size;
        fs << "tag_spacing" << board_config.tag_spacing;
        fs << "tag_rows" << board_config.tag_rows;
        fs << "tag_cols" << board_config.tag_cols;
        fs << "valid_board_count" << valid_board_count;
        fs << "rvec0" << board_info[0].T_wc.GetRvecCV();
        fs << "tvec0" << board_info[0].T_wc.GetTvecCV();
        fs << "rvec1" << board_info[1].T_wc.GetRvecCV();
        fs << "tvec1" << board_info[1].T_wc.GetTvecCV();
        fs << "rvec2" << board_info[2].T_wc.GetRvecCV();
        fs << "tvec2" << board_info[2].T_wc.GetTvecCV();
        fs.release();
    }
};
