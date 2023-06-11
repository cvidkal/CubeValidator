#include "Qualcomm_loader.h"
#include "apriltags/Tag36h11.h"
#include "apriltags/TagDetector.h"

int main() {
    Qualcomm_loader loader;
    VRBody body;
    loader.Load("../tests/device_calibration.xml", body);

    std::cout << body.DebugString() << std::endl;

    cv::Mat bottoms = cv::imread("../tests/bottoms.pgm", CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat tops    = cv::imread("../tests/tops.pgm", CV_LOAD_IMAGE_GRAYSCALE);

    cv::Mat left_bottom  = bottoms(cv::Rect(0, 0, 640, 480));
    cv::Mat right_bottom = bottoms(cv::Rect(640, 0, 640, 480));
    cv::Mat left_top     = tops(cv::Rect(0, 0, 640, 480));
    cv::Mat right_top    = tops(cv::Rect(640, 0, 640, 480));

    AprilTags::TagDetector detector(AprilTags::tagCodes36h11);
    body.SetAprilTagDetector(&detector);

    body.SetImage(left_bottom, 0);
    body.SetImage(right_bottom, 1);
    body.SetImage(left_top, 2);
    body.SetImage(right_top, 3);

    CubeBoard board;
    board.board_config.tag_size    = 0.0775;
    board.board_config.tag_spacing = 0.2;
    board.board_config.tag_rows    = 12;
    board.board_config.tag_cols    = 12;

    board.board_info[0].board_id = 0;
    board.board_info[0].T_wc     = Transform(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));
    board.board_info[1].board_id = 1;
    board.board_info[1].T_wc     = Transform(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));
    board.board_info[2].board_id = 2;
    board.board_info[2].T_wc     = Transform(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));
    board.valid_board_count      = 1;


    body.SetCubeBoard(board);

    body.ValidateExtrinsic();

    return 0;
}