#include "Qualcomm_loader.h"
#include "apriltags/Tag36h11.h"
#include "apriltags/TagDetector.h"
#include "experimental/filesystem"

int main(int argc, char** argv) {
    // ./test --calib [calibfile] --bottoms [bottoms] --tops [tops] --board [board] --output [output]
    if (argc != 11) {
        std::cout << "Usage: ./test --calib [calibfile] --bottoms [bottoms] --tops [tops] --board [board] --output [output]" << std::endl;
        return -1;
    }
    std::string calibfile;
    std::string bottoms;
    std::string tops;
    std::string board;
    std::string output;

    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--calib") {
            calibfile = argv[i + 1];
        }
        if (std::string(argv[i]) == "--bottoms") {
            bottoms = argv[i + 1];
        }
        if (std::string(argv[i]) == "--tops") {
            tops = argv[i + 1];
        }
        if (std::string(argv[i]) == "--board") {
            board = argv[i + 1];
        }
        if (std::string(argv[i]) == "--output") {
            output = argv[i + 1];
        }
        // check option is valid
        if (i % 2 == 1) {
            if (std::string(argv[i]) != "--calib" && std::string(argv[i]) != "--bottoms" && std::string(argv[i]) != "--tops" && std::string(argv[i]) != "--board" && std::string(argv[i]) != "--output") {
                std::cout << "Invalid option: " << argv[i] << std::endl;
                std::cout << "Usage: ./test --calib [calibfile] --bottoms [bottoms] --tops [tops] --board [board] --output [output]" << std::endl;
                return -1;
            }
        }
    }

    // check input
    if (calibfile.empty() || bottoms.empty() || tops.empty() || board.empty() || output.empty()) {
        std::cout << "Usage: ./test --calib [calibfile] --bottoms [bottoms] --tops [tops] --board [board] --output [output]" << std::endl;
        return -1;
    }

    // check output dir exists
    if (!std::experimental::filesystem::exists(output)) {
        // create output dir
        std::experimental::filesystem::create_directory(output);
    }

    Qualcomm_loader loader;
    VRBody body;
    loader.Load(calibfile, body);

    std::cout << body.DebugString() << std::endl;

    cv::Mat bottoms_image = cv::imread(bottoms, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat tops_images   = cv::imread(tops, CV_LOAD_IMAGE_GRAYSCALE);

    cv::Mat left_bottom  = bottoms_image(cv::Rect(0, 0, 640, 480));
    cv::Mat right_bottom = bottoms_image(cv::Rect(640, 0, 640, 480));
    cv::Mat left_top     = tops_images(cv::Rect(0, 0, 640, 480));
    cv::Mat right_top    = tops_images(cv::Rect(640, 0, 640, 480));

    AprilTags::TagDetector detector(AprilTags::tagCodes36h11);
    body.SetAprilTagDetector(&detector);

    body.SetImage(left_bottom, 0);
    body.SetImage(right_bottom, 1);
    body.SetImage(left_top, 2);
    body.SetImage(right_top, 3);
    body.SetOutputDir(output);

    CubeBoard cube;
    // board.board_config.tag_size    = 0.0775;
    // board.board_config.tag_spacing = 0.2;
    // board.board_config.tag_rows    = 12;
    // board.board_config.tag_cols    = 12;

    // Rotation matrix:
    //  [[ 1.17975374  2.01987608 -1.08220641]]
    // Translation matrix:
    //  [[-0.02579971  0.02596734  2.89474429]]
    // Rotation matrix:
    //  [[1.85205872 1.39438682 1.51370436]]
    // Translation matrix:
    //  [[-0.4929295  -0.47776275  1.4398179 ]]
    //  Rotation matrix:
    //  [[-2.55103548 -0.25908806  0.60865052]]
    // Translation matrix:
    //  [[-0.00352168 -0.03780958  2.89701079]]

    // cv::Mat rvec0 = (cv::Mat_<double>(3, 1) << 1.17975374, 2.01987608, -1.08220641);
    // cv::Mat tvec0 = (cv::Mat_<double>(3, 1) << -0.02579971, 0.02596734, 2.89474429);
    // cv::Mat rvec1 = (cv::Mat_<double>(3, 1) << 1.85205872, 1.39438682, 1.51370436);
    // cv::Mat tvec1 = (cv::Mat_<double>(3, 1) << -0.4929295, -0.47776275, 1.4398179);
    // cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << -2.55103548, -0.25908806, 0.60865052);
    // cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -0.00352168, -0.03780958, 2.89701079);

    // Transform Tcb0(rvec0, tvec0);
    // Transform Tcb1(rvec1, tvec1);
    // Transform Tcb2(rvec2, tvec2);

    // board.board_info[0].board_id = 0;
    // board.board_info[0].T_wc     = Transform::Identity();
    // board.board_info[1].board_id = 1;
    // board.board_info[1].T_wc     = Tcb0.Inverse() * Tcb1;
    // board.board_info[2].board_id = 2;
    // board.board_info[2].T_wc     = Tcb0.Inverse() * Tcb2;
    // board.valid_board_count      = 3;

    // board.DumpBoardConfig("kxr_cube_12*12.ymal");

    cube.LoadBoardConfig(board);

    // board.WriteAllPoints("points.txt");

    body.SetCubeBoard(cube);

    body.ValidateExtrinsic();

    return 0;
}