#pragma once
#include "Transform.h"

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
};
