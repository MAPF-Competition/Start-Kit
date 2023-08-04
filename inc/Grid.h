#pragma once

#include "common.h"

class Grid
{
public:
    Grid(string fname);

    int rows = 0;
    int cols = 0;
    std::vector<int> map;
    string map_name;

};
