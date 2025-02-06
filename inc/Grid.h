/**
 * @file Grid.h
 * @brief header file for Grid.cpp.
 * 
 * @authors Zhe Chen Han Zhang, Yue Zhang
 * 
 * @note All authors contributed equally to this work.
 */

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
