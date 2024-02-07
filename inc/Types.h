#pragma once

#define MAX_TIMESTEP INT_MAX/2

#include <vector>
#include <iostream>
enum DONE{
    NOT_DONE,
    DONE
};

struct Int4{
	int d[4];
};

struct DCR{
    int loc;
    int state;
};

typedef std::vector<int> Traj;

typedef std::vector<int> HeuristicTable;



