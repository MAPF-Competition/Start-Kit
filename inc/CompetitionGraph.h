
#pragma once
#include "BasicGraph.h"


class CompetitionGrid :
public BasicGraph
{
 public:

  bool load_map(string fname);
};
