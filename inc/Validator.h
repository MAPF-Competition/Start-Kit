#pragma once
#include <string>
#include "Grid.h"
#include "States.h"

class Validator{
protected:
  std::string msg;
public:
  std::string error_msg(){
    return msg;
  }
  virtual bool is_valid(vector<State>& prev, vector<State> & next)=0;

  virtual ~Validator(){};
};


class ValidatorRotate : public Validator{
protected:
  const Grid& grid;
  int rows;
  int cols;

  int moves[4];

public:
  ValidatorRotate(Grid & grid): grid(grid), rows(grid.rows), cols(grid.cols){
    moves[0] = 1;
    moves[1] = -cols;
    moves[2] = -1;
    moves[3] = cols;

  };

  virtual bool is_valid(vector<State>& prev, vector<State> & next) override;

};
