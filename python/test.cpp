#include"pyMAPFPlanner.hpp"

#include <pybind11/embed.h>


int main(){
    pybind11::scoped_interpreter guard{};
    auto planner=new pyMAPFPlanner();



    return 0;
}