#pragma once

#include "ActionModel.h"
struct Plan{
    //To be changed by the participants as needed
    std::vector<std::vector<Action>> actions;
    std::vector<std::vector<Action>> convert_to_actions() const {
        return actions;
    }
};