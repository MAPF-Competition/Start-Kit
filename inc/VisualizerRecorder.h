#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "Grid.h"
#include "States.h"
#include "ActionModel.h"

class VisualizerRecorder
{
public:
    VisualizerRecorder() = default;
    ~VisualizerRecorder() { close(); }

    bool open(const std::string& output_file,
              const Grid& map,
              int team_size,
              float agent_size,
              int tick_ms,
              int simulation_time,
              int tick_stride);

    void record_tick(int timestep,
                     const std::vector<State>& states,
                     const std::vector<Action>& actions);

    void close();

    bool enabled() const { return enabled_; }

private:
    std::ofstream out_;
    bool enabled_ = false;
    int tick_stride_ = 1;
    int cols_ = 0;
    std::vector<int> rotation_dir_;
};
