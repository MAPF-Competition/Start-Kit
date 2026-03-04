#include "VisualizerRecorder.h"

#include <algorithm>

#include "nlohmann/json.hpp"

using json = nlohmann::ordered_json;

bool VisualizerRecorder::open(const std::string& output_file,
                              const Grid& map,
                              int team_size,
                              float agent_size,
                              int tick_ms,
                              int simulation_time,
                              int tick_stride)
{
    close();

    out_.open(output_file, std::ios::out | std::ios::trunc);
    if (!out_.is_open())
    {
        return false;
    }

    tick_stride_ = std::max(1, tick_stride);
    cols_ = map.cols;
    rotation_dir_.assign(team_size, 0);

    json meta;
    meta["type"] = "meta";
    meta["rows"] = map.rows;
    meta["cols"] = map.cols;
    meta["teamSize"] = team_size;
    meta["agentSize"] = agent_size;
    meta["tickMs"] = tick_ms;
    meta["simulationTime"] = simulation_time;
    meta["tickStride"] = tick_stride_;

    json obstacles = json::array();
    for (int i = 0; i < static_cast<int>(map.map.size()); i++)
    {
        if (map.map[i] == 1)
        {
            obstacles.push_back(i);
        }
    }
    meta["obstacles"] = obstacles;

    out_ << meta.dump() << '\n';
    out_.flush();
    enabled_ = true;
    return true;
}

void VisualizerRecorder::record_tick(int timestep,
                                     const std::vector<State>& states,
                                     const std::vector<Action>& actions)
{
    if (!enabled_ || !out_.is_open())
    {
        return;
    }
    if (timestep % tick_stride_ != 0)
    {
        return;
    }

    json tick;
    tick["type"] = "tick";
    tick["t"] = timestep;

    json loc = json::array();
    json x = json::array();
    json y = json::array();
    json ori = json::array();
    json move_type = json::array();
    json counter = json::array();
    json counter_max = json::array();
    json action = json::array();
    json rotation_dir = json::array();
    json delay = json::array();

    for (int i = 0; i < static_cast<int>(states.size()); i++)
    {
        const auto& s = states[i];
        const int row = s.location / cols_;
        const int col = s.location % cols_;
        float rx = static_cast<float>(col);
        float ry = static_cast<float>(row);
        if (s.moveType == State::Transition && s.counter.maxCount > 0 && s.counter.count > 0)
        {
            const float frac = static_cast<float>(s.counter.count) / static_cast<float>(s.counter.maxCount);
            switch (s.orientation)
            {
                case 0: rx += frac; break; // east
                case 1: ry += frac; break; // south
                case 2: rx -= frac; break; // west
                case 3: ry -= frac; break; // north
                default: break;
            }
        }

        loc.push_back(s.location);
        x.push_back(rx);
        y.push_back(ry);
        ori.push_back(s.orientation);
        move_type.push_back(static_cast<int>(s.moveType));
        counter.push_back(s.counter.count);
        counter_max.push_back(s.counter.maxCount);
        int act = static_cast<int>(Action::W);
        if (i < static_cast<int>(actions.size()))
        {
            act = static_cast<int>(actions[i]);
        }
        action.push_back(act);

        if (s.moveType == State::Rotation)
        {
            if (act == static_cast<int>(Action::CR))
            {
                rotation_dir_[i] = 1;
            }
            else if (act == static_cast<int>(Action::CCR))
            {
                rotation_dir_[i] = -1;
            }
        }
        else
        {
            rotation_dir_[i] = 0;
        }
        rotation_dir.push_back(rotation_dir_[i]);
        delay.push_back(s.delay.inDelay ? 1 : 0);
    }

    tick["loc"] = loc;
    tick["x"] = x;
    tick["y"] = y;
    tick["ori"] = ori;
    tick["mv"] = move_type;
    tick["cnt"] = counter;
    tick["maxCnt"] = counter_max;
    tick["act"] = action;
    tick["rdir"] = rotation_dir;
    tick["delay"] = delay;

    out_ << tick.dump() << '\n';
}

void VisualizerRecorder::close()
{
    if (out_.is_open())
    {
        out_.flush();
        out_.close();
    }
    enabled_ = false;
}
