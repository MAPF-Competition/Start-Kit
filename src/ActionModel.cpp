#include "ActionModel.h"


std::ostream& operator<<(std::ostream &stream, const Action &action)
{
    if (action == Action::FW) {
        stream << "F";
    } else if (action == Action::CR) {
        stream << "R";
    } else if (action == Action::CCR) {
        stream << "C";
    } else {
        stream << "W";
    }

    return stream;
}

vector<State> ActionModelWithRotate::result_states(const vector<State>& prev, const vector<Action> & action)
{
        vector<State> next(prev.size());
        for (size_t i = 0 ; i < prev.size(); i ++){
            next[i] = result_state(prev[i], action[i]);
        }
        return next;
}
    
State ActionModelWithRotate::result_state(const State & prev, Action action)
{
    int new_location = prev.location;
    int new_orientation = prev.orientation;
    State next = prev;
    if (action == Action::FW)
    {
        if(next.counter.tick())
        {
            new_location = new_location += moves[prev.orientation];
        }
    }
    else if (action == Action::CR)
    {
        if(next.counter.tick())
        {
            new_orientation = (prev.orientation + 1) % 4;
        }
  
    }
    else if (action == Action::CCR)
    {
        if(next.counter.tick())
        {
            new_orientation = (prev.orientation - 1) % 4;
            if (new_orientation == -1)
                new_orientation = 3;
                
        }
    }

    return next;
}

vector<ActionModelWithRotate::RealLocation> ActionModelWithRotate::get_real_locations(const vector<State>& state)
{
    vector<RealLocation> locations;
    locations.reserve(state.size());

    for (const auto& s : state)
    {
        RealLocation loc;
        const int row = s.location / cols;
        const int col = s.location % cols;
        float x = static_cast<float>(col);
        float y = static_cast<float>(row);

        if (s.counter.maxCount > 0 && s.counter.count > 0)
        {
            const float frac = static_cast<float>(s.counter.count) / static_cast<float>(s.counter.maxCount);
            switch (s.orientation)
            {
                case 0: x += frac; break; // east
                case 1: y += frac; break; // south
                case 2: x -= frac; break; // west
                case 3: y -= frac; break; // north
                default: break;
            }
        }

        loc.x = x;
        loc.y = y;
        locations.push_back(loc);
    }
    return locations;
}

bool ActionModelWithRotate::is_valid(const vector<State>& prev, const vector<Action> & actions, int timestep)
{
    // clear previous errors
    errors.clear();
    // Track which agents must wait due to invalid actions.
    _wait_agents.assign(prev.size(), 0);
    const int time = prev.empty() ? (timestep + 1) : (prev[0].timestep + 1);
    if (prev.size() != actions.size())
    {
        errors.push_back(make_tuple("incorrect vector size",-1,-1,time));
        return false;
    }
    /* The agents need to be moved to their actual locations by the counter/maxCounter then do the collision checking physically. The agents are treated as sqaures so a bounding box collision checking is needed.
     We need a buffer to store whether the two agents have already been checked.
     For each agent, based on its state, we will get its grid from next, and get its real location from real_loc. 
     We firstly need to build a dictionary from grid location to agent id. Then for each agent, we check its neighboring grids {{r, c}, {r-1, c}, {r+1, c}, {r, c-1}, {r, c+1}, {r-1, c-1}, {r-1, c+1}, {r+1, c-1}, {r+1, c+1}} 
    (including its own grid) 
     to see if there is any other agent in those grids. If yes, we do a bounding box collision checking based on their real locations. Moreover, {{r-2, c}, {r+2, c}, {r, c-2}, {r, c+2}} also need to be checked 
     if the agents are moving in the opposite direction in which case they can end up in the same grid.
    */

    vector<State> next = result_states(prev, actions);
    vector<RealLocation> real_loc = get_real_locations(next);
    bool valid = true;
    unordered_map<int, vector<int>> grid_agents;
    grid_agents.reserve(next.size() * 2);

    for (int i = 0; i < static_cast<int>(next.size()); i++)
    {
        grid_agents[next[i].location].push_back(i);
    }

    unordered_set<unsigned long long> checked_pairs;
    checked_pairs.reserve(next.size() * 4);

    auto overlaps = [](const RealLocation& a, const RealLocation& b) -> bool
    {
        return (a.x < b.x + 1.0f && a.x + 1.0f > b.x && a.y < b.y + 1.0f && a.y + 1.0f > b.y);
    };

    auto pair_key = [](int a, int b) -> unsigned long long
    {
        int lo = a < b ? a : b;
        int hi = a < b ? b : a;
        return (static_cast<unsigned long long>(static_cast<unsigned int>(lo)) << 32) |
               static_cast<unsigned long long>(static_cast<unsigned int>(hi));
    };

    auto check_pair = [&](int a, int b) -> bool
    {
        unsigned long long key = pair_key(a, b);
        if (!checked_pairs.insert(key).second)
            return false;
        if (overlaps(real_loc[a], real_loc[b]))
        {
            errors.push_back(make_tuple("collision", a, b, time));
            if (logger != nullptr)
            {
                logger->log_warning("Agent collision between " + std::to_string(a) + " and " + std::to_string(b), time);
            }
            return true;
        }
        return false;
    };

    static const int kDr1[9] = {0, -1, 1, 0, 0, -1, -1, 1, 1};
    static const int kDc1[9] = {0, 0, 0, -1, 1, -1, 1, -1, 1};
    static const int kDr2[4] = {-2, 2, 0, 0};
    static const int kDc2[4] = {0, 0, -2, 2};

    for (int i = 0; i < static_cast<int>(next.size()); i++)
    {
        const int r = next[i].location / cols;
        const int c = next[i].location % cols;
        const bool moving_i = (next[i].counter.maxCount > 0 && next[i].counter.count > 0);
        const int ori_i = next[i].orientation;

        for (int k = 0; k < 9; k++)
        {
            const int nr = r + kDr1[k];
            const int nc = c + kDc1[k];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                continue;
            const int nloc = nr * cols + nc;
            auto it = grid_agents.find(nloc);
            if (it == grid_agents.end())
                continue;
            for (int j : it->second)
            {
                if (j == i)
                    continue;
                if (check_pair(i, j))
                    valid = false;
            }
        }

        if (!moving_i)
            continue;
        for (int k = 0; k < 4; k++)
        {
            const int nr = r + kDr2[k];
            const int nc = c + kDc2[k];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                continue;
            const int nloc = nr * cols + nc;
            auto it = grid_agents.find(nloc);
            if (it == grid_agents.end())
                continue;
            for (int j : it->second)
            {
                if (j == i)
                    continue;
                const bool moving_j = (next[j].counter.maxCount > 0 && next[j].counter.count > 0);
                if (!moving_j)
                    continue;
                const int ori_j = next[j].orientation;
                bool opposite = false;
                if (kDr2[k] == 0 && kDc2[k] == 2)
                    opposite = (ori_i == 0 && ori_j == 2);
                else if (kDr2[k] == 0 && kDc2[k] == -2)
                    opposite = (ori_i == 2 && ori_j == 0);
                else if (kDr2[k] == 2 && kDc2[k] == 0)
                    opposite = (ori_i == 1 && ori_j == 3);
                else if (kDr2[k] == -2 && kDc2[k] == 0)
                    opposite = (ori_i == 3 && ori_j == 1);
                if (!opposite)
                    continue;
                if (check_pair(i, j))
                    valid = false;
            }
        }
    }

    if (!errors.empty())
    {
        // Seed wait agents with those explicitly involved in errors.
        for (const auto& error : errors)
        {
            const int a = std::get<1>(error);
            const int b = std::get<2>(error);
            if (a >= 0 && a < static_cast<int>(_wait_agents.size()))
                _wait_agents[a] = 1;
            if (b >= 0 && b < static_cast<int>(_wait_agents.size()))
                _wait_agents[b] = 1;
        }
        // Build dependencies using physical overlaps instead of target grid occupancy.
        vector<vector<int>> blocked_by(prev.size());
        blocked_by.reserve(prev.size());
        for (int i = 0; i < static_cast<int>(prev.size()); i++)
        {
            const int r = next[i].location / cols;
            const int c = next[i].location % cols;
            for (int k = 0; k < 9; k++)
            {
                const int nr = r + kDr1[k];
                const int nc = c + kDc1[k];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                    continue;
                const int nloc = nr * cols + nc;
                auto it = grid_agents.find(nloc);
                if (it == grid_agents.end())
                    continue;
                for (int j : it->second)
                {
                    if (j == i)
                        continue;
                    if (actions[j] != Action::FW)
                        continue;
                    const bool moving_j = (next[j].counter.maxCount > 0 && next[j].counter.count > 0);
                    if (!moving_j)
                        continue;
                    if (overlaps(real_loc[i], real_loc[j]))
                        blocked_by[i].push_back(j);
                }
            }
        }
        // Propagate waits along dependency chains.
        vector<int> queue;
        queue.reserve(prev.size());
        for (int k = 0; k < static_cast<int>(prev.size()); k++)
        {
            if (_wait_agents[k])
                queue.push_back(k);
        }
        size_t qidx = 0;
        while (qidx < queue.size())
        {
            const int blocked = queue[qidx++];
            for (int mover : blocked_by[blocked])
            {
                if (!_wait_agents[mover])
                {
                    _wait_agents[mover] = 1;
                    queue.push_back(mover);
                }
            }
        }
    }

    return valid;
}
