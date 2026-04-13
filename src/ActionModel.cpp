#include "ActionModel.h"
#include <cmath>

namespace
{
string action_to_string(Action action);

struct BoxMotion
{
    float x0 = 0.0f;
    float y0 = 0.0f;
    float x1 = 0.0f;
    float y1 = 0.0f;
    float size = 0.0f;
};

struct Rect
{
    float min_x = 0.0f;
    float min_y = 0.0f;
    float max_x = 0.0f;
    float max_y = 0.0f;
};

enum ResolveStatus : char
{
    UNSEEN = 0,
    VISITING = 1,
    DONE = 2
};

struct StepResolveContext
{
    const Grid& grid;
    int rows;
    int cols;
    int n;
    const vector<State>& prev_states;
    const vector<Action>& requested_actions;
    const vector<State>& requested_states;
    const vector<State>& wait_states;
    const vector<char>& requested_valid;
    const vector<char>& wait_valid;
    const vector<BoxMotion>& requested_motions;
    const vector<BoxMotion>& wait_motions;
    const unordered_map<int, vector<int>>& current_bins;
    float neighbor_margin;

    vector<char>& status;
    vector<char>& has_decision;
    vector<char>& chose_requested;
    vector<Action>& decided_actions;
    vector<State>& decided_states;
    vector<BoxMotion>& decided_motions;
    vector<string>& requested_fail_reasons;

    vector<int>& seen_agents;
    int& seen_token;
    vector<int>& recursion_stack;
    vector<int>& stack_pos;
    list<std::tuple<std::string, int, int, int>>& errors;
    Logger* logger;
    int time;
};

bool moving_boxes_collide(const BoxMotion& a, const BoxMotion& b);

void set_reason_if_empty(vector<string>& reasons, int agent_id, const string& reason)
{
    if (reasons[agent_id].empty())
        reasons[agent_id] = reason;
}

bool is_occupancy_dependency_edge(int from_agent, int to_agent, const StepResolveContext& ctx)
{
    return ctx.requested_states[from_agent].location == ctx.prev_states[to_agent].location;
}

bool is_directional_physical_dependency_edge(int from_agent, int to_agent, const StepResolveContext& ctx)
{
    if (!ctx.requested_valid[from_agent] || !ctx.wait_valid[to_agent])
        return false;
    return moving_boxes_collide(ctx.requested_motions[from_agent], ctx.wait_motions[to_agent]);
}

string format_cycle_description(int current_agent, int cycle_agent, const StepResolveContext& ctx)
{
    int start = 0;
    if (cycle_agent >= 0 && cycle_agent < static_cast<int>(ctx.stack_pos.size()) && ctx.stack_pos[cycle_agent] >= 0)
        start = ctx.stack_pos[cycle_agent];

    bool occupancy_chain = true;
    for (int idx = start; idx + 1 < static_cast<int>(ctx.recursion_stack.size()); idx++)
    {
        const int from = ctx.recursion_stack[idx];
        const int to = ctx.recursion_stack[idx + 1];
        if (!is_occupancy_dependency_edge(from, to, ctx))
        {
            occupancy_chain = false;
            break;
        }
    }
    if (occupancy_chain && !is_occupancy_dependency_edge(current_agent, cycle_agent, ctx))
        occupancy_chain = false;

    string msg = occupancy_chain ? "occupancy-chain cycle" : "directed physical-dependency cycle";
    msg += ": ";
    for (int idx = start; idx < static_cast<int>(ctx.recursion_stack.size()); idx++)
    {
        const int a = ctx.recursion_stack[idx];
        msg += "a" + std::to_string(a) +
               "(loc=" + std::to_string(ctx.prev_states[a].location) +
               ", act=" + action_to_string(ctx.requested_actions[a]) +
               ", next=" + std::to_string(ctx.requested_states[a].location) + ") -> ";
    }
    msg += "a" + std::to_string(cycle_agent) +
           "(loc=" + std::to_string(ctx.prev_states[cycle_agent].location) +
           ", act=" + action_to_string(ctx.requested_actions[cycle_agent]) +
           ", next=" + std::to_string(ctx.requested_states[cycle_agent].location) + ")";
    return msg;
}

void record_actionmodel_message(const StepResolveContext& ctx, const string& msg, int agent1, int agent2)
{
    ctx.errors.push_back(std::make_tuple(msg, agent1, agent2, ctx.time));
}

void log_actionmodel_message(const StepResolveContext& ctx, const string& msg, int agent1 = -1, int agent2 = -1)
{
    record_actionmodel_message(ctx, msg, agent1, agent2);
    if (ctx.logger != nullptr)
        ctx.logger->log_warning(msg, ctx.time);
    else
        cout << "[ActionModel] t=" << ctx.time << " " << msg << endl;
}

Rect make_swept_rect(const BoxMotion& m)
{
    Rect r;
    r.min_x = std::min(m.x0, m.x1);
    r.min_y = std::min(m.y0, m.y1);
    r.max_x = std::max(m.x0, m.x1) + m.size;
    r.max_y = std::max(m.y0, m.y1) + m.size;
    return r;
}

bool rects_overlap(const Rect& a, const Rect& b)
{
    return (a.min_x < b.max_x && a.max_x > b.min_x && a.min_y < b.max_y && a.max_y > b.min_y);
}

bool motion_swept_bounds_valid(const BoxMotion& m, int rows, int cols)
{
    const Rect r = make_swept_rect(m);
    return r.min_x >= 0.0f && r.min_y >= 0.0f &&
           r.max_x <= static_cast<float>(cols) &&
           r.max_y <= static_cast<float>(rows);
}

bool intersect_open_linear_interval(float d0, float dv, float lower, float upper, float& out_t0, float& out_t1)
{
    const float eps = 1e-6f;
    if (dv > -eps && dv < eps)
    {
        if (d0 > lower + eps && d0 < upper - eps)
        {
            out_t0 = 0.0f;
            out_t1 = 1.0f;
            return true;
        }
        return false;
    }

    float t0 = (lower - d0) / dv;
    float t1 = (upper - d0) / dv;
    if (t0 > t1)
        std::swap(t0, t1);

    const float teps = eps / std::fabs(dv);
    t0 += teps;
    t1 -= teps;

    out_t0 = std::max(0.0f, t0);
    out_t1 = std::min(1.0f, t1);
    return out_t0 <= out_t1;
}

bool moving_boxes_collide(const BoxMotion& a, const BoxMotion& b)
{
    const Rect ra = make_swept_rect(a);
    const Rect rb = make_swept_rect(b);
    if (!rects_overlap(ra, rb))
        return false;

    const float d0x = a.x0 - b.x0;
    const float d0y = a.y0 - b.y0;
    const float dvx = (a.x1 - a.x0) - (b.x1 - b.x0);
    const float dvy = (a.y1 - a.y0) - (b.y1 - b.y0);

    float tx0 = 0.0f, tx1 = 1.0f;
    float ty0 = 0.0f, ty1 = 1.0f;
    if (!intersect_open_linear_interval(d0x, dvx, -a.size, b.size, tx0, tx1))
        return false;
    if (!intersect_open_linear_interval(d0y, dvy, -a.size, b.size, ty0, ty1))
        return false;

    return std::max(tx0, ty0) <= std::min(tx1, ty1);
}

bool motion_hits_hard_obstacle(const BoxMotion& motion, const Grid& grid, int rows, int cols)
{
    const Rect sweep = make_swept_rect(motion);
    int c0 = std::max(0, static_cast<int>(std::floor(sweep.min_x)));
    int r0 = std::max(0, static_cast<int>(std::floor(sweep.min_y)));
    int c1 = std::min(cols - 1, static_cast<int>(std::ceil(sweep.max_x) - 1));
    int r1 = std::min(rows - 1, static_cast<int>(std::ceil(sweep.max_y) - 1));

    for (int r = r0; r <= r1; r++)
    {
        for (int c = c0; c <= c1; c++)
        {
            const int loc = r * cols + c;
            if (grid.map[loc] != 1)
                continue;

            BoxMotion obstacle;
            obstacle.x0 = obstacle.x1 = static_cast<float>(c);
            obstacle.y0 = obstacle.y1 = static_cast<float>(r);
            obstacle.size = 1.0f;
            if (moving_boxes_collide(motion, obstacle))
                return true;
        }
    }
    return false;
}

string action_to_string(Action action)
{
    switch (action)
    {
        case Action::FW: return "F";
        case Action::CR: return "R";
        case Action::CCR: return "C";
        case Action::W: return "W";
        case Action::NA: return "NA";
        default: return "?";
    }
}

void add_current_box_to_bins(const BoxMotion& box, int agent_id, int rows, int cols, unordered_map<int, vector<int>>& bins)
{
    Rect r;
    r.min_x = box.x0;
    r.min_y = box.y0;
    r.max_x = box.x0 + box.size;
    r.max_y = box.y0 + box.size;

    const int c0 = std::max(0, static_cast<int>(std::floor(r.min_x)));
    const int r0 = std::max(0, static_cast<int>(std::floor(r.min_y)));
    const int c1 = std::min(cols - 1, static_cast<int>(std::ceil(r.max_x) - 1));
    const int r1 = std::min(rows - 1, static_cast<int>(std::ceil(r.max_y) - 1));
    for (int rr = r0; rr <= r1; rr++)
    {
        for (int cc = c0; cc <= c1; cc++)
        {
            bins[rr * cols + cc].push_back(agent_id);
        }
    }
}

void collect_neighbor_agents(int agent_id, const BoxMotion& motion, StepResolveContext& ctx, vector<int>& out)
{
    out.clear();
    Rect q = make_swept_rect(motion);
    q.min_x -= ctx.neighbor_margin;
    q.min_y -= ctx.neighbor_margin;
    q.max_x += ctx.neighbor_margin;
    q.max_y += ctx.neighbor_margin;

    const int c0 = std::max(0, static_cast<int>(std::floor(q.min_x)));
    const int r0 = std::max(0, static_cast<int>(std::floor(q.min_y)));
    const int c1 = std::min(ctx.cols - 1, static_cast<int>(std::ceil(q.max_x) - 1));
    const int r1 = std::min(ctx.rows - 1, static_cast<int>(std::ceil(q.max_y) - 1));

    ++ctx.seen_token;
    for (int r = r0; r <= r1; r++)
    {
        for (int c = c0; c <= c1; c++)
        {
            const int key = r * ctx.cols + c;
            auto it = ctx.current_bins.find(key);
            if (it == ctx.current_bins.end())
                continue;
            for (int j : it->second)
            {
                if (j == agent_id)
                    continue;
                if (ctx.seen_agents[j] == ctx.seen_token)
                    continue;
                ctx.seen_agents[j] = ctx.seen_token;
                out.push_back(j);
            }
        }
    }
}

bool resolve_agent_recursive(int agent_id, StepResolveContext& ctx);

bool try_commit_candidate(int agent_id,
                          bool use_requested,
                          StepResolveContext& ctx,
                          vector<int>& neighbors)
{
    const vector<char>& validity = use_requested ? ctx.requested_valid : ctx.wait_valid;
    if (!validity[agent_id])
    {
        if (use_requested)
            set_reason_if_empty(ctx.requested_fail_reasons, agent_id, "requested motion is invalid (boundary or hard obstacle)");
        return false;
    }

    const BoxMotion& motion_i = use_requested ? ctx.requested_motions[agent_id] : ctx.wait_motions[agent_id];
    collect_neighbor_agents(agent_id, motion_i, ctx, neighbors);

    for (int j : neighbors)
    {
        if (ctx.has_decision[j])
        {
            if (moving_boxes_collide(motion_i, ctx.decided_motions[j]))
            {
                if (use_requested)
                {
                    set_reason_if_empty(
                        ctx.requested_fail_reasons,
                        agent_id,
                        "requested motion conflicts with resolved agent " + std::to_string(j));
                }
                return false;
            }
            continue;
        }

        // Directional dependency: i depends on j only if j staying (wait motion) blocks i's requested motion.
        // This avoids turning linear "j is leaving, i follows" chains into fake mutual cycles.
        if (!(use_requested && ctx.wait_valid[j] && moving_boxes_collide(motion_i, ctx.wait_motions[j])))
            continue;

        if (ctx.status[j] == VISITING)
        {
            if (use_requested)
            {
                const string cycle_desc = format_cycle_description(agent_id, j, ctx);
                set_reason_if_empty(
                    ctx.requested_fail_reasons,
                    agent_id,
                    cycle_desc);
                log_actionmodel_message(ctx, cycle_desc, agent_id, j);
            }
            return false;
        }

        const bool dep_moved = resolve_agent_recursive(j, ctx);
        if (!ctx.has_decision[j])
        {
            if (use_requested)
            {
                set_reason_if_empty(
                    ctx.requested_fail_reasons,
                    agent_id,
                    "dependency on agent " + std::to_string(j) + " could not be resolved");
            }
            return false;
        }
        if (moving_boxes_collide(motion_i, ctx.decided_motions[j]))
        {
            if (use_requested)
            {
                string reason = "requested motion blocked by agent " + std::to_string(j);
                if (!dep_moved)
                    reason += " (agent waited)";
                set_reason_if_empty(ctx.requested_fail_reasons, agent_id, reason);
            }
            return false;
        }
    }

    ctx.has_decision[agent_id] = 1;
    ctx.chose_requested[agent_id] = use_requested ? 1 : 0;
    ctx.decided_actions[agent_id] = use_requested ? ctx.requested_actions[agent_id] : Action::W;
    ctx.decided_states[agent_id] = use_requested ? ctx.requested_states[agent_id] : ctx.wait_states[agent_id];
    ctx.decided_motions[agent_id] = motion_i;
    return true;
}

bool resolve_agent_recursive(int agent_id, StepResolveContext& ctx)
{
    if (ctx.has_decision[agent_id])
        return ctx.chose_requested[agent_id] != 0;
    if (ctx.status[agent_id] == VISITING)
        return false;

    ctx.status[agent_id] = VISITING;
    ctx.stack_pos[agent_id] = static_cast<int>(ctx.recursion_stack.size());
    ctx.recursion_stack.push_back(agent_id);
    vector<int> neighbors;
    neighbors.reserve(16);

    bool moved = try_commit_candidate(agent_id, true, ctx, neighbors);
    if (!moved)
        (void)try_commit_candidate(agent_id, false, ctx, neighbors);

    // Fallback safety: current state should always be valid, but never leave agent unresolved.
    if (!ctx.has_decision[agent_id])
    {
        ctx.has_decision[agent_id] = 1;
        ctx.chose_requested[agent_id] = 0;
        ctx.decided_actions[agent_id] = Action::W;
        ctx.decided_states[agent_id] = ctx.wait_states[agent_id];
        ctx.decided_motions[agent_id] = ctx.wait_motions[agent_id];
    }

    ctx.recursion_stack.pop_back();
    ctx.stack_pos[agent_id] = -1;
    ctx.status[agent_id] = DONE;
    return ctx.chose_requested[agent_id] != 0;
}
}


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
    next.timestep = prev.timestep + 1;
    Action executed_action = action;
    if (prev.counter.count > 0 && prev.current_action != Action::NA)
    {
        // Once an action has started, only a wait can pause it; any other command
        // must continue the in-progress action until completion.
        if (action == Action::W)
        {
            return next;
        }
        executed_action = prev.current_action;
    }

    if (executed_action == Action::FW)
    {
        if(next.counter.tick())
        {
            new_location = new_location += moves[prev.orientation];
            next.current_action = Action::NA;
        }
        else
        {
            next.current_action = Action::FW;
        }
    }
    else if (executed_action == Action::CR)
    {
        if(next.counter.tick())
        {
            new_orientation = (prev.orientation + 1) % 4;
            next.current_action = Action::NA;
        }
        else
        {
            next.current_action = Action::CR;
        }
  
    }
    else if (executed_action == Action::CCR)
    {
        if(next.counter.tick())
        {
            new_orientation = (prev.orientation - 1) % 4;
            if (new_orientation == -1)
                new_orientation = 3;
            next.current_action = Action::NA;
        }
        else
        {
            next.current_action = Action::CCR;
        }
    }
    else
    {
        next.current_action = prev.counter.count > 0 ? prev.current_action : Action::NA;
    }
    next.location = new_location;
    next.orientation = new_orientation;
    return next;
}

vector<ActionModelWithRotate::RealLocation> ActionModelWithRotate::get_real_locations(const vector<State>& state)
{
    vector<RealLocation> locations;
    locations.reserve(state.size());

    for (size_t i = 0; i < state.size(); i++)
    {
        const auto& s = state[i];
        RealLocation loc;
        const int row = s.location / cols;
        const int col = s.location % cols;
        float x = static_cast<float>(col);
        float y = static_cast<float>(row);

        // Only a forward action in progress produces translational offset; rotations stay in-cell.
        if (s.current_action == Action::FW && s.counter.maxCount > 0 && s.counter.count > 0)
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

void ActionModelWithRotate::sanity_check_states(const vector<State>& states)
{
    const int n = static_cast<int>(states.size());
    vector<RealLocation> real = get_real_locations(states);
    unordered_map<int, vector<int>> grid_agents;
    grid_agents.reserve(states.size() * 2 + 1);

    for (int i = 0; i < n; i++)
    {
        const int loc = states[i].location;
        if (loc < 0 || loc >= rows * cols)
        {
            throw std::runtime_error(
                "Sanity check failed: agent " + std::to_string(i) + " has invalid grid location");
        }

        Rect r;
        r.min_x = real[i].x;
        r.min_y = real[i].y;
        r.max_x = real[i].x + _agent_size;
        r.max_y = real[i].y + _agent_size;

        if (r.min_x < 0.0f || r.min_y < 0.0f ||
            r.max_x > static_cast<float>(cols) || r.max_y > static_cast<float>(rows))
        {
            throw std::runtime_error(
                "Sanity check failed: agent " + std::to_string(i) + " overlaps map boundary");
        }

        const int c0 = std::max(0, static_cast<int>(std::floor(r.min_x)));
        const int r0 = std::max(0, static_cast<int>(std::floor(r.min_y)));
        const int c1 = std::min(cols - 1, static_cast<int>(std::ceil(r.max_x) - 1));
        const int r1 = std::min(rows - 1, static_cast<int>(std::ceil(r.max_y) - 1));
        for (int rr = r0; rr <= r1; rr++)
        {
            for (int cc = c0; cc <= c1; cc++)
            {
                const int cell = rr * cols + cc;
                if (grid.map[cell] != 1)
                    continue;
                Rect obstacle;
                obstacle.min_x = static_cast<float>(cc);
                obstacle.min_y = static_cast<float>(rr);
                obstacle.max_x = obstacle.min_x + 1.0f;
                obstacle.max_y = obstacle.min_y + 1.0f;
                const bool obs_overlap =
                    r.min_x < obstacle.max_x - _overlap_eps && r.max_x > obstacle.min_x + _overlap_eps &&
                    r.min_y < obstacle.max_y - _overlap_eps && r.max_y > obstacle.min_y + _overlap_eps;
                if (obs_overlap)
                {
                    throw std::runtime_error(
                        "Sanity check failed: agent " + std::to_string(i) + " overlaps hard obstacle");
                }
            }
        }

        grid_agents[loc].push_back(i);
    }

    static const int kDr[9] = {0, -1, 1, 0, 0, -1, -1, 1, 1};
    static const int kDc[9] = {0, 0, 0, -1, 1, -1, 1, -1, 1};
    unordered_set<unsigned long long> checked_pairs;
    checked_pairs.reserve(states.size() * 4 + 1);

    for (int i = 0; i < n; i++)
    {
        Rect ri;
        ri.min_x = real[i].x;
        ri.min_y = real[i].y;
        ri.max_x = real[i].x + _agent_size;
        ri.max_y = real[i].y + _agent_size;

        const int row = states[i].location / cols;
        const int col = states[i].location % cols;
        for (int k = 0; k < 9; k++)
        {
            const int nr = row + kDr[k];
            const int nc = col + kDc[k];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                continue;
            auto it = grid_agents.find(nr * cols + nc);
            if (it == grid_agents.end())
                continue;
            for (int j : it->second)
            {
                if (j == i)
                    continue;
                const unsigned long long key = (static_cast<unsigned long long>(std::min(i, j)) << 32) |
                                               static_cast<unsigned long long>(std::max(i, j));
                if (!checked_pairs.insert(key).second)
                    continue;

                Rect rj;
                rj.min_x = real[j].x;
                rj.min_y = real[j].y;
                rj.max_x = real[j].x + _agent_size;
                rj.max_y = real[j].y + _agent_size;

                // Epsilon tolerance consistent with moving_boxes_collide:
                // agents whose boxes merely touch (overlap <= eps) are not
                // in conflict — avoids false positives from float rounding.
                const bool overlap =
                    ri.min_x < rj.max_x - _overlap_eps && ri.max_x > rj.min_x + _overlap_eps &&
                    ri.min_y < rj.max_y - _overlap_eps && ri.max_y > rj.min_y + _overlap_eps;
                if (overlap)
                {
                    throw std::runtime_error(
                        "Sanity check failed after dependency resolution: agent " + std::to_string(i) +
                        " overlaps agent " + std::to_string(j));
                }
            }
        }
    }
}

vector<State> ActionModelWithRotate::step(const vector<State>& prev, vector<Action> & actions, int timestep)
{
    (void)timestep;
    errors.clear();
    _wait_agents.assign(prev.size(), 0);
    if (prev.size() != actions.size())
    {
        throw std::invalid_argument("Size of state and action vectors must match.");
    }

    const int n = static_cast<int>(prev.size());
    const vector<Action> requested_actions = actions;
    vector<Action> wait_actions(prev.size(), Action::W);

    vector<State> requested_states = result_states(prev, actions);
    vector<State> wait_states = result_states(prev, wait_actions);

    vector<RealLocation> prev_real = get_real_locations(prev);
    vector<RealLocation> requested_real = get_real_locations(requested_states);
    vector<RealLocation> wait_real = get_real_locations(wait_states);

    vector<BoxMotion> current_boxes(n);
    vector<BoxMotion> requested_motions(n);
    vector<BoxMotion> wait_motions(n);
    vector<char> requested_valid(n, 1);
    vector<char> wait_valid(n, 1);

    for (int i = 0; i < n; i++)
    {
        current_boxes[i] = BoxMotion{prev_real[i].x, prev_real[i].y, prev_real[i].x, prev_real[i].y, _agent_size};
        requested_motions[i] = BoxMotion{prev_real[i].x, prev_real[i].y, requested_real[i].x, requested_real[i].y, _agent_size};
        wait_motions[i] = BoxMotion{prev_real[i].x, prev_real[i].y, wait_real[i].x, wait_real[i].y, _agent_size};

        if (requested_states[i].location < 0 || requested_states[i].location >= rows * cols)
            requested_valid[i] = 0;
        if (wait_states[i].location < 0 || wait_states[i].location >= rows * cols)
            wait_valid[i] = 0;

        if (requested_valid[i] &&
            (!motion_swept_bounds_valid(requested_motions[i], rows, cols) ||
             motion_hits_hard_obstacle(requested_motions[i], grid, rows, cols)))
            requested_valid[i] = 0;

        if (wait_valid[i] &&
            (!motion_swept_bounds_valid(wait_motions[i], rows, cols) ||
             motion_hits_hard_obstacle(wait_motions[i], grid, rows, cols)))
            wait_valid[i] = 0;
    }

    unordered_map<int, vector<int>> current_bins;
    current_bins.reserve(prev.size() * 4 + 1);
    for (int i = 0; i < n; i++)
        add_current_box_to_bins(current_boxes[i], i, rows, cols, current_bins);

    vector<char> status(n, UNSEEN);
    vector<char> has_decision(n, 0);
    vector<char> chose_requested(n, 0);
    vector<Action> decided_actions = actions;
    vector<State> decided_states = wait_states;
    vector<BoxMotion> decided_motions = wait_motions;
    vector<string> requested_fail_reasons(n);
    vector<int> seen_agents(n, -1);
    int seen_token = 0;
    vector<int> recursion_stack;
    recursion_stack.reserve(n);
    vector<int> stack_pos(n, -1);
    const int time = prev.empty() ? (timestep + 1) : (prev[0].timestep + 1);

    StepResolveContext ctx{
        grid,
        rows,
        cols,
        n,
        prev,
        requested_actions,
        requested_states,
        wait_states,
        requested_valid,
        wait_valid,
        requested_motions,
        wait_motions,
        current_bins,
        1.0f + _agent_size,
        status,
        has_decision,
        chose_requested,
        decided_actions,
        decided_states,
        decided_motions,
        requested_fail_reasons,
        seen_agents,
        seen_token,
        recursion_stack,
        stack_pos,
        errors,
        logger,
        time
    };

    for (int i = 0; i < n; i++)
        resolve_agent_recursive(i, ctx);

    for (int i = 0; i < n; i++)
    {
        actions[i] = decided_actions[i];
        if (actions[i] == Action::W && requested_actions[i] != Action::W)
        {
            _wait_agents[i] = 1;
            string reason = requested_fail_reasons[i].empty() ? "requested motion rejected by recursive dependency resolution" : requested_fail_reasons[i];
            std::cout << "Agent " << i << "'s state: " << prev[i] << ", requested action: " << requested_actions[i] << ", reason for waiting: " << reason << std::endl;
            errors.push_back(std::make_tuple(
                "Agent waits instead of " + action_to_string(requested_actions[i]) + ": " + reason,
                i,
                -1,
                time));

            if (logger != nullptr)
                logger->log_warning("Agent " + std::to_string(i) + " waits instead of " + action_to_string(requested_actions[i]) + ": " + reason, time);
        }
    }

    vector<State> next_states = result_states(prev, actions);
    sanity_check_states(next_states);
    return next_states;
}
