#include "Evaluation.h"


void DummyPlanner::load_plans(std::string fname){
    std::ifstream ifs(fname);
    auto jf = nlohmann::json::parse(ifs);
    if (!jf.contains("Planner Paths") || !jf["Planner Paths"].is_array()){
        return;
    }

    for  (auto it = jf["Planner Paths"].begin(); it != jf["Planner Paths"].end(); ++it)
    {
        if (!it->is_string())
        {
            agent_plans.clear();
            return;
        }
        agent_plans.emplace_back();
        for (auto& ch: it->get<std::string>())
        {
            if (ch=='W')
            {
                agent_plans.back().push_back(Action::W);
            }
            else if (ch=='C')
            {
                agent_plans.back().push_back(Action::CCR);
            }
            else if (ch=='R')
            {
                agent_plans.back().push_back(Action::CR);
            }
            else if (ch=='F')
            {
                agent_plans.back().push_back(Action::FW);
            }
        }
    }
}


std::vector<Action> DummyPlanner::plan(int time_limit)
{
    std::vector<Action> result;
    for (auto & dq: agent_plans)
    {
        if (!dq.empty())
        {
            result.push_back(dq.front());
            dq.pop_front();
        }
    }
    return result;
}
