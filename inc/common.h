#pragma once
#include <utility>
#include <tuple>
#include <list>
#include <vector>
#include <iostream>
#include <cfloat>
#include <ctime>
#include <fstream>
#include <random>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using boost::unordered_set;
using boost::unordered_map;

//got the error error: no member named 'set' in namespace 'std' in my machine
//so I have a quick check to see we do not use this in our program, and I just delete this for easy debug
//using std::set;
using std::vector;
using std::tuple;
using std::deque;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::list;
using std::cout;
using std::endl;
using std::ostream;
using std::string;
using std::max;
using std::min;
using std::priority_queue;

//#include <boost/graph/adjacency_list.hpp>
//typedef boost::adjacency_list_traits<int, int, boost::undirectedS > confilctGraph_t;
//typedef confilctGraph_t::vertex_descriptor vertex_t;
//typedef confilctGraph_t::edge_descriptor edge_t;

enum heuristics_type { NONE, CG, DG, WDG, STRATEGY_COUNT };

typedef tuple<int, int, int, int, bool> Constraint;
typedef tuple<int, int, int, int, int> Conflict;
// typedef vector<unordered_set<std::pair<int,int> > > ConstraintTable;
typedef tuple<int, int, bool> Interval; // [t_min, t_max), have conflicts or not
#define INTERVAL_MAX 10000

ostream& operator<<(ostream& os, const Constraint& constraint);

ostream& operator<<(ostream& os, const Conflict& conflict);

ostream& operator<<(ostream& os, const Interval& interval);

////////////////////////////////////////////////////
inline std::vector<int> read_int_vec(string fname, int team_size)
{
    std::vector<int> res;
	string line;
	std::ifstream myfile(fname.c_str());
	if (!myfile.is_open()) return {};

	getline(myfile, line);
    while (!myfile.eof() && line[0] == '#') 
    {
        getline(myfile, line);
    }

    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();

    int max_team_size = atoi((*beg).c_str());
    if (max_team_size < team_size)
    {
        std::cerr<<"Input file wrong, no enough agents in agent file";
        exit (-1);
    }
    // My benchmark
    for (int i = 0; i < team_size; i++) {

        getline(myfile, line);
        while (!myfile.eof() && line[0] == '#'){
            getline(myfile, line);
        }
        boost::tokenizer<boost::char_separator<char>> tok(line, sep);
        boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
        // read start [row,col] for agent i
        res.push_back(atoi((*beg).c_str()));

    }
    myfile.close();

	return res;
}


//inline std::vector<int> read_int_vec(string fname)
inline std::vector<list<int>> read_int_vec(string fname)
{
    std::vector<list<int>> res;
	string line;
	std::ifstream myfile(fname.c_str());
	if (!myfile.is_open()) return {};

	getline(myfile, line);
    while (!myfile.eof() && line[0] == '#') 
    {
        getline(myfile, line);
    }

    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();

    int team_size = atoi((*beg).c_str());
    // My benchmark
    for (int i = 0; i < team_size; i++)
    {

        getline(myfile, line);
        while (!myfile.eof() && line[0] == '#')
        {
            getline(myfile, line);
        }
        boost::tokenizer<boost::char_separator<char>> tok(line, sep);
        boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
        list<int> locs;
        for(;beg!=tok.end();++beg)
        {
            locs.push_back(atoi((*beg).c_str()));
        }
        res.push_back(locs);
    }
    myfile.close();

	return res;
}


template <typename T>
T read_param_json(nlohmann::json& data, std::string name)
{
    if (!data.contains(name))
    {
        std::cerr << "missing property " << name << " in the input JSON." << std::endl;
        exit(1);
    }
    try
    {
        return data[name].get<T>();
    }
    catch(nlohmann::json::type_error error)
    {
        std::cerr << "Incorrect input JSON format for " << name << std::endl;
        std::cerr << "Message: " << error.what() << std::endl;
        exit(1);
    }
}


template <typename T>
T read_param_json(nlohmann::json& data, std::string name, T default_value)
{
    if (!data.contains(name))
    {
        return default_value;
    }
    try
    {
        return data[name].get<T>();
    }
    catch(nlohmann::json::type_error error )
    {
        std::cerr << "Incorrect input JSON format for " << name << std::endl;
        std::cerr << "Message: " << error.what() << std::endl;
        exit(1);
    }
}


inline void load_delay_profile(const std::string& delay_file,
                               int team_size,
                               std::vector<std::pair<int, int>>& delay_ranges,
                               std::vector<std::vector<std::pair<int, int>>>& delay_schedule)
{
    nlohmann::json delay_data;
    std::ifstream delay_stream(delay_file);
    if (!delay_stream.is_open())
    {
        std::cerr << "Failed to open delay file " << delay_file << std::endl;
        exit(1);
    }

    try
    {
        delay_data = nlohmann::json::parse(delay_stream);
    }
    catch (nlohmann::json::parse_error error)
    {
        std::cerr << "Failed to parse delay file " << delay_file << std::endl;
        std::cerr << "Message: " << error.what() << std::endl;
        exit(1);
    }

    if (delay_data.contains("agent_delay_ranges") && delay_data["agent_delay_ranges"].is_array())
    {
        for (const auto& range_entry : delay_data["agent_delay_ranges"])
        {
            if (!range_entry.contains("agent") || !range_entry.contains("min") || !range_entry.contains("max"))
            {
                continue;
            }

            int agent = range_entry["agent"].get<int>();
            if (agent < 0 || agent >= team_size)
            {
                continue;
            }
            int min_delay = range_entry["min"].get<int>();
            int max_delay = range_entry["max"].get<int>();
            if (max_delay < min_delay)
            {
                std::swap(max_delay, min_delay);
            }
            delay_ranges[agent] = {min_delay, max_delay};
        }
    }

    if (delay_data.contains("schedule") && delay_data["schedule"].is_array())
    {
        for (const auto& step_entry : delay_data["schedule"])
        {
            if (!step_entry.contains("t") || !step_entry.contains("delays"))
            {
                continue;
            }

            int t = step_entry["t"].get<int>();
            if (t < 0)
            {
                continue;
            }
            if (t >= static_cast<int>(delay_schedule.size()))
            {
                delay_schedule.resize(t + 1);
            }

            const auto& step_delays = step_entry["delays"];
            if (!step_delays.is_array())
            {
                continue;
            }
            for (const auto& item : step_delays)
            {
                if (!item.is_array() || item.size() < 2)
                {
                    continue;
                }

                int agent = item[0].get<int>();
                int duration = item[1].get<int>();
                if (agent < 0 || agent >= team_size || duration <= 0)
                {
                    continue;
                }
                delay_schedule[t].push_back({agent, duration});
            }
        }
    }
}
