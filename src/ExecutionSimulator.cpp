#include "ExecutionSimulator.h"
#include "States.h"
#include <boost/asio/io_service.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <boost/asio/ip/address.hpp>
#include "nlohmann/json.hpp"



vector<State> TurtlebotExecutor::get_agent_locations(int timestep) {
    const std::string PATH = "/get_locations";
    boost::system::error_code ec;
    using namespace boost::asio;
    using json = nlohmann::json;

    io_service svc;
    ip::tcp::socket sock(svc);
    try
    {
        // Consider a custom DNS to resolve central controller dynamically instead of hard-coding a static IP
        sock.connect({{}, 8080}); // http://localhost:8080 for testing
    } catch (boost::system::system_error const& e)
    {
        std::cout << "Warning: could not connect to central controller for Turtlebot executor : " << e.what() << std::endl;
        // Safe failure??
    }
    std::string request("GET " + PATH + "?timestep=" + std::to_string(timestep) + " HTTP/1.0\r\n\r\n");
    sock.send(buffer(request));

    std::string response;
    do {
        char buf[1024];
        size_t bytes_transferred = sock.receive(buffer(buf), {}, ec);
        if (!ec) response.append(buf, buf + bytes_transferred);
    } while (!ec);
    if (response.length() > 0)
    {
        json json_received = json::parse(response);
        vector<json> str_vec = json_received["locations"].get<std::vector<json>>();
        vector<State> curr_states(str_vec.size());
        for (int i = 0; i < curr_states.size(); i++) 
        {
            auto state_json = str_vec[i];
            int location = state_json["location"].get<int>();
            int orientation = state_json["orientation"].get<int>();
            int agent_id = state_json["agent_id"].get<int>();
            curr_states[agent_id] = State(location, timestep, orientation);
            std::cout << agent_id << location << orientation << std::endl;
        }
        return curr_states;
    } else {
        return vector<State>(0);
    }
    
}   


