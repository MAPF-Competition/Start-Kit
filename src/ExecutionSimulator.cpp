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

using json = nlohmann::json;
using namespace boost::asio;

json stateToJSON(State& curr_state) {
    json obj = json::object();

    obj["location"] = curr_state.location;
    obj["timestep"] = curr_state.timestep;
    obj["orientation"] = curr_state.orientation;
    
    return obj;
}

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
        sock.connect(controller_endpoint); // http://localhost:8080 for testing
    } catch (boost::system::system_error const& e)
    {
        std::cout << "Warning: could not connect to central controller for Turtlebot executor : " << e.what() << std::endl;
        // Safe failure??
    }
    std::string request("GET " + PATH + "?timestep=" + std::to_string(timestep) + " HTTP/1.1\r\n\r\n");
    sock.send(buffer(request));

    std::string response;
    do 
    {
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

void TurtlebotExecutor::send_plan(vector<State>& next_states) 
{
    const std::string PATH = "/extend_path";
    boost::system::error_code ec;

    
    json plans = json::array();
    for (int i = 0; i < next_states.size(); i++ )
    {
        plans.push_back(stateToJSON(next_states[i]));
    }
    json payload = json::object();
    payload["plans"] = plans;

    std::cout << payload << std::endl; // Debugging serialisation

    io_service svc;
    ip::tcp::socket sock(svc);
    try
    {
        // Consider a local DNS to resolve central controller dynamically by url instead of hard-coding a static IP
        sock.connect(controller_endpoint); // http://localhost:8080 for testing
    } catch (boost::system::system_error const& e)
    {
        std::cout << "Warning: could not connect to central controller for Turtlebot executor : " << e.what() << std::endl;
    }

    std::string serial_payload = payload.dump();
    std::string request("POST " + PATH + " HTTP/1.1\r\n"
                        + "Host: Planner\r\n"
                        + "Connection: close\r\n"
                        + "Accept: */*\r\n"
                        + "User-Agent: LeagueOfRobotRunners\r\n"
                        + "Content-Type: applications/json\r\n"
                        + "Content-Length: " + std::to_string(serial_payload.length()) + "\r\n"
                        + "\r\n"
                        + serial_payload
                        );
    sock.send(buffer(request));

    std::string response;
    do {
        char buf[1024];
        size_t bytes_transferred = sock.receive(buffer(buf), {}, ec);
        if (!ec) response.append(buf, buf + bytes_transferred);
    } while (!ec);
    
    if (response.length() > 0) // Should check if OK was received but eh
    {
        std::cout << response << std::endl;
    } else {
        std::cout << "Failed?" << std::endl;
    }
}


