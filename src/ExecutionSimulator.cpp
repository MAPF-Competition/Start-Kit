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

json stateToJSON(FreeState& state) {
    json obj = json::object();

    obj["x"] = state.x;
    obj["y"] = -state.y;
    obj["timestep"] = state.timestep;
    obj["theta"] = state.theta;
    
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
        // Parse the data section, ignoring response headers
        uint64_t split = response.rfind('\n', response.length());
        string data = response.substr(split, response.length() - split);

        json json_received = json::parse(data);

        vector<json> locations = json_received["locations"].get<std::vector<json>>();
        vector<State> curr_states(locations.size());
        for (int i = 0; i < curr_states.size(); i++) 
        {
            auto state_json = locations[i];
            int x = state_json["x"].get<int>();
            int y = abs(state_json["y"].get<int>());
            int theta = state_json["theta"].get<int>();
            int agent_id = state_json["agent_id"].get<int>();
            curr_states[agent_id] = State(xy_to_location(x, y), timestep, theta / 90); // Bad map from 0-360 to 0-3 (+x,+y,-x,-y)
            std::cout << agent_id << x << theta << std::endl;
        }
        return curr_states;
    } else {
        std::cout << "Empty response?" << std::endl;
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
        FreeState prepared_state = transform_state(next_states[i]);
        plans.push_back(stateToJSON(prepared_state));
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


