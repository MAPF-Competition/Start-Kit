#include "ExecutionSimulator.h"
#include "States.h"
#include "nlohmann/json.hpp"
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/core/buffers_to_string.hpp>
#include <boost/beast/core/tcp_stream.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/http/dynamic_body.hpp>
#include <boost/beast/http/empty_body.hpp>
#include <boost/beast/http/file_body.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/http/verb.hpp>
#include <boost/beast/version.hpp>
#include <iostream>
#include <string>
#include <vector>

using json = nlohmann::json;
namespace beast = boost::beast; // from <boost/beast.hpp>
namespace http = beast::http;   // from <boost/beast/http.hpp>
namespace net = boost::asio;    // from <boost/asio.hpp>
using tcp = net::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

json stateToJSON(FreeState &state, int agent_id) {
  json obj = json::object();

  obj["x"] = state.x;
  obj["y"] = -state.y;
  obj["timestep"] = state.timestep;
  obj["theta"] = state.theta;
  obj["agent_id"] = agent_id;

  return obj;
}

bool validateStep(vector<State> &curr_states, vector<State> &next_states) {
  int length = next_states.size();
  for (int i = 0; i < length; i++) {
    for (int j = i + 1; j < length; j++) {
      std::cout << "Checking agents " << std::to_string(i) << " and " << std::to_string(j) << " at " << std::to_string(next_states.at(i).location) << " and " << std::to_string(next_states.at(j).location) << std::endl;
      if (next_states.at(i).location == next_states.at(j).location) {
        return false;
      }
    }
  }
  // This can also be O(n) with hash table?
  // Check Equal curr and next locations does not have swapping conflict
  for (int i = 0; i < length; i++) {
    for (int j = 0; j < length; j++) {
      if (i == j) {
        continue;
      }
      if (curr_states.at(i).location == next_states.at(j).location &&
         curr_states.at(j).location == next_states.at(i).location) {
        return false;
      }
    }
  }

  return true;
}

vector<State> TurtlebotExecutor::get_agent_locations(int timestep) {
  const string PATH = "/get_locations";
  using json = nlohmann::json;

  beast::tcp_stream stream(ioc_);
  tcp::resolver resolver(ioc_);

  auto const results = resolver.resolve(hostname, port);

  stream.connect(results);

  http::request<http::empty_body> request;
  request.method(http::verb::get);
  request.target(PATH);
  request.version(11);
  request.set(http::field::host, hostname + ":" + port);

  http::write(stream, request);

  beast::flat_buffer buffer;

  // Declare a container to hold the response
  http::response<http::dynamic_body> res;

  // Receive the HTTP response
  http::read(stream, buffer, res);


  int response_code = res.base().result_int();
  if (response_code == 404) {
    // Try again
    stream.close();
    std::cout << "Waiting until Central Controller is ready." << std::endl;
    sleep(1);
    return get_agent_locations(timestep);
  }
  if (res.base().result_int() != 200) { // OK response
    std::cout << "Unsuccessful GET" << std::endl;
    // Gracefully close the socket
    beast::error_code ec;
    (void)stream.socket().wait(boost::asio::ip::tcp::socket::wait_write, ec);
    // stream.socket().close();
    // not_connected happens sometimes
    // so don't bother reporting it.
    //
    if (ec && ec != beast::errc::not_connected)
      throw beast::system_error{ec};
    return vector<State>{}; // TODO: Handle unsuccessful GET more gracefully
  }
  json json_received = json::parse(beast::buffers_to_string(res.body().data()));

  vector<json> locations = json_received["locations"].get<std::vector<json>>();
  vector<State> curr_states(locations.size());
  for (int i = 0; i < curr_states.size(); i++) {
    auto state_json = locations[i];
    int x = state_json["x"].get<int>();
    int y = abs(state_json["y"].get<int>());
    int theta = state_json["theta"].get<int>();
    int agent_id = state_json["agent_id"].get<int>();
    curr_states[agent_id] =
        State(xy_to_location(x, y), timestep,
              theta / 90); // Bad map from 0-360 to 0-3 (+x,+y,-x,-y)
  }
  // Gracefully close the socket
  beast::error_code ec;
  (void)stream.socket().wait(boost::asio::ip::tcp::socket::wait_write, ec);
  // stream.close();
  // not_connected happens sometime
  // so don't bother reporting it.
  //
  if (ec && ec != beast::errc::not_connected)
    throw beast::system_error{ec};
  return curr_states;
}

void TurtlebotExecutor::send_plan(vector<State> &curr_states, vector<State> &next_states) {
  if (!validateStep(curr_states, next_states)) {
    std::cout << "Edge or vertex conflict detected, replanning" << std::endl;
    return;
  }


  const std::string PATH = "/extend_path";
  beast::tcp_stream stream(ioc_);
  tcp::resolver resolver(ioc_);

  auto const results = resolver.resolve(hostname, port);

  stream.connect(results);

  json plans = json::array();
  // The agent id is also the index, as the state vec is sized
  for (int i = 0; i < next_states.size(); i++) {
    FreeState prepared_state = transform_state(next_states[i]);
    plans.push_back(stateToJSON(prepared_state, i));
  }
  json payload = json::object();
  payload["plans"] = plans;

  std::cout << "Sending plan" << std::endl;
  std::cout << payload << std::endl; // Debugging serialisation

  http::request<http::string_body> req;
  req.method(http::verb::post);
  req.target(PATH);
  req.set(http::field::host, hostname + ":" + port);
  req.set(http::field::content_type, "application/json");
  req.set(http::field::user_agent, "LeagueOfRobotRunners");
  req.body() = payload.dump();
  req.prepare_payload();

  http::write(stream, req);

  beast::flat_buffer buffer;

  http::response<http::dynamic_body> res;

  http::read(stream, buffer, res);

  if (res.base().result_int() != 200) { // OK response
    std::cout << "Unsuccessful POST" << std::endl;
  }
  // Gracefully close the socket
  beast::error_code ec;
  (void)stream.socket().wait(boost::asio::ip::tcp::socket::wait_write, ec);
  // stream.socket().close();
  // not_connected happens sometimes
  // so don't bother reporting it.
  //
  if (ec && ec != beast::errc::not_connected)
    throw beast::system_error{ec};
  std::cout << "Successful POST" << std::endl;
  return;
}
