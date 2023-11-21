#pragma once
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

namespace logging = boost::log;
namespace keywords = boost::log::keywords;



class Logger
{
public:
    Logger(std::string filename, int severity=logging::trivial::info);
    ~Logger(){};

    void log_info(std::string input);
    void log_info(std::string input, int timestep);
    void log_fatal(std::string input);
    void log_fatal(std::string input, int timestep);
    void log_warning(std::string input);
    void log_warning(std::string input, int timestep);
    void flush();
private:
    logging::core_ptr core;


};
