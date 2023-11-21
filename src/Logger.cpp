#include "Logger.h"

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
using namespace logging::trivial;
src::severity_logger< severity_level > lg;


Logger::Logger(std::string filename, int severity){
    this->core = logging::core::get();
    logging::add_common_attributes();
    logging::core::get()->set_filter
    (
        logging::trivial::severity >= severity
    );

    if (filename != "")
        logging::add_file_log
        (
            keywords::file_name = filename,
            keywords::format = "[%TimeStamp%]: *%Severity%* %Message%"
        );
}

void Logger::flush(){
    this->core->flush();
}


void Logger::log_info(std::string input)
{

    BOOST_LOG_SEV(lg, info) << input;
}


void Logger::log_info(std::string input, int timestep)
{
    log_info("[timestep=" + std::to_string(timestep) + "] " + input);
}


void Logger::log_fatal(std::string input, int timestep)
{
    log_fatal("[timestep=" + std::to_string(timestep) + "] " + input);
}


void Logger::log_fatal(std::string input)
{

    BOOST_LOG_SEV(lg, fatal) << input;
}


void Logger::log_warning(std::string input)
{

    BOOST_LOG_SEV(lg, warning) << input;
}


void Logger::log_warning(std::string input, int timestep)
{
    log_warning("[timestep=" + std::to_string(timestep) + "] " + input);
}
