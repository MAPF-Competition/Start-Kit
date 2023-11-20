#include "Logger.h"

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;


void Logger::set_logfile(std::string filename)
{
    this->sink = logging::add_file_log
        (
         keywords::file_name = filename,
         keywords::format = "[%TimeStamp%]: *%Severity%* %Message%"
         );
}


void Logger::init()
{
    // logging::add_file_log
    // (
    //     keywords::format = "[%TimeStamp%]: %Message%"
    // );
    logging::core::get()->set_filter
        (
         logging::trivial::severity >= logging::trivial::info
         );
}


void Logger::log_info(std::string input)
{
    logging::add_common_attributes();

    using namespace logging::trivial;
    src::severity_logger< severity_level > lg;
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
    logging::add_common_attributes();

    using namespace logging::trivial;
    src::severity_logger< severity_level > lg;
    BOOST_LOG_SEV(lg, fatal) << input;
}


void Logger::log_warning(std::string input)
{
    logging::add_common_attributes();

    using namespace logging::trivial;
    src::severity_logger< severity_level > lg;
    BOOST_LOG_SEV(lg, warning) << input;
}


void Logger::log_warning(std::string input, int timestep)
{
    log_warning("[timestep=" + std::to_string(timestep) + "] " + input);
}
