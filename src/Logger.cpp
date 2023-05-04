#include "Logger.h"
namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;

void Logger::set_logfile(std::string filename)
{
    logging::add_file_log
    (
        keywords::file_name = filename,
        keywords::format = "[%TimeStamp%]: %Message%"
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

void Logger::log_preprocessing(bool succ)
{
    logging::add_common_attributes();

    using namespace logging::trivial;
    src::severity_logger< severity_level > lg;
    if (!succ)
        BOOST_LOG_SEV(lg, fatal) << "Pre-processing Failed";
        //BOOST_LOG_TRIVIAL(fatal) << "Error: Preprocessing Failed";
    else
        BOOST_LOG_SEV(lg, info) << "Pre-processing Success";
        //BOOST_LOG_TRIVIAL(info) << "Preprocessing Success";
}

void Logger::log_plan(bool succ, int time)
{
    logging::add_common_attributes();

    using namespace logging::trivial;
    src::severity_logger< severity_level > lg;
    if (!succ)
    {
        BOOST_LOG_SEV(lg, fatal) << "Planner Timeout at "<< time;
    }
        
        //BOOST_LOG_TRIVIAL(warning) << "Warning: Planner Timeout";
}
