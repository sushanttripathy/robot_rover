//
// Created by sushant on 7/27/19.
//

#ifndef ZMQ_IMAGE_SERVER_LOGGER_HPP
#define ZMQ_IMAGE_SERVER_LOGGER_HPP

#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/formatter_parser.hpp>
#include <fstream>
#include <string>
namespace bl = boost::log;
class logger {
public:
  logger(std::string file) : tag_(file) {
    using backend_type = bl::sinks::text_file_backend;
    using sink_type = bl::sinks::synchronous_sink<backend_type>;
    namespace kw = bl::keywords;

    auto backend = boost::make_shared<backend_type>(
        kw::file_name = file + "_%N.log", kw::rotation_size = 10 * 1024 * 1024,
        kw::time_based_rotation =
            bl::sinks::file::rotation_at_time_point(0, 0, 0),
        kw::auto_flush = true);

    auto sink = boost::make_shared<sink_type>(backend);
    sink->set_formatter(bl::parse_formatter(g_format));
    sink->set_filter(tag_attr == tag_);

    bl::core::get()->add_sink(sink);
  }

  void log(const std::string &s) {
    BOOST_LOG_SCOPED_THREAD_TAG("Tag", tag_);
    BOOST_LOG_SEV(g_logger, bl::trivial::info) << s;
  }

private:
  const std::string tag_;
};

#endif // ZMQ_IMAGE_SERVER_LOGGER_HPP
