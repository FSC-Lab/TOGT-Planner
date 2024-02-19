#pragma once

#include <exception>
#include <sstream>
#include "drolib/file/filesystem.hpp"
#include "drolib/file/yaml.hpp"

namespace drolib {

struct ParameterException : public std::exception {
  ParameterException() = default;
  ParameterException(const std::string& msg)
    : msg(std::string("Drolib Parameter Exception: ") + msg) {}
  const char* what() const throw() { return msg.c_str(); }

  const std::string msg{"Drolib Parameter Exception"};
};

struct ParameterBase {
  virtual ~ParameterBase() = default;
  virtual bool load(const fs::path& filename);
  virtual bool load(const Yaml& yaml);
  virtual bool valid() const;
};

}  // namespace drolib