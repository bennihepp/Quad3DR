/*
 * options.h
 *
 *  Created on: Dec 23, 2016
 *      Author: bhepp
 */

#pragma once

#include <boost/program_options.hpp>
#include <memory>
#include "common.h"

namespace bh {

namespace po = boost::program_options;

class ConfigOptions {
public:

  class Error : public bh::Error {
  public:
    Error(const std::string& what)
          : bh::Error(what) {
    }

    Error(const char* what)
          : bh::Error(what) {
    }
  };

  ConfigOptions() {}

  virtual ~ConfigOptions() {}

  ConfigOptions(const std::string& prefix, const std::string& description = "", const std::string& delimiter = ".")
  : prefix_(prefix), delimiter_(delimiter), options_(description) {}

  const po::options_description& getBoostOptions() const {
    return options_;
  }

  void setVariablesMap(const po::variables_map& vm) {
    vm_ = vm;
  }

  std::string getFullOptionName(const std::string& name) {
    if (prefix_.empty()) {
      return name;
    }
    else {
      return prefix_ + delimiter_ + name;
    }
  }

  template <typename OtherOptions>
  OtherOptions getOptionsAs() {
    OtherOptions other_options;
    for (const boost::shared_ptr<po::option_description>& od : other_options.options_.options()) {
      const std::string value_key = getOptionKey(od->long_name());
      if (vm_.count(value_key) > 0) {
        other_options.vm_.insert(std::make_pair(value_key, po::variable_value(vm_[value_key])));
      }
      else if (od->semantic()->is_required()) {
        throw Error(std::string("Option ") + value_key + std::string(" is required but was not given"));
      }
    }
    return other_options;
  }

  template <typename OtherOptions>
  OtherOptions getSubOptionsAs(const std::string& new_prefix) {
    return getSubOptionsAs<OtherOptions>(new_prefix, delimiter_);
  }

  template <typename OtherOptions>
  OtherOptions getSubOptionsAs(const std::string& new_prefix, const std::string& new_delimiter = ".") {
    OtherOptions other_options;
    for (const boost::shared_ptr<po::option_description>& od : other_options.options_.options()) {
      const std::string value_key = getOptionKey(od->long_name());
      if (vm_.count(value_key) > 0) {
        const std::string new_value_key = getOptionKey(od->long_name(), new_prefix, new_delimiter);
        other_options.vm_.insert(std::make_pair(new_value_key, po::variable_value(vm_[value_key])));
      }
      else if (od->semantic()->is_required()) {
        throw Error(std::string("Option ") + value_key + std::string(" is required but was not given"));
      }
    }
    return other_options;
  }

  template <typename T>
  void addOption(const std::string& name) {
    options_.add_options()
        (getFullOptionName(name).c_str(), po::value<T>()->required(), name.c_str());
  }

  template <typename T>
  void addOption(const std::string& name, T init_value) {
    options_.add_options()
        (getFullOptionName(name).c_str(), po::value<T>()->default_value(init_value), name.c_str());
  }

  template <typename T>
  void addOptionalOption(const std::string& name) {
    options_.add_options()
        (getFullOptionName(name).c_str(), po::value<T>(), name.c_str());
  }

  template <typename T>
  void addOptionRequired(const std::string& name, T* value_ptr) {
    options_.add_options()
        (getFullOptionName(name).c_str(), po::value<T>(value_ptr)->required(), name.c_str());
  }

  template <typename T>
  void addOption(const std::string& name, T* value_ptr) {
    options_.add_options()
        (getFullOptionName(name).c_str(), po::value<T>(value_ptr), name.c_str());
  }

  template <typename T>
  void addOption(const std::string& name, T* value_ptr, T init_value) {
    options_.add_options()
      (getFullOptionName(name).c_str(), po::value<T>(value_ptr)->default_value(init_value), name.c_str());
  }

  bool isSet(const std::string& name) {
    std::string full_name = prefix_ + delimiter_ + name;
    return vm_.count(full_name) > 0;
  }

  template <typename T>
  const T& getValue(const char* c_str) const {
    return getValue<T>(std::string(c_str));
  }

  template <typename T>
  const T& getValue(const std::string& name) const {
    return getValue<T>(name, nullptr);
  }

  template <typename T>
  const T& getValue(const char* c_str, bool* is_set) const {
    return getValue<T>(std::string(c_str), is_set);
  }

  template <typename T>
  const T& getValue(const std::string& name, bool* is_set) const {
    std::string full_name = prefix_ + delimiter_ + name;
    if (is_set != nullptr) {
      *is_set = vm_.count(full_name) > 0;
    }
    return vm_[full_name].as<T>();
  }

  std::string getOptionKey(const std::string& name) const {
    return getOptionKey(name, prefix_, delimiter_);
  }

  static std::string getOptionKey(
          const std::string& name, const std::string& prefix, const std::string& delimiter = ".") {
    if (prefix.empty()) {
      return name;
    }
    else {
      return prefix + delimiter + name;
    }
  }

private:
  std::string prefix_;
  std::string delimiter_;
  po::options_description options_;
  po::variables_map vm_;
};

}
