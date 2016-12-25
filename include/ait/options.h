/*
 * options.h
 *
 *  Created on: Dec 23, 2016
 *      Author: bhepp
 */

#pragma once

#include <boost/program_options.hpp>

namespace ait {

namespace po = boost::program_options;

class ConfigOptions {
public:

  ConfigOptions() {}

  virtual ~ConfigOptions() {}

  ConfigOptions(const std::string& prefix)
  : prefix_(prefix) {}

  ConfigOptions(const std::string& prefix, const std::string& description)
  : prefix_(prefix), options_(description) {}

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
      return prefix_ + "." + name;
    }
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
  void addOption(const std::string& name, T* value_ptr, bool required = false) {
    po::typed_value<T>* po_value = po::value<T>(value_ptr);
    if (required) {
      po_value = po_value->required();
    }
    options_.add_options()
          (getFullOptionName(name).c_str(), po_value, name.c_str());
  }

  template <typename T>
  void addOption(const std::string& name, T* value_ptr, T init_value) {
    options_.add_options()
        (getFullOptionName(name).c_str(), po::value<T>(value_ptr)->default_value(init_value), name.c_str());
  }

  bool isSet(const std::string& name) {
    std::string full_name = prefix_ + "." + name;
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
    std::string full_name = prefix_ + "." + name;
    if (is_set != nullptr) {
      *is_set = vm_.count(full_name) > 0;
    }
    return vm_[full_name].as<T>();
  }

private:
  std::string prefix_;
  po::options_description options_;
  po::variables_map vm_;
};

}
