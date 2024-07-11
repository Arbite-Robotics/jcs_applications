// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef CONFIG_HELPERS_H_
#define CONFIG_HELPERS_H_

#include <string>
#include "yaml-cpp/yaml.h"

namespace config {
    bool get_yaml_doc(std::string filename, YAML::Node& doc);
}

#endif