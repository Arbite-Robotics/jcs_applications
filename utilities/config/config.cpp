// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "config.h"
#include <iostream>
#include <fstream>  // ifstream

namespace config {

bool get_yaml_doc(std::string filename, YAML::Node& doc) {
    if (filename == "") { 
        std::cout << "Missing config file name\n";
        return false; 
    }

    std::ifstream fin(filename.c_str());
    if (fin.fail()) {
        std::cout << "Could not read config file " << filename << std::endl;
        return false;
    }
    doc = YAML::Load(fin);

    return true;
}


} // End namespace