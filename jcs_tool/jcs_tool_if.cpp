// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "jcs_tool_if.h"

jcs_tool_if::jcs_tool_if(std::string name, jcs::jcs_host* host, bool use_mem_lock) : 
    name_(name), host_(host), use_mem_lock_(use_mem_lock_)
{

}

jcs_tool_if::~jcs_tool_if() {}

std::string const jcs_tool_if::name() {
    return name_;
}
