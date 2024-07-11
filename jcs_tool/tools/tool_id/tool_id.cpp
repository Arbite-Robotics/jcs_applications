// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "tool_id.h"

#include "jcs_host.h"
#include <string>
#include <iostream>


tool_id::tool_id(std::string name, jcs::jcs_host* host) :
    jcs_tool_if(name, host, true)
{

}

// tool_id::~tool_id::() {}

int tool_id::load_config(std::string tool_config) {
    return jcs::RET_OK;
}

// No cyclic work to do
int tool_id::step_startup_rt() {
    return jcs::RET_OK;
}
int tool_id::step_rt() {
    return jcs::RET_OK;
}
int tool_id::step_shutdown_rt() {
    return jcs::RET_OK;
}

int tool_id::step_parameter_startup() {

    // Start network.
    // We have unknown devices on the network, so only
    // initialise dev_jc devices
    if (host_->start_network(true) != jcs::RET_OK) {
        std::cout << "ERROR: Failed to start network\n";
        return jcs::RET_ERROR;
    }

    // Fetch device ID for unknown device
    // Ports are configured in structure.yaml
    std::string const jc_device_name = "JC_0";
    std::string const unknown_device_name = "DEVICE_0";

    jcs::dev_type device_type;
    jcs::dev_id   device_id;

    std::cout << "Attempting to get device ID\n";

    if (host_->node_device_id_get(jc_device_name, unknown_device_name, &device_type, &device_id) != jcs::RET_OK) {
        std::cout << "ERROR: node_device_id_get failed\n";
        return jcs::RET_ERROR;
    }

    std::cout << "Got device type and ID:\n";
    std::cout << "Device type: " << std::hex << (unsigned int)device_type << std::dec << std::endl;
    std::cout << "Device ID config entry: device_id: [" << 
        device_id.id[0] << ", " <<
        device_id.id[1] << ", " <<
        device_id.id[2] << "]\n";

    // Return not ready here to shutdown
    return jcs::RET_NRDY;
}

// No cyclic parameter work to do
int tool_id::step_parameter() {
    return jcs::RET_OK;
}
int tool_id::step_parameter_shutdown() {
    return jcs::RET_OK;
}