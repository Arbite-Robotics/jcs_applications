//
//
//
#include <string>
// #include <stdio.h>
#include <iostream>
#include "jcs_host.h"
#include "jcs_user_external.h"
#include "task_rt.h"
#include "cmd_input_parser.h"


void* thread_host_rt(void* arg);
void* thread_host_param(void* arg);

struct thread_host_args {
    jcs::jcs_host* host;
    bool do_running;

    bool request_start;
    bool request_stop;
    bool request_shutdown;
    bool request_reset;
};

static task_rt::thd_context thread_host;
static thread_host_args     host_args;

int main(int argc, char* argv[]) {

    cmd_input_parser cmd_parser(argc, argv);

    std::string config_path = cmd_parser.cmd_option_get("-p");
    if (!config_path.empty()) {
        std::cout << "test_vec_pool: Read path: " << config_path << std::endl;
    } else {
        std::cout << "test_vec_pool: ERROR: Need config path with -p\n";
        return -1;
    }

    jcs::jcs_host* host = jcs::jcs_host::make_jcs_host(config_path, false, false);
    if (host == NULL) {
        std::cout << "test_vec_pool: ERROR: host initialise\n";
        return -1;
    }

    // Attach the host
    host_args.host = host;
    // Attach threads and parameters
    thread_host.rt_thread_fn = &thread_host_rt;
    thread_host.thread_fn    = &thread_host_param;
    thread_host.thread_args  = reinterpret_cast<void*>(&host_args);

    host_args.do_running = true;

    // Start non realtime thread
    if (task_rt::task_start(&thread_host) != jcs::RET_OK) {
        std::cout << "test_vec_pool: task_start failed\n";
        return -1;        
    }

    // Start the rt cyclic thread
    if (task_rt::task_start_rt(&thread_host) != jcs::RET_OK) {
        std::cout << "test_vec_pool: task_start_rt failed\n";
        return -1;        
    }

    // Wait for tasks to exit
    task_rt::task_wait_rt(&thread_host);
    task_rt::task_wait(&thread_host);
    return 0;
}

// All we do in this thread is tick the host and exchange data
// All parameter and config occurs in the non-rt thread
void* thread_host_rt(void* arg) {
    
    thread_host_args* host_args = (thread_host_args*)arg;
    jcs::jcs_host* host = host_args->host;

    // Configure signal storage
    std::vector<float> signals_in_f(host->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0));
    std::vector<float> signals_out_f(host->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0));

    std::cout << "test_vec_pool: About to enter RT loop at time " << jcs::external::time_now_ns() << "\n";
    std::cout << "test_vec_pool: Waiting for cyclic warmup\n";

    // Prepare cycle time. 
    // Noting jcs_host recalculates cycle_time_ns each tick
    // to account for Ethercat DC clocks
    // First guess of cycle_time_ns from jcs_host:
    int64_t cycle_time_ns = (int64_t)1e9 / (int64_t)host->base_frequency_get();
    task_rt::ready_cycle_rt(&thread_host, cycle_time_ns);

    while (host_args->do_running) {
        task_rt::wait_next_cycle_rt(&thread_host, cycle_time_ns);

        if (host->data_is_valid_rt()) {
            host->sig_input_set_rt(0, signals_in_f);
        }

        if (host->step_rt(&cycle_time_ns) != jcs::RET_OK) {
            std::cout << "test_vec_pool: Error: step_rt\n";
            // Bail - presently cant come back from a failure here
            host_args->do_running = false;
        }

        if (host->data_is_valid_rt()) {
            host->sig_output_get_rt(0, &signals_out_f);
        }
    }
    return 0;
}

void* thread_host_param(void* arg) {

    thread_host_args* host_args = (thread_host_args*)arg;
    jcs::jcs_host* host = host_args->host;

    // Wait for host_rt cyclic to become ready
    while (!host->cyclic_ready()) {
        // Should check for timeout here
        jcs::external::sleep_us(1e6);
        if (host_args->do_running == false) {
            std::cout << "test_vec_pool: Host cyclic_ready failed\n";
            return 0;
        }
    }

    // Start network configuration
    if (host->start_network() != jcs::RET_OK) {
        host->trigger_estop();
        host_args->do_running = false;
        return 0;
    }

    // Cycle for parameter and estop events
    while (host_args->do_running) {
        jcs::external::sleep_us(10000);

        if (host->has_estop()) {
            // Print estop reason
            if (host->device_error_estop_print() != jcs::RET_OK) {
                host_args->do_running = false;
                return 0;
            } 
        }

        if (host_args->request_stop) {
            host_args->request_stop = false;
            host->stop();
        }
        if (host_args->request_start) {
            host_args->request_start = false;
            if (host->ready_devices() != jcs::RET_OK) {
                continue;
            }
            if (host->start() != jcs::RET_OK) {
                continue;
            }
        }
        if (host_args->request_reset) {
            host_args->request_reset = false;
            host->reset();
        }
        if (host_args->request_shutdown) {
            host_args->request_shutdown = false;
            // Run stop lists and exit cyclic data exchange mode
            // Note: Do not exit RT thread (do_running = false) until we 
            // are done stopping and shutting down
            if (host->stop() != jcs::RET_OK) {
                host_args->do_running = false;
                return 0;
            }
            if (host->shutdown() != jcs::RET_OK) {
                host_args->do_running = false;
                return 0;
            } 
            host_args->do_running = false;
        }

    }
    return 0;
}
