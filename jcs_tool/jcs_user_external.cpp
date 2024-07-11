// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include <ctime>
#include "jcs_user_external.h"
#include "task_rt.h"

namespace jcs {
namespace external {

void sleep_us(long int us) {
    task_rt::sleep_us(us);
}

static const long int nsec_per_sec = 1000000000;

long int time_now_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * nsec_per_sec + ts.tv_nsec;
}


} // End namespace external
} // End namespace jcs