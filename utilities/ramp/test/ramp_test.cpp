// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include <iostream>//cout
#include "../ramp.h"

int main(int argc, char* argv[]) {
    double dt = 0.1;
    ramp test_ramp(dt);

    // Ramp from 0 - 1 over a second. Should take 10 ticks
    test_ramp.start(0.0, 1.0, 1.0, 1.0, 1.0);
    int tick = 0;
    do {
        float ramp_val = test_ramp.step();
        std::cout << "tick: " << tick << ", ramp_val: " << ramp_val << "\n";
        tick++;
    } while (!test_ramp.is_done());

    // Ramp down
    test_ramp.start(1.0, 0.0, 1.0, 1.0, 1.0);
    tick = 0;
    do {
        float ramp_val = test_ramp.step();
        std::cout << "tick: " << tick << ", ramp_val: " << ramp_val << "\n";
        tick++;
    } while (!test_ramp.is_done());


    return 0;
}
