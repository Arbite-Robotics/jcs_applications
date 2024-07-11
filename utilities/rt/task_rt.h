// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef HELPER_TASK_RT_H_
#define HELPER_TASK_RT_H_

#include <thread>
#include <ctime>


namespace task_rt {

    // Some helpers for starting Preempt-RT threads
    struct thd_context {
        pthread_t rt_thread;
        struct timespec rt_cycle_ts;

        // Attach your rt cyclic thread function
        void* (*rt_thread_fn)(void*);

        pthread_t thread;
        // Attach your non rt cyclic thread function
        void* (*thread_fn)(void*);

        // Attach your thread args to thread_args
        void* thread_args;

        thd_context() : rt_thread(),
                        rt_cycle_ts{0},
                        rt_thread_fn(nullptr),
                        thread(),
                        thread_fn(nullptr),
                        thread_args() {}
    };

    // Start and stop the thread
    int task_start_rt(thd_context* ctx, bool do_mem_lock=true);
    void task_wait_rt(thd_context* ctx);
    void task_stop_rt(thd_context* ctx);

    // Compute initial timespec.
    // Call just prior to entering periodic loop
    void ready_cycle_rt(thd_context* ctx, int64_t cycle_time_ns);
    // Sleep until next cycle
    void wait_next_cycle_rt(thd_context* ctx, int64_t cycle_time_ns);

    // Non RT thread
    int task_start(thd_context* ctx);
    void task_wait(thd_context* ctx);

    void sleep_us(long int us);
    void sleep_ms(long int ms);
    // int64_t time_now_ns();

    inline bool timespec_less_than(const struct timespec &ta, const struct timespec &tb) {
        if(ta.tv_sec < tb.tv_sec) { return 1; }
        if(ta.tv_sec > tb.tv_sec) { return 0; }
        return ta.tv_nsec < tb.tv_nsec;
    }

} // End namespace task_rt

#endif