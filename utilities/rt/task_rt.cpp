// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "task_rt.h"
#include <stdint.h>
#include <iostream>
#include <sys/mman.h>   // Needed for mlockall()
#include <malloc.h>
#include <unistd.h>
#include <fcntl.h>

#include "jcs_host.h"

namespace task_rt {
    // 32MB pagefault free buffer region
    const size_t pre_alloc_size = 1024*1024*32;
    const int64_t nsec_per_sec = 1000000000;

    int ready_for_rt(bool do_mem_lock);
    int configure_memory_rt();
    void advance_timespec(struct timespec *ts, int64_t nsec);
}

//
// Start an RT thread using preempt-RT
// References:
// 1 - https://rt.wiki.kernel.org/index.php/Threaded_RT-application_with_memory_locking_and_stack_handling_example 
// 2 - linuxcnc: uspace_rtapi_app.cc 
// 3 - linuxcnc: uspace_common.h
// 4 - https://hebinglong.github.io/2018/08/15/Linux-with-PREEMPT-RT/#Examples
// 5 - https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/application_base
// 6 - https://www.linuxembedded.fr/2020/01/overcommit-memory-in-linux
// 7 - https://www.cs.auckland.ac.nz/references/unix/digital/APS33DTE/DOCU_005.HTM
// 
// Assume > 1 processor
int task_rt::task_start_rt(thd_context* ctx, bool do_mem_lock) {
    pthread_attr_t attr;
    int r = 0;

    if (ctx->rt_thread_fn == NULL) {
        std::cout << "rt_thread_fn cannot be NULL\n";
        return jcs::RET_ERROR;
    }

    r = pthread_attr_init(&attr);
    if (r < 0) {
        std::cout << "pthread_attr_init failed: " << r << std::endl;
        return jcs::RET_ERROR;
    }


    r = pthread_attr_setstacksize(&attr, pre_alloc_size);
    if (r < 0) {
        std::cout << "pthread_attr_setstacksize failed: " << r << std::endl;
        return jcs::RET_ERROR;
    }

    r = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (r < 0) {
        std::cout << "pthread_attr_setinheritsched failed: " << r << std::endl;
        return jcs::RET_ERROR;
    }
    int policy = SCHED_FIFO;
    r = pthread_attr_setschedpolicy(&attr, policy);
    if (r < 0) {
        std::cout << "pthread_attr_setschedpolicy failed: " << r << std::endl;
        return jcs::RET_ERROR;
    }

    struct sched_param schedparam;
    schedparam.sched_priority = 87; // preempt_rt
    
    r = pthread_attr_setschedparam(&attr, &schedparam);
    if (r < 0) {
        std::cout << "pthread_attr_setschedparam failed: " << r << std::endl;
        return jcs::RET_ERROR;
    }

    // Pin the RT application to CPU 1
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(1, &cpu_set);

    r = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set), &cpu_set);
    if (r < 0) {
        std::cout << "pthread_attr_setaffinity_np failed: " << r << std::endl;
        return jcs::RET_ERROR;
    }

    if (ready_for_rt(do_mem_lock) != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }

    // Start the thread
    r = pthread_create(&ctx->rt_thread, &attr, ctx->rt_thread_fn, ctx->thread_args);
    if (r < 0) {
        std::cout << "pthread_create failed: " << r << std::endl;
        return jcs::RET_ERROR;
    }
    return jcs::RET_OK;
}

void task_rt::task_stop_rt(thd_context* ctx) {
    pthread_cancel(ctx->rt_thread);
    pthread_join(ctx->rt_thread, NULL);
}

void task_rt::task_wait_rt(thd_context* ctx) {
    // pthread_cancel(ctx->thread);
    pthread_join(ctx->rt_thread, NULL);
}


int task_rt::configure_memory_rt() {
    int s;

    // Note: From [6] - mlockall locks ALL threads in the process - even non-realtime threads

    // Lock all current and future pages from being paged out by the OS
    s = mlockall( MCL_CURRENT | MCL_FUTURE );
    if (s < 0) {
        std::cout << "mlockall failed: " << s << std::endl;
        return jcs::RET_ERROR;
    }

    // Turn off malloc trimming
    mallopt(M_TRIM_THRESHOLD, -1);

    // Turn off mmap usage
    // mallopt(M_MMAP_MAX, 0);


    // Reserve some memory
    char* buffer = static_cast<char *>(malloc(task_rt::pre_alloc_size));
    if (buffer == NULL) {
        std::cout << "malloc(PRE_ALLOCATION_SIZE) failed\n";
        return jcs::RET_ERROR;   
    }

    long pagesize = sysconf(_SC_PAGESIZE);
    // Touch each page in this piece of memory to map it into RAM
    for (int i=0; i<task_rt::pre_alloc_size; i += pagesize) {
        // Each write to this buffe will generate a pagefault.
        // Once each pagefault is handled a page will be locked in
        // memory and never returned to the system
        buffer[i] = 0;
    }
    // buffer will now be released. As Glibc is configured such that it 
    // never gives back memory to the kernel, the memory allocated above is
    // locked for this process. All malloc() and new() calls come from
    // the memory pool reserved and locked above. Issuing free() and
    // delete() does NOT make this locking undone. So, with this locking
    // mechanism we can build C++ applications that will never run into
    // a major/minor pagefault, even with swapping enabled.
    free(buffer);

    return jcs::RET_OK;
}

int task_rt::ready_for_rt(bool do_mem_lock) {

    if (do_mem_lock) {
        if (configure_memory_rt() != jcs::RET_OK) {
            return jcs::RET_ERROR;
        }
    }

    // Prohibit machine from entering sleep states
    int fd = open("/dev/cpu_dma_latency", O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        std::cout << "Opening /dev/cpu_dma_latency failed: " << fd << std::endl;
        return jcs::RET_ERROR;
    }
    // Write that we want zero latency to cpu_dma_latency, 
    // then hold the file descriptor open for the duration of our program
    int r = write(fd, "\0\0\0\0", 4);
    if (r != 4) {
        std::cout << "Writing to /dev/cpu_dma_latency failed: " << r << std::endl;
        return jcs::RET_ERROR;
    }

    // Go and start RT tasks
    return jcs::RET_OK;
}

void task_rt::ready_cycle_rt(thd_context* ctx, int64_t cycle_time_ns) {
    clock_gettime(CLOCK_MONOTONIC, &ctx->rt_cycle_ts);
    // Round to nearest ms
    int ht = (ctx->rt_cycle_ts.tv_nsec / 1000000) + 1;
    ctx->rt_cycle_ts.tv_nsec = ht * 1000000;
}

void task_rt::wait_next_cycle_rt(thd_context* ctx, int64_t cycle_time_ns) {
    // Compute next cycle start time
    advance_timespec(&ctx->rt_cycle_ts, cycle_time_ns);

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (timespec_less_than(ctx->rt_cycle_ts, now)) {
        // std::cout << "OVERRUN\n";
        // Overrun happened - Dont sleep!
        return;
    }

    // Wait for next cycle
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ctx->rt_cycle_ts, NULL);
}

// Non RT thread
int task_rt::task_start(thd_context* ctx) {
    if (ctx->thread_fn == NULL) {
        std::cout << "thread_fn cannot be NULL\n";
        return jcs::RET_ERROR;
    }
    pthread_attr_t attr;
    int r = 0;

    r = pthread_attr_init(&attr);
    if (r < 0) {
        std::cout << "pthread_attr_init failed: " << r << std::endl;
        return jcs::RET_ERROR;
    }

    r = pthread_attr_setstacksize(&attr, pre_alloc_size);
    if (r < 0) {
        std::cout << "pthread_attr_setstacksize failed: " << r << std::endl;
        return jcs::RET_ERROR;
    }

    // Isolate CPU 1 for the realtime task
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(1, &cpu_set);

    r = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set), &cpu_set);
    if (r < 0) {
        std::cout << "pthread_attr_setaffinity_np failed: " << r << std::endl;
        return jcs::RET_ERROR;
    }

    // Start the thread
    r = pthread_create(&ctx->thread, &attr, ctx->thread_fn, ctx->thread_args);
    if (r < 0) {
        std::cout << "pthread_create failed: " << r << std::endl;
        return jcs::RET_ERROR;
    }
    return jcs::RET_OK;
}

void task_rt::task_wait(thd_context* ctx) {
    // pthread_cancel(ctx->thread);
    pthread_join(ctx->thread, NULL);
}


void task_rt::sleep_us(long int us) {
    struct timespec ts;
    // Get the current timespec
    clock_gettime(CLOCK_MONOTONIC, &ts);
    // Advance it
    task_rt::advance_timespec(&ts, us * 1000);
    // Sleep until time is up
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

void task_rt::sleep_ms(long int ms) {
    sleep_us(ms * 1000);
}

// int64_t task_rt::time_now_ns() {
//     struct timespec ts;
//     clock_gettime(CLOCK_MONOTONIC, &ts);
//     return ts.tv_sec * task_rt::nsec_per_sec + ts.tv_nsec;
// }

void task_rt::advance_timespec(struct timespec *ts, int64_t nsec) {
    int64_t add_sec;
    int64_t add_nsec;

    add_nsec = nsec % task_rt::nsec_per_sec;
    add_sec  = (nsec - add_nsec) / task_rt::nsec_per_sec;

    ts->tv_sec += add_sec;
    ts->tv_nsec += add_nsec;

    if ( ts->tv_nsec >= task_rt::nsec_per_sec ) {
        add_nsec = ts->tv_nsec % task_rt::nsec_per_sec;
        ts->tv_sec += (ts->tv_nsec - add_nsec) / task_rt::nsec_per_sec;
        ts->tv_nsec = add_nsec;
    }
}

