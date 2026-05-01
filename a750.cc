#include "a750.h"

#include "serialrpc/client.h"

#include "lib/print.h"

using namespace lib;
using namespace a750_control;

void a750_control::Robot::connect(str device_path, error err) {
    Robot &r = *this;
    if (r.connected) {
        err("already connected");
        return;
    }

    r.rpc_conn = serialrpc::connect(device_path, {&r.robot_service}, err);
    if (err) {
        return;
    }

    r.connected = true;
}

void a750_control::set_high_thread_priority(error err) {
    const int thread_priority = sched_get_priority_max(SCHED_FIFO);
    if (thread_priority == -1) {
        err("unable to get maximum possible thread priority: %s", strerror(errno));
        return;
    }

    sched_param thread_param = {
        .sched_priority = thread_priority,
    };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
        err("unable to set realtime scheduling: %v", strerror(errno));
        return;
  }
}
void a750_control::Robot::disconnect(error err) {
    Robot &r = *this;
    if (!r.connected) {
        return;
    }

    r.rpc_conn->close(err);
    if (err) {
        return;
    }
    
    r.rpc_conn = nil;
    r.robot_service = {};
    r.connected = false;
}
