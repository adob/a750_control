#include "a750.h"

#include "lib/print.h"
#include "lib/serial/serial_linux.h"
#include <memory>

using namespace lib;
using namespace a750_control;

void a750_control::Robot::connect(str device_path, error err) {
    Robot &r = *this;
    if (r.connected) {
        err("already connected");
        return;
    }
    r.conn = std::make_shared<serial::Port>(serial::open(device_path, err));
    if (err) {
        return;
    }

    r.client.init(r.conn);
    
    client.start(err);
    if (err) {
        return;
    }
    r.connected = true;

    // eprint "a750_control: connection established";
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
        err("lunable to set realtime scheduling: ", strerror(errno));
        return;
  }
}
