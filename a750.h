#pragma once

#include <memory>

#include "lib/base.h"
#include "lib/serial/serial_linux.h"
#include "robot.pb_client.h"


namespace a750_control {
    struct Robot {
      std::shared_ptr<lib::serial::Port> conn;
      a750pb::RPCClient client;

      void connect(lib::str device_path, lib::error err);
    } ;

    void set_high_thread_priority(lib::error);
}