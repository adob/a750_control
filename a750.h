#pragma once

#include <memory>

#include "lib/base.h"
#include "lib/serial/serial_linux.h"
#include "generated/proto/robot_service.pb_client.h"

namespace a750_control {
    struct Robot {
      std::shared_ptr<serialrpc::Client> rpc_conn;;
      a750pb::RobotServiceStub robot_service;
      bool connected = false;

      void connect(lib::str device_path, lib::error err);
      void disconnect(lib::error err);
    } ;

    void set_high_thread_priority(lib::error);
}