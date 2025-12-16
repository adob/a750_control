#include "a750.h"

#include "lib/print.h"
#include "lib/serial/serial_linux.h"
#include <memory>

using namespace lib;
using namespace a750_control;

void a750_control::Robot::connect(str device_path, error err) {
    Robot &r = *this;
    r.conn = std::make_shared<serial::Port>(serial::open(device_path, err));
    if (err) {
        return;
    }

    r.client.init(r.conn);
    
    client.start(err);
    if (err) {
        return;
    }

    print "a750_control: connection established";
}
