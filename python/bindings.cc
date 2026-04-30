#include <pybind11/pybind11.h>

#include <string>
#include <string_view>

#include "../a750.h"
#include "lib/error.h"
#include "lib/fmt/fmt.h"
#include "lib/sync/mutex.h"

namespace py = pybind11;
using namespace lib;

// struct PyError : lib::ErrorReporter {
//     void handle(lib::Error& e) override {
//         String msg = fmt::stringify(e);
//         throw std::runtime_error(msg.std_string());
//     }
// } ;

static void py_err_hadler(const lib::Error& e) {
    String msg = fmt::stringify(e);
    throw std::runtime_error(msg.std_string());
}

struct PyRobot {
    explicit PyRobot(std::string_view device_path)
        : device_path(device_path) {}

    std::string device_path;
    a750_control::Robot robot;
    sync::Mutex mtx;

    bool connect(this PyRobot& p) {
        sync::Lock lock(p.mtx);

        // ErrorRecorder err;
        // a750_control::set_high_thread_priority(err);
        // if (err) {
        //     fmt::fprintf(os::stderr, "warning: a750_control: unable to set high thread priority: %s\n", err.msg);
        // }

        p.robot.connect(p.device_path, py_err_hadler);
        return p.robot.connected;
    }

    void disconnect(this PyRobot &p) {
        sync::Lock lock(p.mtx);
        p.robot.disconnect(py_err_hadler);
    }

    bool is_connected(this PyRobot &p) {
        sync::Lock lock(p.mtx);
        return p.robot.connected;
    }
};

PYBIND11_MODULE(_native, module, py::mod_gil_not_used()) {
    module.doc() = "Python bindings for a750_control";

    py::class_<PyRobot>(module, "Robot")
        .def(py::init<std::string_view>())
        .def("connect", &PyRobot::connect, py::call_guard<py::gil_scoped_release>())
        .def("disconnect", &PyRobot::disconnect, py::call_guard<py::gil_scoped_release>())
        .def("is_connected", &PyRobot::is_connected, py::call_guard<py::gil_scoped_release>());
}