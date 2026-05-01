#include <atomic>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
#include <string_view>

#include "../a750.h"
#include "a750_control/generated/proto/robot_service.pb_msg.h"
#include "lib/error.h"
#include "lib/fmt/fmt.h"
#include "lib/sync/lock.h"
#include "lib/sync/mutex.h"
#include "lib/time/time.h"
#include "lib/math/math.h"

namespace py = pybind11;
using namespace lib;

static void py_err_handler(const lib::Error& e) {
    String msg = fmt::stringify(e);
    throw std::runtime_error(msg.std_string());
}

namespace a750 {
    const float GripperMPerRad = float(0.15268  / (M_PI / 180.0) / 1000.0);

    const std::array<float, 7> DefaultPositionGain = {
        240.f, 
        240.f,
        240.f,
        24.0f, 
        31.0f,  
        25.0f, 
        25.0f
    };

    const std::array<float, 7> DefaultVelGain = {
        3, 
        3, 
        3,                                  
        0.2f, 
        0.2f, 
        0.2f, 
        0.2f
    };

    const std::array<float, 7> MaxJointSpeedsRads = {
        math::deg2rad(180.f),
        math::deg2rad(195.f),
        math::deg2rad(180.f),
        math::deg2rad(225.f),
        math::deg2rad(225.f),
        math::deg2rad(225.f),
        math::deg2rad(225.f)
    };


    float gripper_pos_rad_to_m(float pos_rad) {
        return 0.06f + pos_rad * GripperMPerRad;
    }

    float gripper_vel_rads_to_ms(float pos_rad) {
        return pos_rad * GripperMPerRad;
    }

    float gripper_pos_m_to_rad(float pos_m) {
        return (pos_m - 0.06f) / GripperMPerRad;
    }

    float gripper_vel_ms_to_rads(float vel_ms) {
        return vel_ms / GripperMPerRad;
    }

    float gripper_torque_nm_to_force_n(float torque_nm) {
        return torque_nm / GripperMPerRad;
    }

    float gripper_force_n_to_torque_nm(float force_n) {
        return force_n * GripperMPerRad;
    }

    struct Gripper {
        float pos_m = 0;
        float vel_ms = 0;
        float force_n = 0;
        float temp_mosfet_c = 0;
        float temp_rotor_c = 0;
    } ;

    struct GripperCommand {
        float pos_setpoint_m = 0;
        float vel_setpoint_ms = 0;
        
        float pos_gain_n_per_m = 0;
        float vel_gain_ns_per_m = 0;
        float acc_mss = 0;
        float force_n = 0;
    } ;

    struct RobotState {
        a750pb::RobotState::Mode mode = {};
        a750pb::Joint joint1 = {};
        a750pb::Joint joint2 = {};
        a750pb::Joint joint3 = {};
        a750pb::Joint joint4 = {};
        a750pb::Joint joint5 = {};
        a750pb::Joint joint6 = {};
        Gripper gripper = {};
    } ;

    struct RobotCommand {
        a750pb::JointCommand joint1 = {};
        a750pb::JointCommand joint2 = {};
        a750pb::JointCommand joint3 = {};
        a750pb::JointCommand joint4 = {};
        a750pb::JointCommand joint5 = {};
        a750pb::JointCommand joint6 = {};
        GripperCommand gripper = {};
    } ;

    RobotState convert_robot_state(a750pb::RobotState const &state) {
        RobotState s = {};
        s.mode = state.mode;
        s.joint1 = state.joint1;
        s.joint2 = state.joint2;
        s.joint3 = state.joint3;
        s.joint4 = state.joint4;
        s.joint5 = state.joint5;
        s.joint6 = state.joint6;

        s.gripper.pos_m = gripper_pos_rad_to_m(state.gripper.pos_rad);
        s.gripper.vel_ms = gripper_vel_rads_to_ms(state.gripper.vel_rads);
        s.gripper.force_n = gripper_torque_nm_to_force_n(state.gripper.torque_nm);
        s.gripper.temp_mosfet_c = state.gripper.temp_mosfet_c;
        s.gripper.temp_rotor_c = state.gripper.temp_rotor_c;
        return s;
    }

    a750pb::JointCommand convert_gripper_command(GripperCommand const &in) {
        a750pb::JointCommand out = {};
        out.pos_setpoint_rad = gripper_pos_m_to_rad(in.pos_setpoint_m);
        out.vel_setpoint_rads = gripper_vel_ms_to_rads(in.vel_setpoint_ms);

        if (in.pos_gain_n_per_m < 0) {
            out.pos_gain_nmrad = DefaultPositionGain[6];
        } else {
            out.pos_gain_nmrad = in.pos_gain_n_per_m * GripperMPerRad * GripperMPerRad;
        }

        if (in.vel_gain_ns_per_m < 0) {
            out.vel_gain_nms_rad = DefaultVelGain[6];
        } else {
            out.vel_gain_nms_rad = in.vel_gain_ns_per_m * GripperMPerRad * GripperMPerRad;
        }

        out.acc_radss = in.acc_mss / GripperMPerRad;
        out.torque_nm = gripper_force_n_to_torque_nm(in.force_n);
        return out;
    }

    a750pb::JointCommand convert_joint_command(a750pb::JointCommand const &in, int joint_index) {
        a750pb::JointCommand out = in;
        
        if (out.pos_gain_nmrad < 0) {
            out.pos_gain_nmrad = DefaultPositionGain[joint_index];
        }

        if (out.vel_gain_nms_rad < 0) {
            out.vel_gain_nms_rad = DefaultVelGain[joint_index];
        }
        
        return out;
    }

    a750pb::CommandJointsRequest convert_command(RobotCommand const &in) {
        a750pb::CommandJointsRequest out = {};
        out.joint1 = convert_joint_command(in.joint1, 0);
        out.joint2 = convert_joint_command(in.joint2, 1);
        out.joint3 = convert_joint_command(in.joint3, 2);
        out.joint4 = convert_joint_command(in.joint4, 3);
        out.joint5 = convert_joint_command(in.joint5, 4);
        out.joint6 = convert_joint_command(in.joint6, 5);
        out.gripper = convert_gripper_command(in.gripper);
        return out;
    }

    struct ControlLoop {
        sync::Mutex mtx;
        a750_control::Robot robot;
        sync::go control_loop;
        a750::RobotState current_state;
        a750pb::CommandJointsRequest joints_command;

        static const int MaxCommandAge = 200; // number of control loop iterations after which command is considered stale
        uint command_age = MaxCommandAge;

        bool has_error = false;
        std::atomic<bool> stop_requested = false;
        String error_message;

        void connect(this ControlLoop& c, std::string_view device_path, error err) {
            sync::Lock lock(c.mtx);

            c.has_error = false;
            c.error_message = "";
            c.robot.connect(device_path, err);
        }

        void disconnect(this ControlLoop& c, error err) {
            bool should_join = false;
            {
                sync::Lock lock(c.mtx);
                if (c.control_loop.active) {
                    c.stop_requested.store(true);
                    should_join = true;
                }

                c.robot.disconnect(err);
            }
            
            if (should_join) {
                c.control_loop.join();
            }
        }

        bool is_connected(this ControlLoop& c) {
            sync::Lock lock(c.mtx);

            return c.robot.connected;
        }

        void start_control_loop(this ControlLoop& c, error err) {
            sync::Lock lock(c.mtx);
            
            if (c.control_loop.active) {
                err("control loop is already running");
                return;
            }

            if (!c.robot.connected) {
                err("not connected");
                return;
            }

            c.command_age = MaxCommandAge;
            c.stop_requested.store(false);
            c.control_loop = sync::go([&] {
                c.loop([&](const lib::Error& e) { c.handle_error(e); });
            });
        }

        void stop_control_loop(this ControlLoop& c, error err) {
            {
                sync::Lock lock(c.mtx);
                if (!c.control_loop.active) {
                    return;
                }

                c.stop_requested.store(true);
            }

            c.control_loop.join();
        }

        a750::RobotState get_current_state(this ControlLoop& c, error err) {
            sync::Lock lock(c.mtx);
            if (c.has_error) {
                err(c.error_message);
                return {};
            }
            if (!c.control_loop.active) {
                err("control loop is not running");
                return {};
            }

            return c.current_state;
        }

        void command_robot(this ControlLoop& c, const a750::RobotCommand& command, error err) {
            sync::Lock lock(c.mtx);
            if (c.has_error) {
                err(c.error_message);
                return;
            }
            if (!c.control_loop.active) {
                err("control loop is not running");
                return;
            }

            c.command_age = 0;
            c.joints_command = convert_command(command);
        }

        void command_joint_positions(this ControlLoop &c, const std::array<double, 6> &positions_rad, double velocity_ratio, error err) {
            sync::Lock lock(c.mtx);

            if (c.has_error) {
                err(c.error_message);
                return;
            }
            if (!c.control_loop.active) {
                err("control loop is not running");
                return;
            }

            a750pb::CommandJointsRequest command = {};
            if (c.command_age < MaxCommandAge) {
                command = c.joints_command;
            }
            
            command.joint1.pos_setpoint_rad = positions_rad[0];
            command.joint2.pos_setpoint_rad = positions_rad[1];
            command.joint3.pos_setpoint_rad = positions_rad[2];
            command.joint4.pos_setpoint_rad = positions_rad[3];
            command.joint5.pos_setpoint_rad = positions_rad[4];
            command.joint6.pos_setpoint_rad = positions_rad[5];

            command.joint1.pos_gain_nmrad = DefaultPositionGain[0];
            command.joint2.pos_gain_nmrad = DefaultPositionGain[1];
            command.joint3.pos_gain_nmrad = DefaultPositionGain[2];
            command.joint4.pos_gain_nmrad = DefaultPositionGain[3];
            command.joint5.pos_gain_nmrad = DefaultPositionGain[4];
            command.joint6.pos_gain_nmrad = DefaultPositionGain[5];

            command.joint1.vel_setpoint_rads = velocity_ratio * MaxJointSpeedsRads[0];
            command.joint2.vel_setpoint_rads = velocity_ratio * MaxJointSpeedsRads[1];
            command.joint3.vel_setpoint_rads = velocity_ratio * MaxJointSpeedsRads[2];
            command.joint4.vel_setpoint_rads = velocity_ratio * MaxJointSpeedsRads[3];
            command.joint5.vel_setpoint_rads = velocity_ratio * MaxJointSpeedsRads[4];
            command.joint6.vel_setpoint_rads = velocity_ratio * MaxJointSpeedsRads[5];

            command.joint1.vel_gain_nms_rad = DefaultVelGain[0];
            command.joint2.vel_gain_nms_rad = DefaultVelGain[1];
            command.joint3.vel_gain_nms_rad = DefaultVelGain[2];
            command.joint4.vel_gain_nms_rad = DefaultVelGain[3];
            command.joint5.vel_gain_nms_rad = DefaultVelGain[4];
            command.joint6.vel_gain_nms_rad = DefaultVelGain[5];

            c.command_age = 0;
            c.joints_command = command;
        }

        void command_gripper_position(this ControlLoop &c, double pos_m, error err) {
            sync::Lock lock(c.mtx);

            if (c.has_error) {
                err(c.error_message);
                return;
            }
            if (!c.control_loop.active) {
                err("control loop is not running");
                return;
            }

            a750pb::CommandJointsRequest command = {};
            if (c.command_age < MaxCommandAge) {
                command = c.joints_command;
            }

            command.gripper = convert_gripper_command(GripperCommand{
                .pos_setpoint_m = gripper_pos_m_to_rad(pos_m),
                .vel_setpoint_ms = 0,
                .pos_gain_n_per_m = DefaultPositionGain[6],
                .vel_gain_ns_per_m = 0,
                .acc_mss = 0,
                .force_n = 0
            });

            c.command_age = 0;
            c.joints_command = command;
        }

        void loop(this ControlLoop& c, error err) {
            c.robot.robot_service.start_realtime_control(err);
            if (err) {
                return;
            }

            time::LoopTimer timer(time::hz(1'000));
            timer.start();

            for (;;) {
                if (c.stop_requested.load()) {
                    c.robot.robot_service.stop_realtime_control(error::ignore);
                    return;
                }

                a750pb::CommandJointsRequest req = {};

                {
                    sync::Lock lock(c.mtx);
                    if (c.command_age < MaxCommandAge) {
                        req = c.joints_command;
                        c.command_age++;
                    }

                }

                a750pb::RobotState resp = c.robot.robot_service.command_joints(req, err);
                if (err) {
                    c.robot.robot_service.stop_realtime_control(error::ignore);
                    return;
                }

                {
                    sync::Lock lock(c.mtx);
                    c.current_state = a750::convert_robot_state(resp);
                }

                timer.delay();
            }
        }

        void handle_error(this ControlLoop& c, const lib::Error& e) {
            String msg = fmt::stringify(e);

            sync::Lock lock(c.mtx);
            c.has_error = true;
            c.error_message = std::move(msg);
        }

        virtual ~ControlLoop() {
            ControlLoop& c = *this;

            if (c.control_loop.active) {
                c.stop_requested.store(true);
            }

            if (c.robot.connected) {
                c.robot.disconnect(error::ignore);
            }
        }
    } ;
}

struct PyRobot {
    explicit PyRobot(std::string_view device_path)
        : device_path(device_path) {}

    std::string device_path;
    a750::ControlLoop control_loop;
        
    void connect(this PyRobot& p) {
        ErrorRecorder err;
        a750_control::set_high_thread_priority(err);
        if (err) {
            fmt::fprintf(os::stderr, "warning: a750_control: unable to set high thread priority: %s\n", err.msg);
        }

        p.control_loop.connect(p.device_path, py_err_handler);
    }

    void disconnect(this PyRobot &p) {
        p.control_loop.disconnect(py_err_handler);
    }

    bool is_connected(this PyRobot &p) {
        return p.control_loop.is_connected();
    }

    void start_control_loop(this PyRobot &p) {
        p.control_loop.start_control_loop(py_err_handler);
    }

    void stop_control_loop(this PyRobot &p) {
        p.control_loop.stop_control_loop(py_err_handler);
    }

    a750::RobotState get_current_state(this PyRobot &p) {
        return p.control_loop.get_current_state(py_err_handler);
    }

    void command_robot(this PyRobot &p, const a750::RobotCommand &command) {
        p.control_loop.command_robot(command, py_err_handler);
    }

    void command_joint_positions(this PyRobot &p, const std::array<double, 6> &positions_rad, double velocity_ratio) {
        p.control_loop.command_joint_positions(positions_rad, velocity_ratio, py_err_handler);
    }

    void command_gripper_position(this PyRobot &p, double pos_m) {
        p.control_loop.command_gripper_position(pos_m, py_err_handler);
    }
};

PYBIND11_MODULE(_native, module, py::mod_gil_not_used()) {
    module.doc() = "Python bindings for a750_control";

    py::class_<a750pb::Joint>(module, "Joint")
        .def(py::init<>())
        .def_readwrite("pos_rad", &a750pb::Joint::pos_rad)
        .def_readwrite("vel_rads", &a750pb::Joint::vel_rads)
        .def_readwrite("torque_nm", &a750pb::Joint::torque_nm)
        .def_readwrite("temp_mosfet_c", &a750pb::Joint::temp_mosfet_c)
        .def_readwrite("temp_rotor_c", &a750pb::Joint::temp_rotor_c);

    py::class_<a750::Gripper>(module, "Gripper")
        .def(py::init<>())
        .def_readwrite("pos_m", &a750::Gripper::pos_m)
        .def_readwrite("vel_ms", &a750::Gripper::vel_ms)
        .def_readwrite("force_n", &a750::Gripper::force_n)
        .def_readwrite("temp_mosfet_c", &a750::Gripper::temp_mosfet_c)
        .def_readwrite("temp_rotor_c", &a750::Gripper::temp_rotor_c);

    py::enum_<a750pb::RobotState::Mode>(module, "RobotMode")
        .value("Startup", a750pb::RobotState::Startup)
        .value("Enabled", a750pb::RobotState::Enabled)
        .value("Stopped", a750pb::RobotState::Stopped)
        .value("Fault", a750pb::RobotState::Fault)
        .export_values();

    py::class_<a750::RobotState>(module, "RobotState")
        .def(py::init<>())
        .def_readwrite("mode", &a750::RobotState::mode)
        .def_readwrite("joint1", &a750::RobotState::joint1)
        .def_readwrite("joint2", &a750::RobotState::joint2)
        .def_readwrite("joint3", &a750::RobotState::joint3)
        .def_readwrite("joint4", &a750::RobotState::joint4)
        .def_readwrite("joint5", &a750::RobotState::joint5)
        .def_readwrite("joint6", &a750::RobotState::joint6)
        .def_readwrite("gripper", &a750::RobotState::gripper);

    py::class_<a750::GripperCommand>(module, "GripperCommand")
        .def(py::init<>())
        .def_readwrite("pos_setpoint_m", &a750::GripperCommand::pos_setpoint_m)
        .def_readwrite("vel_setpoint_ms", &a750::GripperCommand::vel_setpoint_ms)
        .def_readwrite("pos_gain_n_per_m", &a750::GripperCommand::pos_gain_n_per_m)
        .def_readwrite("vel_gain_ns_per_m", &a750::GripperCommand::vel_gain_ns_per_m)
        .def_readwrite("acc_mss", &a750::GripperCommand::acc_mss)
        .def_readwrite("force_n", &a750::GripperCommand::force_n);

    py::class_<a750pb::JointCommand>(module, "JointCommand")
        .def(py::init<>())
        .def_readwrite("pos_setpoint_rad", &a750pb::JointCommand::pos_setpoint_rad)
        .def_readwrite("pos_gain_nmrad", &a750pb::JointCommand::pos_gain_nmrad)
        .def_readwrite("vel_setpoint_rads", &a750pb::JointCommand::vel_setpoint_rads)
        .def_readwrite("vel_gain_nms_rad", &a750pb::JointCommand::vel_gain_nms_rad)
        .def_readwrite("acc_radss", &a750pb::JointCommand::acc_radss)
        .def_readwrite("torque_nm", &a750pb::JointCommand::torque_nm);

    py::class_<a750::RobotCommand>(module, "RobotCommand")
        .def(py::init<>())
        .def_readwrite("joint1", &a750::RobotCommand::joint1)
        .def_readwrite("joint2", &a750::RobotCommand::joint2)
        .def_readwrite("joint3", &a750::RobotCommand::joint3)
        .def_readwrite("joint4", &a750::RobotCommand::joint4)
        .def_readwrite("joint5", &a750::RobotCommand::joint5)
        .def_readwrite("joint6", &a750::RobotCommand::joint6)
        .def_readwrite("gripper", &a750::RobotCommand::gripper);

    py::class_<PyRobot>(module, "Robot")
        .def(py::init<std::string_view>())
        .def("connect", &PyRobot::connect, py::call_guard<py::gil_scoped_release>())
        .def("disconnect", &PyRobot::disconnect, py::call_guard<py::gil_scoped_release>())
        .def("is_connected", &PyRobot::is_connected, py::call_guard<py::gil_scoped_release>())
        .def("start_control_loop", &PyRobot::start_control_loop, py::call_guard<py::gil_scoped_release>())
        .def("stop_control_loop", &PyRobot::stop_control_loop, py::call_guard<py::gil_scoped_release>())
        .def("get_current_state", &PyRobot::get_current_state)
        .def("command_robot", &PyRobot::command_robot)
        .def("command_joint_positions", &PyRobot::command_joint_positions)
        .def("command_gripper_position", &PyRobot::command_gripper_position);
}
