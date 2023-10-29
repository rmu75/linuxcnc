#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <hal.hh>

namespace py = pybind11;

PYBIND11_MODULE(hal, m) {
    m.doc() = "Interface to linuxcnc hal";
    m.def("component_exists", &hal::component_exists);
    m.def("pin_has_writer", &hal::pin_has_writer);
    m.def("component_is_ready", &hal::component_is_ready);
    m.def("get_value", &hal::get_value);
    m.def("set_p", &hal::set_p);
    m.def("get_info_signals", &hal::get_info_signals);
    m.def("sigNew", &hal::sigNew);
    m.def("sigLink", &hal::sigLink);
    m.attr("is_kernelspace") = py::int_(rtapi_is_kernelspace());
    m.attr("is_userspace") = py::int_(!rtapi_is_kernelspace());
    m.def("connect", &hal_link);//TODO: check hal_shmem_base
    m.def("disconnect", &hal_unlink);//TODO: check hal_shmem_base
    m.def("new_sig", &hal_signal_new);

    py::class_<PyPin>(m, "Pin")
        .def(py::init<>())
        .def("get", &PyPin::getitem)
        .def("set", &PyPin::setitem<double>)
        .def("set", &PyPin::setitem<bool>)
        .def("set", &PyPin::setitem<int32_t>)
        .def("set", &PyPin::setitem<uint32_t>)
        //hal_port functions:
        .def("write", &PyPin::write)
        .def("size", &PyPin::size)
        .def("writable", &PyPin::writable)
        .def("readable", &PyPin::readable)
        .def("waitWritable", &PyPin::waitWritable)
        .def("read", [](PyPin t,unsigned int c){ return py::bytes(t.read(c));})
        .def("peek", [](PyPin t,unsigned int c){ return py::bytes(t.peek(c));})

        //.def_property("value", &PyPin::getitem, &PyPin::setitem<bool>)
        //.def_property("value", &PyPin::getitem, &PyPin::setitem<double>)
        .def_property("value", &PyPin::getitem, &PyPin::setitem_variant)
        // .def_property_readonly("name", &PyPin::getname)
        .def("get_name", &PyPin::getname);
        // .def("is_pin", &PyPin::is_pin)
        // .def("get_type", &PyPin::get_type)
        // .def("get_dir", &PyPin::get_dir);
        //.def_property("value", &PyPin::getitem, [](py::object o) { py::cast<> });

    py::class_<hal_comp>(m, "component")
        .def(py::init<std::string>())
        .def("newpin", &hal_comp::newpin)
        .def("setprefix", &hal_comp::setprefix)
        .def("getprefix", &hal_comp::getprefix)
        .def("getitem", &hal_comp::getitem)
        .def("__getitem__", &hal_comp::getitem)
        .def("__getattr__", &hal_comp::getitem)
        .def("__setitem__", &hal_comp::setitem<double>)
        .def("__setitem__", &hal_comp::setitem<bool>)
        .def("__setitem__", &hal_comp::setitem<int32_t>)
        .def("__setitem__", &hal_comp::setitem<uint32_t>)
        .def("__setattr__", &hal_comp::setitem<double>)
        .def("__setattr__", &hal_comp::setitem<bool>)
        .def("__setattr__", &hal_comp::setitem<int32_t>)
        .def("__setattr__", &hal_comp::setitem<uint32_t>)
        .def("__setitem__", &hal_comp::setitem<int32_t>)
        .def("__setitem__", &hal_comp::setitem<double>)
        .def("ready", &hal_comp::ready)
        .def("exit", &hal_comp::exit);

    py::enum_<hal_type_t>(m, "hal_type_t")
    .value("HAL_FLOAT", HAL_FLOAT)
    .value("HAL_BIT", HAL_BIT)
    .value("HAL_S32", HAL_S32)
    .value("HAL_U32", HAL_U32)
    .value("HAL_PORT", HAL_PORT)
    .export_values();

    py::enum_<hal_dir>(m, "hal_dir")
    .value("HAL_IN", hal_dir::IN)
    .value("HAL_OUT", hal_dir::OUT)
    .value("HAL_IO", hal_dir::IO)
    .export_values();
}
