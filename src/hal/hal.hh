// #pragma once

#ifndef HALXX_HH
#define HALXX_HH

#include <map>
#include <string>
#include <variant>
#include <stdexcept>
#include <hal.h>
#include <hal_priv.h>
#include <string.h>


// ich würd alles in namespace hal {} stecken und dafür die hal_ prefixes weglassen

// std::strings als parameter: überall wo man potentiell einen c_str braucht würd ich
// const std::string& übergeben, wo man keinen c_str braucht einen string_view

class hal_mutex_guard
{
public:
    hal_mutex_guard()
    {
        rtapi_mutex_get(&(hal_data->mutex));
    }
    ~hal_mutex_guard()
    {
        rtapi_mutex_give(&(hal_data->mutex));
    }
};

enum class hal_dir
{
    IN = HAL_IN,
    OUT = HAL_OUT,
    IO = HAL_IO,
};


// comment RS: why not inherit from hal_pin_t? would that mean hal_param and hal_signal class
// templates are needed too?
template<typename T>
class hal_pin
{
public:
    std::string name;
    T** ptr;
    T operator=(const T& value)
    {
        **(const_cast<volatile T**>(ptr)) = value;
        return **ptr;
    }
    operator T()
    {
        return **(const_cast<volatile T**>(ptr));
    }
};

// requires
// typedef int hal_port;
// using hal_port = int;
struct hal_port
{
    hal_port(volatile hal_port& port)
    {
        ptr = port.ptr;
    }
    // public:
    int ptr;
    // operator int(){
    //     return ptr;
    // }
};
/*
funzt nicht

template<>
class hal_pin<hal_port> {
   public:
    std::string name;
    volatile hal_port** ptr;
    hal_port operator=(const hal_port& value) volatile {
        **ptr = value;
        return **ptr;
    }
    operator hal_port (){
        return **ptr;
    }
};
*/
using pin_t = std::
    variant<hal_pin<double>, hal_pin<bool>, hal_pin<int32_t>, hal_pin<uint32_t>, hal_pin<hal_port>>;
using pin_value_t = std::variant<double, bool, int32_t, uint32_t>;

// helper type for the visitor (should work with c++17)
template<class... Ts>
struct overloaded: Ts...
{
    using Ts::operator()...;
};
// explicit deduction guide (not needed as of C++20)
template<class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

class PyPin
{
public:
    pin_t pin_;
    PyPin(pin_t pin)
    {
        pin_ = pin;
    }
    PyPin()
    {}
    auto getname()
    {
        std::string s;
        std::visit([&s](auto&& pin) { s = pin.name; }, pin_);
        return s;
    }
    pin_value_t getitem()
    {
        return std::visit(overloaded {[](double arg) -> pin_value_t { return arg; },
                                      [](int32_t arg) -> pin_value_t { return arg; },
                                      [](uint32_t arg) -> pin_value_t { return arg; },
                                      [](bool arg) -> pin_value_t { return arg; },
                                      [](hal_port port) -> pin_value_t { return 0; }},
                          pin_);
        return 0;
    }

    void setitem_variant(pin_value_t value)
    {
        // much better would be
        // std::visit(overloaded{[this](auto arg){setitem(arg);}}, value);
        // but that doesn't compile
        std::visit(overloaded {[this](double arg) { setitem(arg); },
                               [this](int32_t arg) { setitem(arg); },
                               [this](uint32_t arg) { setitem(arg); },
                               [this](bool arg) { setitem(arg); },
                               [](hal_port) {}},
                   value);
    }

    template<typename T>
    void setitem(T value)
    {
        std::visit(overloaded {[value](hal_pin<double>& pin) { pin = value; },
                               [value](hal_pin<int32_t>& pin) { pin = value; },
                               [value](hal_pin<uint32_t>& pin) { pin = value; },
                               [value](hal_pin<bool>& pin) { pin = value; },
                               [](hal_pin<hal_port>& pin) {}},
                   pin_);
    }

    bool write(std::string data)
    {
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            auto ret =
                hal_port_write((**v->ptr).ptr, data.c_str(), data.length());  //(**v->ptr).ptr,
            return ret;
        }
        // not a port
        return false;
    }

    unsigned int writable()
    {
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            auto ret = hal_port_writable((**v->ptr).ptr);  //(**v->ptr).ptr,
            return ret;
        }
        // not a port
        return 0;
    }

    unsigned int readable()
    {
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            auto ret = hal_port_readable((**v->ptr).ptr);  //(**v->ptr).ptr,
            return ret;
        }
        // not a port
        return 0;
    }

    void waitWritable(unsigned int count)
    {
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            hal_port_wait_writable((hal_port_t**)(v->ptr), count, 0);  //(**v->ptr).ptr,
        }
    }

    std::string read(unsigned int count)
    {
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            std::string foo(count, '\0');
            auto ret = hal_port_read((**v->ptr).ptr, foo.data(), count);  //(**v->ptr).ptr,
            if (ret) {
                return foo;
            }
            else {
                return "";
            }
        }
        // not a port
        return "";
    }

    std::string peek(unsigned int count)
    {
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            std::string foo(count, '\0');
            auto ret = hal_port_peek((**v->ptr).ptr, foo.data(), count);  //(**v->ptr).ptr,
            if (ret) {
                return foo;
            }
            else {
                return "";
            }
        }
        // not a port
        return "";
    }

    unsigned int size()
    {
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            auto ret = hal_port_buffer_size((**v->ptr).ptr);  //(**v->ptr).ptr,
            return ret;
        }
        // not a port
        return 0;
    }
};

class hal
{
private:
    static int set_common(hal_type_t type, void* d_ptr, const char* value)
    {
        // This function assumes that the mutex is held
        int retval = 0;
        double fval;
        long lval;
        unsigned long ulval;
        long long llval;
        unsigned long long ullval;
        char* cp;

        switch (type) {
            case HAL_BIT:
                if ((strcmp("1", value) == 0) || (strcasecmp("TRUE", value) == 0)) {
                    *(hal_bit_t*)(d_ptr) = 1;
                }
                else if ((strcmp("0", value) == 0) || (strcasecmp("FALSE", value)) == 0) {
                    *(hal_bit_t*)(d_ptr) = 0;
                }
                else {
                    retval = -EINVAL;
                }
                break;
            case HAL_FLOAT:
                fval = strtod(value, &cp);
                if ((*cp != '\0') && (!isspace(*cp))) {
                    // invalid character(s) in string
                    retval = -EINVAL;
                }
                else {
                    *((hal_float_t*)(d_ptr)) = fval;
                }
                break;
            case HAL_S32:
                lval = strtol(value, &cp, 0);
                if ((*cp != '\0') && (!isspace(*cp))) {
                    // invalid chars in string
                    retval = -EINVAL;
                }
                else {
                    *((hal_s32_t*)(d_ptr)) = lval;
                }
                break;
            case HAL_U32:
                ulval = strtoul(value, &cp, 0);
                if ((*cp != '\0') && (!isspace(*cp))) {
                    // invalid chars in string
                    retval = -EINVAL;
                }
                else {
                    *((hal_u32_t*)(d_ptr)) = ulval;
                }
                break;
            case HAL_S64:
                llval = strtoll(value, &cp, 0);
                if ((*cp != '\0') && (!isspace(*cp))) {
                    // invalid chars in string
                    retval = -EINVAL;
                }
                else {
                    *((hal_s64_t*)(d_ptr)) = llval;
                }
                break;
            case HAL_U64:
                ullval = strtoull(value, &cp, 0);
                if ((*cp != '\0') && (!isspace(*cp))) {
                    // invalid chars in string
                    retval = -EINVAL;
                }
                else {
                    *((hal_u64_t*)(d_ptr)) = ullval;
                }
                break;
            default:
                // Shouldn't get here, but just in case...
                retval = -EINVAL;
        }
        return retval;
    }

public:
    static bool component_exists(std::string name)
    {
        return halpr_find_comp_by_name(name.c_str()) != NULL;
    }
    static bool pin_has_writer(std::string name)
    {
        hal_pin_t* pin = halpr_find_pin_by_name(name.c_str());
        if (!pin) {  // pin does not exist
            return false;
        }
        if (pin->signal) {
            hal_sig_t* signal = (hal_sig_t*)SHMPTR(pin->signal);
            return signal->writers > 0;
        }
        return false;
    }
    static bool component_is_ready(std::string name)
    {
        // Bad form to assume comp name exists - stop crashing!
        hal_comp_t* thecomp = halpr_find_comp_by_name(name.c_str());
        return thecomp && (thecomp->ready != 0);
    }
    static int sigNew(std::string name, hal_type_t dir)
    {
        return hal_signal_new(name.c_str(), dir);
    }
    static int sigLink(std::string pin_name, std::string sig_name)
    {
        return hal_link(pin_name.c_str(), sig_name.c_str());
    }
    // TODO
    static std::variant<double, bool, int32_t, uint32_t> get_value(const std::string& name)
    {
        hal_param_t* param;
        hal_pin_t* pin;
        hal_sig_t* sig;
        hal_type_t type;
        void* d_ptr;

        // if(!hal_shmem_base) {
        // PyErr_Format(PyExc_RuntimeError,
        // 	"Cannot call before creating component");
        // return NULL;
        // }
        /* get mutex before accessing shared data */

        hal_mutex_guard m;
        /* search param list for name */
        param = halpr_find_param_by_name(name.c_str());
        if (param) {
            /* found it */
            type = param->type;
            d_ptr = SHMPTR(param->data_ptr);
            /* convert to python value */
            switch (type) {
                case HAL_BIT:
                    return ((bool)*(hal_bit_t*)d_ptr);
                case HAL_U32:
                    return ((uint32_t) * (hal_u32_t*)d_ptr);
                case HAL_S32:
                    return ((int32_t) * (hal_s32_t*)d_ptr);
                // case HAL_U64: return ((unsigned long long)*(hal_u64_t *)d_ptr);
                // case HAL_S64: return ((long long)*(hal_s64_t *)d_ptr);
                case HAL_FLOAT:
                    return ((double)*(hal_float_t*)d_ptr);
                case HAL_PORT:  // HAL_PORT is currently not supported
                case HAL_TYPE_UNSPECIFIED: /* fallthrough */;
                case HAL_TYPE_UNINITIALIZED: /* fallthrough */;
                default:;  // gets rid of warning but makes sun go nova
            }
        }
        /* not found, search pin list for name */
        pin = halpr_find_pin_by_name(name.c_str());
        if (pin) {
            /* found it */
            type = pin->type;
            if (pin->signal != 0) {
                sig = (hal_sig_t*)SHMPTR(pin->signal);
                d_ptr = SHMPTR(sig->data_ptr);
            }
            else {
                sig = 0;
                d_ptr = &(pin->dummysig);
            }
            /* convert to python value */
            switch (type) {
                case HAL_BIT:
                    return ((bool)*(hal_bit_t*)d_ptr);
                case HAL_U32:
                    return ((uint32_t) * (hal_u32_t*)d_ptr);
                case HAL_S32:
                    return ((int32_t) * (hal_s32_t*)d_ptr);
                // case HAL_U64: return ((unsigned long long)*(hal_u64_t *)d_ptr);
                // case HAL_S64: return ((long long)*(hal_s64_t *)d_ptr);
                case HAL_FLOAT:
                    return ((double)*(hal_float_t*)d_ptr);
                case HAL_PORT:  // HAL_PORT is currently not supported
                case HAL_TYPE_UNSPECIFIED: /* fallthrough */;
                case HAL_TYPE_UNINITIALIZED: /* fallthrough */;
                default:;  // gets rid of warning but makes sun go nova
            }
        }
        sig = halpr_find_sig_by_name(name.c_str());
        if (sig != 0) {
            /* found it */
            type = sig->type;
            d_ptr = SHMPTR(sig->data_ptr);
            /* convert to python value */
            switch (type) {
                case HAL_BIT:
                    return ((bool)*(hal_bit_t*)d_ptr);
                case HAL_U32:
                    return ((uint32_t) * (hal_u32_t*)d_ptr);
                case HAL_S32:
                    return ((int32_t) * (hal_s32_t*)d_ptr);
                // case HAL_U64: return ((unsigned long long)*(hal_u64_t *)d_ptr);
                // case HAL_S64: return ((long long)*(hal_s64_t *)d_ptr);
                case HAL_FLOAT:
                    return ((double)*(hal_float_t*)d_ptr);
                case HAL_PORT:  // HAL_PORT is currently not supported
                case HAL_TYPE_UNSPECIFIED: /* fallthrough */;
                case HAL_TYPE_UNINITIALIZED: /* fallthrough */;
                default:;  // gets rid of warning but makes sun go nova
            }
        }
        /* error if here */
        throw std::invalid_argument("Can't set value: pin / param " + name + " not found");
    }

    static bool set_p(const std::string& name, const std::string& value)
    {
        hal_mutex_guard m;
        auto param = halpr_find_param_by_name(name.c_str());
        hal_type_t type;
        void* d_ptr;
        if (param == 0) {
            auto pin = halpr_find_pin_by_name(name.c_str());
            if (pin == 0) {
                throw std::invalid_argument("pin not found");
            }
            else {
                // found it
                type = pin->type;
                if (pin->dir == HAL_OUT) {
                    throw std::invalid_argument("pin not writable");
                }
                if (pin->signal != 0) {
                    throw std::invalid_argument("pin connected to signal");
                }
                d_ptr = (void*)&pin->dummysig;
            }
        }
        else {
            // found it
            type = param->type;
            /* is it read only? */
            if (param->dir == HAL_RO) {
                throw std::invalid_argument("param not writable");
            }
            d_ptr = SHMPTR(param->data_ptr);
        }
        auto retval = set_common(type, d_ptr, value.c_str());
        return retval != 0;
    }
    // TODO
    static auto get_info_signals()
    {
        std::map<std::string, int> ret;
        return ret;
    }
};

template<typename T>
struct hal_type_to_type_t
{
};
template<>
struct hal_type_to_type_t<bool>
{
    static const hal_type_t type_t_val = HAL_BIT;
};
template<>
struct hal_type_to_type_t<int32_t>
{
    static const hal_type_t type_t_val = HAL_S32;
};
template<>
struct hal_type_to_type_t<uint32_t>
{
    static const hal_type_t type_t_val = HAL_U32;
};
template<>
struct hal_type_to_type_t<double>
{
    static const hal_type_t type_t_val = HAL_FLOAT;
};
template<>
struct hal_type_to_type_t<hal_port>
{
    static const hal_type_t type_t_val = HAL_PORT;
};

class hal_comp
{
    int comp_id;
    std::string comp_name;
    std::map<std::string, pin_t> map;
    template<typename T>
    int add_pin_(const std::string& name, hal_dir dir, hal_pin<T> pin)
    {
        return hal_pin_new(name.c_str(),
                           hal_type_to_type_t<T>::type_t_val,
                           static_cast<hal_pin_dir_t>(dir),
                           (void**)(pin.ptr),
                           comp_id);
    }

public:
    int error = 0;
    hal_comp(const std::string& name)
    {
        comp_id = hal_init(name.c_str());
        comp_name = name;
        if (comp_id < 0) {
            error -= 1;
            rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed\n", comp_name.c_str());
            hal_exit(comp_id);
        }
    }
    hal_comp() = delete;

    void setprefix(const std::string& name)
    {
        comp_name = name;
    }

    std::string getprefix()
    {
        return comp_name;
    }

    template<typename T>
    PyPin newpin_(const std::string& name, hal_dir dir)
    {
        auto pin = hal_pin<T>();
        pin.name = name;
        add_pin(name, dir, pin);
        map[name] = pin;
        return PyPin(pin);
    }
    PyPin newpin(const std::string& name, hal_type_t type, hal_dir dir)
    {
        switch (type) {
            case HAL_BIT:
                return newpin_<bool>(name, dir);
            case HAL_FLOAT:
                return newpin_<double>(name, dir);
            case HAL_S32:
                return newpin_<int32_t>(name, dir);
            case HAL_U32:
                return newpin_<uint32_t>(name, dir);
            case HAL_PORT:
                return newpin_<hal_port>(name, dir);
                [[fallthrough]];
            default:
                break;
        }
        auto& pin = map[name];
        auto foo = PyPin(pin);
        return foo;
    }

    std::variant<double, bool, int32_t, uint32_t> getitem(const std::string& name)
    {
        // code duplication
        auto pin = map.at(name);
        return std::visit(overloaded {[](double arg) -> pin_value_t { return arg; },
                                      [](int32_t arg) -> pin_value_t { return arg; },
                                      [](uint32_t arg) -> pin_value_t { return arg; },
                                      [](bool arg) -> pin_value_t { return arg; },
                                      [](hal_port port) -> pin_value_t { return 0; }},
                          pin);
        return 0;
    }

    template<typename T>
    void setitem(const std::string& name, T value)
    {
        // code duplication
        auto pin = map.at(name);
        std::visit(overloaded {[value](hal_pin<double>& pin) { pin = value; },
                               [value](hal_pin<int32_t>& pin) { pin = value; },
                               [value](hal_pin<uint32_t>& pin) { pin = value; },
                               [value](hal_pin<bool>& pin) { pin = value; },
                               [](hal_pin<hal_port>& pin) {}},
                   pin);
    }

    void ready()
    {
        hal_ready(comp_id);
    }
    void exit()
    {
        if (comp_id > 0) {
            hal_exit(comp_id);
        }
        comp_id = -1;
    }

    template<typename T>
    void add_pin(const std::string& pin_name, hal_dir dir, hal_pin<T>& pin)
    {
        // warum nicht hal_malloc(sizeof(T)); ?
        // potentielle fussangel falls da was größeres dazugebaut wird
        pin.ptr = static_cast<T**>(hal_malloc(8));
        if (!pin.ptr) {
            error -= 1;  // really? not error = -1; ?
            rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_malloc() failed\n", pin_name.c_str());
            hal_exit(comp_id);
        }
        error += add_pin_(comp_name + "." + pin_name, dir, pin);
        if (error < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_pin_new() failed\n", pin_name.c_str());
            hal_exit(comp_id);
        }
    }

    ~hal_comp()
    {
        exit();  // kann sein dass das exit rekursiv aufruft? wäre das schädlich?
    }
};

#endif
