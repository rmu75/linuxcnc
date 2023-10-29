#ifndef HALXX_HH
#define HALXX_HH

#include <map>
#include <string>
#include <variant>
#include <stdexcept>
#include <hal.h>
#include <hal_priv.h>
#include <string.h>

enum class hal_dir{
    IN = HAL_IN,
    OUT = HAL_OUT,
    IO = HAL_IO,
};

template<typename T>
class hal_pin{
    public:
    std::string name;
    volatile T** ptr;
    T operator=(const T& value){
        **ptr = value;
        return **ptr;
    }
    operator T(){
        return **ptr;
    }
};
//requires
//typedef int hal_port;
//using hal_port = int;
struct hal_port{
    //public:
    int ptr;
    // operator int(){
    //     return ptr;
    // }
};
using pin_t = std::variant<hal_pin<double>,hal_pin<bool>,hal_pin<int32_t>,hal_pin<uint32_t>,hal_pin<hal_port>>;

class PyPin{
    public:
    pin_t pin_;
    PyPin(pin_t pin){
        pin_ = pin;
    }
    PyPin(){
    }
    auto getname(){
        std::string s;
        std::visit([&s](auto&& pin){ s=pin.name; }, pin_);
        return s;
    }
    std::variant<double,bool,int32_t,uint32_t> getitem(){
        if (auto* v = std::get_if<hal_pin<double>>(&pin_)) {
            return *v;
        } else if (auto* v = std::get_if<hal_pin<bool>>(&pin_)) {
            return *v;
        } else if (auto* v = std::get_if<hal_pin<int32_t>>(&pin_)) {
            return *v;
        } else if (auto* v = std::get_if<hal_pin<uint32_t>>(&pin_)) {
            return *v;
        }
        return 0;
    }

    void setitem_variant(std::variant<double,bool,int32_t,uint32_t> value){
        if (auto* p = std::get_if<double>(&value)) {
            setitem<double>(*p);
        } else if (auto* p = std::get_if<bool>(&value)) {
            setitem<bool>(*p);
        } else if (auto* p = std::get_if<int32_t>(&value)) {
            setitem<int32_t>(*p);
        } else if (auto* p = std::get_if<uint32_t>(&value)) {
            setitem<uint32_t>(*p);
        }
    }

    template<typename T>
    void setitem(T value){
        if (auto* p = std::get_if<hal_pin<double>>(&pin_)) {
            *p = value;
        } else if (auto* p = std::get_if<hal_pin<bool>>(&pin_)) {
            *p = value;
        } else if (auto* p = std::get_if<hal_pin<int32_t>>(&pin_)) {
            *p = value;
        } else if (auto* p = std::get_if<hal_pin<uint32_t>>(&pin_)) {
            *p = value;
        }
    }

    bool write(std::string data){
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            auto ret = hal_port_write((**v->ptr).ptr, data.c_str(), data.length());//(**v->ptr).ptr,
            return ret;
        }
        //not a port
        return false;
    }

    unsigned int writable(){
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            auto ret = hal_port_writable((**v->ptr).ptr);//(**v->ptr).ptr,
            return ret;
        }
        //not a port
        return 0;
    }

    unsigned int readable(){
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            auto ret = hal_port_readable((**v->ptr).ptr);//(**v->ptr).ptr,
            return ret;
        }
        //not a port
        return 0;
    }

    void waitWritable(unsigned int count){
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            hal_port_wait_writable((hal_port_t**)(v->ptr), count, 0);//(**v->ptr).ptr,
        }
    }

    std::string read(unsigned int count){
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            std::string foo(count, '\0');
            auto ret = hal_port_read((**v->ptr).ptr, foo.data(), count);//(**v->ptr).ptr,
            if(ret)
                return foo;
            else
                return "";
        }
        //not a port
        return "";
    }

    std::string peek(unsigned int count){
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            std::string foo(count, '\0');
            auto ret = hal_port_peek((**v->ptr).ptr, foo.data(), count);//(**v->ptr).ptr,
            if(ret)
                return foo;
            else
                return "";
        }
        //not a port
        return "";
    }

    unsigned int size(){
        if (auto* v = std::get_if<hal_pin<hal_port>>(&pin_)) {
            auto ret = hal_port_buffer_size((**v->ptr).ptr);//(**v->ptr).ptr,
            return ret;
        }
        //not a port
        return 0;
    }
};

class hal{
    private:
    static int set_common(hal_type_t type, void *d_ptr, const char *value) {
        // This function assumes that the mutex is held
        int retval = 0;
        double fval;
        long lval;
        unsigned long ulval;
        long long llval;
        unsigned long long ullval;
        char *cp;

        switch (type) {
        case HAL_BIT:
        if ((strcmp("1", value) == 0) || (strcasecmp("TRUE", value) == 0)) {
            *(hal_bit_t *) (d_ptr) = 1;
        } else if ((strcmp("0", value) == 0) || (strcasecmp("FALSE", value)) == 0) {
            *(hal_bit_t *) (d_ptr) = 0;
        } else {
            retval = -EINVAL;
        }
        break;
        case HAL_FLOAT:
        fval = strtod ( value, &cp );
        if ((*cp != '\0') && (!isspace(*cp))) {
            // invalid character(s) in string
            retval = -EINVAL;
        } else {
            *((hal_float_t *) (d_ptr)) = fval;
        }
        break;
        case HAL_S32:
        lval = strtol(value, &cp, 0);
        if ((*cp != '\0') && (!isspace(*cp))) {
            // invalid chars in string
            retval = -EINVAL;
        } else {
            *((hal_s32_t *) (d_ptr)) = lval;
        }
        break;
        case HAL_U32:
        ulval = strtoul(value, &cp, 0);
        if ((*cp != '\0') && (!isspace(*cp))) {
            // invalid chars in string
            retval = -EINVAL;
        } else {
            *((hal_u32_t *) (d_ptr)) = ulval;
        }
        break;
        case HAL_S64:
        llval = strtoll(value, &cp, 0);
        if ((*cp != '\0') && (!isspace(*cp))) {
            // invalid chars in string
            retval = -EINVAL;
        } else {
            *((hal_s64_t *) (d_ptr)) = llval;
        }
        break;
        case HAL_U64:
        ullval = strtoull(value, &cp, 0);
        if ((*cp != '\0') && (!isspace(*cp))) {
            // invalid chars in string
            retval = -EINVAL;
        } else {
            *((hal_u64_t *) (d_ptr)) = ullval;
        }
        break;
        default:
        // Shouldn't get here, but just in case... 
        retval = -EINVAL;
        }
        return retval;
    }
    public:
    static bool component_exists(std::string name){
        return halpr_find_comp_by_name(name.c_str()) != NULL;
    }
    static bool pin_has_writer(std::string name){
        hal_pin_t *pin = halpr_find_pin_by_name(name.c_str());
        if(!pin) {//pin does not exist
            return false;
        }
        if(pin->signal) {
            hal_sig_t *signal = (hal_sig_t*)SHMPTR(pin->signal);
            return signal->writers > 0;
        }
        return false;
    }
    static bool component_is_ready(std::string name){
        // Bad form to assume comp name exists - stop crashing!
        hal_comp_t *thecomp = halpr_find_comp_by_name(name.c_str());
        return thecomp && (thecomp->ready != 0);
    }
    static int sigNew(std::string name, hal_type_t dir){
        return hal_signal_new(name.c_str(), dir);
    }
    static int sigLink(std::string pin_name, std::string sig_name){
        return hal_link(pin_name.c_str(), sig_name.c_str());
    }
    //TODO
    static int get_value(std::string name){
        return 0;
    }
    //TODO
    static void set_p(std::string name, std::string value){

    static bool set_p(const std::string &name, const std::string &value){
        rtapi_mutex_get(&(hal_data->mutex));
        auto param = halpr_find_param_by_name(name.c_str());
        hal_type_t type;
        void *d_ptr;
        if (param == 0) {
            auto pin = halpr_find_pin_by_name(name.c_str());
            if(pin == 0) {
                rtapi_mutex_give(&(hal_data->mutex));
                throw std::invalid_argument("pin not found");
            } else {
                // found it 
                type = pin->type;
                if(pin->dir == HAL_OUT) {
                    rtapi_mutex_give(&(hal_data->mutex));
                    throw std::invalid_argument("pin not writable");
                }
                if(pin->signal != 0) {
                    rtapi_mutex_give(&(hal_data->mutex));
                    throw std::invalid_argument("pin connected to signal");
                }
                d_ptr = (void*)&pin->dummysig;
            }
        } else {
            // found it 
            type = param->type;
            /* is it read only? */
            if (param->dir == HAL_RO) {
                rtapi_mutex_give(&(hal_data->mutex));
                throw std::invalid_argument("param not writable");
            }
            d_ptr = SHMPTR(param->data_ptr);
        }
        auto retval = set_common(type, d_ptr, value.c_str());
        rtapi_mutex_give(&(hal_data->mutex));   
        return retval != 0;
    }
    //TODO
    static auto get_info_signals(){
        std::map<std::string, int> ret;
        return ret;
    }
};

class hal_comp{
    int comp_id;
    std::string comp_name;
    std::map<std::string,pin_t> map;
    int add_pin_(std::string name, hal_dir dir, hal_pin<bool> pin){
        return hal_pin_new(name.c_str(), HAL_BIT, static_cast<hal_pin_dir_t>(dir), (void **)(pin.ptr), comp_id);
    }
    int add_pin_(std::string name, hal_dir dir, hal_pin<int32_t> pin){
        return hal_pin_new(name.c_str(), HAL_S32, static_cast<hal_pin_dir_t>(dir), (void **)(pin.ptr), comp_id);
    }
    int add_pin_(std::string name, hal_dir dir, hal_pin<uint32_t> pin){
        return hal_pin_new(name.c_str(), HAL_U32, static_cast<hal_pin_dir_t>(dir), (void **)(pin.ptr), comp_id);
    }
    int add_pin_(std::string name, hal_dir dir, hal_pin<double> pin){
        return hal_pin_new(name.c_str(), HAL_FLOAT, static_cast<hal_pin_dir_t>(dir), (void **)(pin.ptr), comp_id);
    }
    int add_pin_(std::string name, hal_dir dir, hal_pin<hal_port> pin){
        return hal_pin_new(name.c_str(), HAL_PORT, static_cast<hal_pin_dir_t>(dir), (void **)(pin.ptr), comp_id);
    }
    public:
    int error = 0;
    hal_comp(std::string name){
        comp_id = hal_init(name.c_str());
        comp_name = name;
        if(comp_id < 0){
            error -= 1;
            rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed\n", comp_name.c_str());
            hal_exit(comp_id);
        }
    }
    hal_comp() = delete;

    void setprefix(std::string name){
        comp_name = name;
    }
    
    std::string getprefix(){
        return comp_name;
    }

    PyPin newpin(std::string name, hal_type_t type, hal_dir dir){
        auto& pin = map[name];
        switch(type){
            case HAL_BIT:
            pin = hal_pin<bool>();
            std::get<hal_pin<bool>>(pin).name = name;
            add_pin(name, dir, std::get<hal_pin<bool>>(pin));
            break;
            case HAL_FLOAT:
            pin = hal_pin<double>();
            std::get<hal_pin<double>>(pin).name = name;
            add_pin(name, dir, std::get<hal_pin<double>>(pin));
            break;
            case HAL_S32:
            pin = hal_pin<int32_t>();
            std::get<hal_pin<int32_t>>(pin).name = name;
            add_pin(name, dir, std::get<hal_pin<int32_t>>(pin));
            break;
            case HAL_U32:
            pin = hal_pin<uint32_t>();
            std::get<hal_pin<uint32_t>>(pin).name = name;
            add_pin(name, dir, std::get<hal_pin<uint32_t>>(pin));
            break;
            case HAL_PORT:
            pin = hal_pin<hal_port>();
            std::get<hal_pin<hal_port>>(pin).name = name;
            add_pin(name, dir, std::get<hal_pin<hal_port>>(pin));
            break;
            [[fallthrough]];
            default:
            break;
        }
        auto foo = PyPin(pin);
        return foo;
    }

    std::variant<double,bool,int32_t,uint32_t> getitem(std::string name){
        auto pin = map.at(name);
        if (auto* v = std::get_if<hal_pin<double>>(&pin)) {
            return *v;
        } else if (auto* v = std::get_if<hal_pin<bool>>(&pin)) {
            return *v;
        } else if (auto* v = std::get_if<hal_pin<int32_t>>(&pin)) {
            return *v;
        } else if (auto* v = std::get_if<hal_pin<uint32_t>>(&pin)) {
            return *v;
        }
        return 0;
    }

    template<typename T>
    void setitem(std::string name, T value){
        auto pin = map.at(name);
        if (auto* p = std::get_if<hal_pin<double>>(&pin)) {
            *p = value;
        } else if (auto* p = std::get_if<hal_pin<bool>>(&pin)) {
            *p = value;
        } else if (auto* p = std::get_if<hal_pin<int32_t>>(&pin)) {
            *p = value;
        } else if (auto* p = std::get_if<hal_pin<uint32_t>>(&pin)) {
            *p = value;
        }
    }

    void ready(){
        hal_ready(comp_id);
    }
    void exit(){
        if(comp_id > 0)
            hal_exit(comp_id);
        comp_id = -1;
    }

    template<typename T>
    void add_pin(std::string pin_name, hal_dir dir, hal_pin<T> &pin){
        pin.ptr = (volatile T**)hal_malloc(8);
        if(!pin.ptr){
            error -= 1;
            rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_malloc() failed\n", pin_name.c_str());
            hal_exit(comp_id);
        }
        error += add_pin_(comp_name + "." + pin_name, dir, pin);
        if(error < 0){
            rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_pin_new() failed\n", pin_name.c_str());
            hal_exit(comp_id);
        }
    }

    ~hal_comp(){
        exit();
    }
};

#endif
