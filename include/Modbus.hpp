#ifndef MODBUS_MODBUS_HPP
#define MODBUS_MODBUS_HPP

#include <mcu/types.h>
#include <sapi/sys/Thread.hpp>
#include <sapi/var/Data.hpp>
#include <sapi/chrono/MicroTime.hpp>

namespace mbus {

class ModbusPhy {
public:
    virtual int initialize(){ return 0; } //success if not implemented
    virtual int finalize(){ return 0; } //success if not implemented
    virtual int send(const var::Data & data) = 0;
    virtual int receive(var::Data & data) = 0;

    u8 calculate_lrc(const var::Data & data);
    u16 calculate_crc(const var::Data & data);
};


class Modbus {
public:
    Modbus(ModbusPhy & phy);

    enum {
        NONE = 0,
        ILLEGAL_FUNCTION = 1,
        ILLEGAL_DATA_ADDRESS = 2,
        ILLEGAL_DATA_VALUE = 3,
        SLAVE_DEVICE_FAILURE = 4,
        ACKNOWLEDGE = 5,
        SLAVE_DEVICE_BUSY = 6,
        NEGATIVE_ACKNOWLEDGE = 7,
        MEMORY_PARITY_ERROR = 8
    };

    enum function_code {
        READ_COIL_STATUS = 0x01,
        READ_INPUT_STATUS = 0x02,
        READ_HOLDING_REGISTERS = 0x03,
        READ_INPUT_REGISTERS = 0x04,
        FORCE_SINGLE_COIL = 0x05,
        PRESET_SINGLE_REGISTER = 0x06,
        READ_EXCEPTION_STATUS = 0x07,
        DIAGNOSTICS = 0x08,
        PROGRAM_484 = 0x09,
        POLL_484 = 0x10,
        FETCH_COMMUNICATION_EVENT_CONTROLLER = 0x11,
        FETCH_COMMUNICATION_EVENT_LOG = 0x12,
        PROGRAM_CONTROLLER = 0x13,
        POLL_CONTROLLER = 0x14,
        FORCE_MULTIPLE_COILS = 0x15,
        PRESET_MULTIPLE_REGISTERS = 0x16,
        REPORT_SLAVE_ID = 0x17,
        PROGRAM_884_M84 = 0x18,
        RESET_COMMUNICATIONS_LINK = 0x19,
        READ_GENERAL_REFERENCE = 0x20,
        WRITE_GENERAL_REFERENCE = 0x21
    };

    int send_read_holding_registers_query(u8 slave_address, u16 register_address, u16 number_of_points);
    int send_read_holding_registers_response(u8 slave_address, const var::Data & data);

    int send_preset_single_register_query(u8 slave_address, u16 register_address, u16 value);
    int send_preset_single_register_response(u8 slave_address, u16 register_address, u16 value);
    int send_exception_response(u8 slave_address, u8 function_code);

    void set_exception_code(u16 exception_code){
        m_exception_code = exception_code;
    }

    u8 exception_code() const { return m_exception_code; }

    ModbusPhy & phy(){ return m_phy; }


private:
    int send_query(u8 slave_address, enum function_code function_code, const var::Data & data);
    int send_response(u8 slave_address, enum function_code function_code, const var::Data & data);

    ModbusPhy & m_phy;
    u16 m_exception_code;



};

class ModbusSlave : public Modbus {
public:
    ModbusSlave(ModbusPhy & phy, int stack_size = 1024) : Modbus(phy), m_thread(stack_size){}

    int initialize();
    void finalize(){
        m_is_running = false;
        m_thread.wait();
    }

    void set_slave_address(u8 slave_address){
        m_slave_address = slave_address;
    }

    virtual int preset_single_register(u16 register_address, u16 value){
        set_exception_code(ILLEGAL_FUNCTION);
        return -1;
    }

    virtual var::Data read_holding_registers(u16 register_address, u16 size){
        set_exception_code(ILLEGAL_FUNCTION);
        return -1;
    }

    void set_polling_interval(const chrono::MicroTime & interval){
        m_interval = interval;
    }


private:
    volatile bool m_is_running;
    u8 m_slave_address;
    sys::Thread m_thread;
    static void * listen_worker(void * args){
        ModbusSlave * object = (ModbusSlave*)args;
        return object->listen();
    }

    void * listen();
    chrono::MicroTime m_interval;
};

}

#endif // MODBUS_MODBUS_HPP
