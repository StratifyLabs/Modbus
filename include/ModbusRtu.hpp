#ifndef MODBUS_MODBUSRTU_HPP
#define MODBUS_MODBUSRTU_HPP

#include <sapi/chrono/Timer.hpp>
#include "Modbus.hpp"

namespace mbus {

class ModbusRtu : public ModbusPhy {
public:
    ModbusRtu();


private:
    int send(const var::Data & data);
    int receive(var::Data & data);

    virtual int write(const var::Data & data) = 0;
    virtual int read(var::Data & data) = 0;
    virtual int bitrate() const = 0;

    chrono::Timer m_packet_spacing_timer;

    chrono::MicroTime packet_spacing() const {
        return chrono::MicroTime(1000000 * 10 / bitrate() * 4);
    }



};

}

#endif // MODBUS_MODBUSRTU_HPP
