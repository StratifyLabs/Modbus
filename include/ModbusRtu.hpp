#ifndef MODBUS_MODBUSRTU_HPP
#define MODBUS_MODBUSRTU_HPP

#include <sapi/chrono/Timer.hpp>
#include "Modbus.hpp"

namespace mbus {

class ModbusRtu : public ModbusPhy {
public:
    ModbusRtu();

protected:
    chrono::MicroTime packet_spacing() const {
        return chrono::MicroTime(1000000 * 10 / bitrate() * 4);
    }

private:
    int send(const var::Data & data);
    var::Data receive();

    virtual int write(const var::Data & data) = 0;
    virtual int read(var::Data & data) = 0;
    virtual int bitrate() const = 0;

    chrono::Timer m_packet_spacing_timer;





};

}

#endif // MODBUS_MODBUSRTU_HPP
