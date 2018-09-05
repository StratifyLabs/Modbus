
#include <sapi/var.hpp>
#include <sapi/chrono.hpp>

#include "ModbusRtu.hpp"

using namespace mbus;

ModbusRtu::ModbusRtu(){
    m_packet_spacing_timer.start();
}


int ModbusRtu::send(const var::Data & data){
    Data packet(data);

    while( m_packet_spacing_timer.microseconds() < packet_spacing()*2 ){
        Timer::wait_milliseconds(5);
    }

    u16 crc = calculate_crc(packet);
    packet << crc;

    int bytes_written = write(packet);

    if( bytes_written != (int)packet.size() ){
        return -1;
    }
    m_packet_spacing_timer.restart();
    return 0;
}

int ModbusRtu::receive(var::Data & data){
    //read data
    int bytes_read;

    bytes_read = read(data);
    if( bytes_read > 2 ){
        data.set_size(bytes_read);

        //crc should match the last 2 bytes of data
        u16 crc_check = data.data_u8()[data.size()-1] << 8 | data.data_u8()[data.size()-2];
        data.set_size(data.size() - 2);

        //verify the checksum
        u16 crc = calculate_crc(data);
        if( crc_check == crc ){
            return data.size();
        }
    }

    return -1;

}

