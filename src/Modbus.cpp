#include <sapi/var.hpp>
#include "Modbus.hpp"

using namespace mbus;


Modbus::Modbus(ModbusPhy & phy) : m_phy(phy){}


int Modbus::send_read_holding_registers_query(u8 slave_address, u16 register_address, u16 number_of_points){
    Vector<u16> packet(2);
    packet.at(0) = register_address;
    packet.at(1) = number_of_points;
    packet.swap_byte_order(2); //convert to big endian
    return send_query(slave_address, READ_HOLDING_REGISTERS, packet);
}

int Modbus::send_read_holding_registers_response(u8 slave_address, const var::Data & data){
    Data packet = data; //make a copy
    packet.swap_byte_order(2); //convert to big endian
    return send_response(slave_address, READ_HOLDING_REGISTERS, packet);
}

int Modbus::send_preset_single_register_query(u8 slave_address, u16 register_address, u16 value){
    Vector<u16> packet(2);
    packet.at(0) = register_address;
    packet.at(1) = value;
    packet.swap_byte_order(2); //convert to big endian
    return send_query(slave_address, PRESET_SINGLE_REGISTER, packet);
}

int Modbus::send_preset_single_register_response(u8 slave_address, u16 register_address, u16 value){
    //response is identical to query
    return send_preset_single_register_query(slave_address, register_address, value);
}

int Modbus::send_exception_response(u8 slave_address, u8 function_code){
    var::Vector<u8> data(1);
    data.at(0) = exception_code();
    return send_response(slave_address, (enum function_code)(function_code | 0x80), data);
}


int Modbus::send_query(u8 slave_address, enum function_code function_code, const var::Data & data){
    Data packet;
    if( packet.copy_contents(data, 2, data.size()) < 0 ){
        //memory allocation error
    }

    //set the slave address and function code
    packet.cdata()[0] = slave_address;
    packet.cdata()[1] = function_code;
    return phy().send(packet);
}

int Modbus::send_response(u8 slave_address, enum function_code function_code, const var::Data & data){
    //response is the same format as a query
    return send_query(slave_address, function_code, data);
}

int ModbusSlave::initialize(){
    return m_thread.create(listen_worker, this);
}

void * ModbusSlave::listen(){

    var::Data incoming_packet;
    while( m_is_running ){

        int bytes_read = phy().receive(incoming_packet);

        if( bytes_read > 0 ){

            //first byte is the slave address
            u8 device_address = incoming_packet.data_u8()[0];
            u8 function_code = incoming_packet.data_u8()[1]; //second byte is the function
            var::Vector<u16> args(2);
            args.at(0) = incoming_packet.data_u16()[1];
            args.at(1) = incoming_packet.data_u16()[2];
            args.swap_byte_order(2);

            set_exception_code(NONE);
            switch(function_code){
            case PRESET_SINGLE_REGISTER:
                preset_single_register(args.at(0), args.at(1));
                if( exception_code() != NONE ){
                    send_exception_response(device_address, function_code);
                } else {
                    send_preset_single_register_response(device_address, args.at(0), args.at(1));
                }


            case READ_HOLDING_REGISTERS:
                var::Data response = read_holding_registers(args.at(0), args.at(1));
                //check for an exception -- then send the response
                if( exception_code() != NONE ){
                    send_exception_response(device_address, function_code);
                } else {
                    send_read_holding_registers_response(device_address, response);
                }
                break;
            }
        }

    }



    return 0;

}

u8 ModbusPhy::calculate_lrc(const var::Data & data){
    u16 size = data.size();
    u8 lrc = 0;
    const u8 * message = data.data_u8();
    while(size--){
        lrc += *message++;
    }
    return (s8)lrc*-1;
}

u16 ModbusPhy::calculate_crc(const var::Data & data){
    u16 crc = 0xFFFF;
    const u8 * message = data.data_u8();

    for (u32 pos = 0; pos < data.size(); pos++) {
        crc ^= (u16)message[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}


