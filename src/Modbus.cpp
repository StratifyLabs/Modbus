#include <sapi/var.hpp>
#include "Modbus.hpp"

using namespace mbus;


Modbus::Modbus(ModbusPhy & phy) : m_phy(phy){
	m_max_packet_size = var::Data::minimum_size();
}


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
		set_error_message("failed to allocate memory");
		return -1;
	}

	//set the slave address and function code
	packet.at_u8(0) = slave_address;
	packet.at_u8(1) = function_code;
	int result =  phy().send(packet);
	if( result < 0 ){
		set_error_message( String().format("phy error; %s", phy().error_message().to_char()) );
	}

	return result;
}

int Modbus::send_response(u8 slave_address, enum function_code function_code, const var::Data & data){
	//response is the same format as a query
	return send_query(slave_address, function_code, data);
}

var::Data ModbusMaster::read_holding_registers(u8 slave_address, u16 register_address, u16 number_of_points){
	var::Data result;

	if( send_read_holding_registers_query(slave_address, register_address, number_of_points) < 0 ){
		//send_read_holding_registers_query() sets the error message
		return result;
	}

	var::Data incoming_packet = wait_for_response();

	//expecting the header plus the data plus the crc

	if( incoming_packet.size() == 0 ){
		//failed to read holding registers
		set_error_message("no response from slave");
		return var::Data();
	}

	//parse incoming packet
	u8 device_address = incoming_packet.at_u8(0);
	u8 function_code = incoming_packet.at_u8(1);

	if( device_address != slave_address ){
		set_error_message(String().format("response device address did not match request (%X != %X)", slave_address, device_address));
		return var::Data();
	}

	if( function_code & 0x80 ){
		set_exception_code( incoming_packet.at_u8(2) );
	} else {
		if( incoming_packet.size() >= number_of_points*sizeof(u16) + 2 ){
			for(u32 i = 0; i < number_of_points; i++){
				result << incoming_packet.at_u16(i+1);
			}
			result.swap_byte_order(2);
		} else {
			return var::Data();
		}
	}

	return result;
}



int ModbusMaster::preset_single_register(u8 slave_address, u16 register_address, u16 value){
	if( send_preset_single_register_query(slave_address, register_address, value) < 0 ){
		return -1;
	}

	var::Data incoming_packet = wait_for_response();

	if( incoming_packet.size() == 0 ){
		return -1;
	}

	u8 device_address = incoming_packet.at_u8(0); //u16 0
	u8 function_code = incoming_packet.at_u8(0);

	if( device_address != slave_address ){
		return -1;
	}

	if( function_code & 0x80 ){
		//exception occurred
		set_exception_code( incoming_packet.at_u8(2) );
		return -1;
	}

	var::Data response;
	response << incoming_packet.at_u16(1);
	response.swap_byte_order(2);

	if( response.at_u16(0) != value ){
		//not the same as was sent
	}

	return 0;
}

var::Data ModbusMaster::wait_for_response(){
	chrono::Timer timer;
	timer.restart();
	var::Data incoming_packet;

	do {
		incoming_packet = phy().receive();
	} while( (incoming_packet.size() == 0) && (timer.microseconds() < m_timeout) );

	return incoming_packet;
}


int ModbusSlave::initialize(){

	if( phy().initialize() < 0 ){
		set_error_message(String().format("phy initialize; %s", phy().error_message().to_char()));
		return -1;
	}

	m_is_running = true;
	return m_thread.create(listen_worker, this);
}

void * ModbusSlave::listen(){

	var::Data incoming_packet( max_packet_size() );
	while( m_is_running ){

		incoming_packet = phy().receive();

		if( incoming_packet.size() > 0 ){

			//first byte is the slave address
			u8 device_address = incoming_packet.at_u8(0);
			u8 function_code = incoming_packet.at_u8(1); //second byte is the function
			var::Vector<u16> args(2);
			args.at(0) = incoming_packet.at_u16(1);
			args.at(1) = incoming_packet.at_u16(2);
			args.swap_byte_order(2);

			if( device_address == m_slave_address ){

				set_exception_code(NONE);
				switch(function_code){
					case PRESET_SINGLE_REGISTER:
						preset_single_register(args.at(0), args.at(1));
						if( exception_code() != NONE ){
							send_exception_response(device_address, function_code);
						} else {
							send_preset_single_register_response(device_address, args.at(0), args.at(1));
						}
						break;

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

		polling_interval().wait();

	}

	phy().finalize();
	return 0;

}

u8 ModbusPhy::calculate_lrc(const var::Data & data){
	u16 size = data.size();
	u8 lrc = 0;
	const u8 * message = data.to_u8();
	while(size--){
		lrc += *message++;
	}
	return (s8)lrc*-1;
}

u16 ModbusPhy::calculate_crc(const var::Data & data){
	u16 crc = 0xFFFF;
	const u8 * message = data.to_u8();

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


