
#include <sapi/var.hpp>
#include <sapi/chrono.hpp>
#include <sapi/sys.hpp>

#include "ModbusRtu.hpp"

using namespace mbus;

ModbusRtu::ModbusRtu(){
	m_packet_spacing_timer.start();
}


int ModbusRtu::send(const var::Data & data){
	Data packet(data);

	packet_spacing().wait();
	packet_spacing().wait();


	u16 crc = calculate_crc(packet);
	packet << crc;

	int bytes_written = write(packet);

	if( bytes_written != (int)packet.size() ){
		return -1;
	}

	return 0;
}

var::Data ModbusRtu::receive(){
	//read data
	int bytes_read;
	var::Data data( Data::minimum_size() );

	bytes_read = read(data);

	if( bytes_read > 0 ){
		buffer() << data;
		m_packet_spacing_timer.restart();
	} else if( (bytes_read <= 0) && (m_packet_spacing_timer.microseconds() > packet_spacing().microseconds()*2) && (buffer().size() > 0) ){
		if( buffer().size() > 2 ){
			//crc should match the last 2 bytes of data
			u16 crc_check = buffer().at_u8(buffer().size()-1) << 8 | buffer().at_u8(buffer().size()-2);
			buffer().set_size(buffer().size() - 2);

			//verify the checksum
			u16 crc = calculate_crc(buffer());
			if( crc_check == crc ){
				var::Data result = buffer();
				buffer().free();
				return result;
			}
		} else {
			buffer().set_size(0);
		}
	}

	return var::Data();
}

