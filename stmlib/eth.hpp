/*
 * eth.hpp
 *
 *  Created on: 14.12.2014
 *      Author: Michael
 */

#ifndef ETH_HPP_
#define ETH_HPP_

#include <stmlib/bits.hpp>

namespace eth {

	void start_phy();
	void start_rmii();

	void phy_write_register( uint8 regnum, uint8 address, uint16 value );
	void phy_write_register( uint8 regnum, uint8 address, bit::masked_value<uint16> value );

	uint16 phy_read_register( uint8 regnum, uint8 address );

	void phy_read_register_start( uint8 regnum, uint8 address );
	uint16 phy_read_register_finish();

	void ptp_initialize();
	void ptp_uninitialize();

	void ptp_init_time(int32 seconds, int32 subseconds);
	void ptp_update_time(uint32 seconds, uint32 subseconds);
	void ptp_start_timestamp();
	void ptp_stop_timestamp();

	void ptp_set_subsecond_increment(uint8 increment);
	void ptp_set_addend(uint32 addend);
	void ptp_discipline(int32 ppb);

	uint32 ptp_subseconds_to_nanos(uint32 subs);
	uint32 ptp_nanos_to_subseconds(uint32 nanos);

	uint64 ptp_hardware_to_logical( uint64 hardware );
	uint64 ptp_logical_to_hardware( uint64 logical );


	void ptp_get_time(uint32& seconds, uint32& subseconds);
}



#endif /* ETH_HPP_ */
