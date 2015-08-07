/*
 * trace_registers.hpp
 *
 *  Created on: 26.12.2014
 *      Author: Michael
 */

#ifndef INCLUDE_STM_ITM_REGISTERS_HPP_
#define INCLUDE_STM_ITM_REGISTERS_HPP_

#include <stmlib/bits.hpp>

namespace itm {

	namespace fields {

		namespace tcr {

			using busy = bit::field<23>;
			using traceid = bit::field<22,16>;
			using prescale = bit::field<9,8>;
			using dwt_enable = bit::field<3>;
			using sync_enable = bit::field<2>;
			using stamp_enable = bit::field<1>;
			using enable = bit::field<0>;

		}

	}

	struct register_map {
		// 32 Stimulus ports
		bit::register_base port[32];
											uint32 reserved0[864];
		// ITM Trace Enable register
		bit::register_base ter;
											uint32 reserved1[15];
		// ITM Trace Priviledge Register
		bit::register_base tpr;
											uint32 reserved2[15];
		// ITM Trace Control Register
		bit::register_base tcr;
											uint32 reserved3[29];
		// ITM Integration Write Register
		bit::register_base iwr;
		// ITM Integration Read Register
		bit::register_base irr;
		// ITM Integration Mode Control Register
		bit::register_base imcr;
											uint32 reserved4[43];
		// ITM Lock Access Register
		bit::register_base lar;
		// ITM Lock Status Register
		bit::register_base lsr;
											uint32 reserved5[6];
		bit::register_base pid4;
		bit::register_base pid5;
		bit::register_base pid6;
		bit::register_base pid7;
		bit::register_base pid0;
		bit::register_base pid1;
		bit::register_base pid2;
		bit::register_base pid3;
		bit::register_base cid0;
		bit::register_base cid1;
		bit::register_base cid2;
		bit::register_base cid3;

	};

	extern "C" register_map __itm__device_0xe0000000;
	static auto& device = __itm__device_0xe0000000;

}



#endif /* INCLUDE_STM_ITM_REGISTERS_HPP_ */
