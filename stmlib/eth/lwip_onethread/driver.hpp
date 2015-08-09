/*
 * eth_lwip_driver.hpp
 *
 *  Created on: 20.12.2014
 *      Author: Michael
 */

#ifndef ETH_LWIP_DRIVER_HPP_
#define ETH_LWIP_DRIVER_HPP_

#include <stmlib_config.hpp>
#ifdef STMLIB_LWIP_ONETHREAD

namespace eth { namespace lwip {

	void configure_phy();
	void start_mac_dma(uint16 phystats, struct netif& interface);
	void resume_rx();


} }

#endif

#endif /* ETH_LWIP_DRIVER_HPP_ */
