/*
 * eth_lwip_driver.hpp
 *
 *  Created on: 20.12.2014
 *      Author: Michael
 */

#ifndef ETH_LWIP_DRIVER_HPP_
#define ETH_LWIP_DRIVER_HPP_

#include <stmlib_config.hpp>

#ifndef STMLIB_LWIP_ONETHREAD

namespace eth::lwip
{

    void start();

} // namespace eth::lwip

#endif

#endif /* ETH_LWIP_DRIVER_HPP_ */
