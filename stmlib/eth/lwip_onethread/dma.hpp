/*
 * eth_lwip_driver_util.hpp
 *
 *  Created on: 20.12.2014
 *      Author: Michael
 */

#ifndef ETH_LWIP_DRIVER_UTIL_HPP_
#define ETH_LWIP_DRIVER_UTIL_HPP_

#include <stmlib_config.hpp>
#ifdef STMLIB_LWIP_ONETHREAD

#include <chtypes.h>
#include <stmlib/eth.hpp>
#include <stmlib/eth/dma_descriptors.hpp>
#include <stmlib/eth/lwip/custom_buffer.hpp>
#include <stmlib/eth_registers.hpp>
#include <type_traits>

struct netif;

namespace eth::lwip
{

    void resume_rx();
    void dma_receive_init();

    void rx_walk_descriptors(struct ::netif *interface);
    void tx_walk_descriptors();

    void dma_transmit_init();
    msg_t transmit(pbuf *buf);

} // namespace eth::lwip

#endif
#endif /* ETH_LWIP_DRIVER_UTIL_HPP_ */
