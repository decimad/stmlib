/*
 * eth_lwip_driver_util.hpp
 *
 *  Created on: 20.12.2014
 *      Author: Michael
 */

#ifndef ETH_LWIP_DRIVER_UTIL_HPP_
#define ETH_LWIP_DRIVER_UTIL_HPP_

#include <stmlib_config.hpp>

#ifndef STMLIB_LWIP_ONETHREAD

#include <stmlib/eth.hpp>
#include <stmlib/eth/dma_descriptors.hpp>
#include <stmlib/eth/lwip/custom_buffer.hpp>
#include <stmlib/eth_registers.hpp>
#include <type_traits>

struct netif;

namespace eth::lwip
{

    void resume_rx();

    // Although the RX-normal descriptor documentation states
    // that PTP-timestamp would be written into RDES2 and RDES3,
    // later sections state that you absolutely need to use
    // enhanced descriptors.

    // We're really relying on the custom buffers being stored
    // in pools in rx and tx, tx can handle
    // lwip-pbufs and pool-custom-buffers though.

    struct receive_descriptor : public eth::enhanced_rx_dma_descriptor
    {
        void arm(custom_buffer_ptr buffer)
        {
            buffer->reset_rx();
            buffer_ = std::move(buffer);
            rdes1 <<= eth::fields::rdes1::rbs1(buffer_->capacity());
            rdes2 = reinterpret_cast<uint32>(buffer_->data());
            rdes0 = eth::fields::rdes0::own(1);
            if (eth::device.dmachrdr.get() == reinterpret_cast<uint32>(this))
            {
                // the dma engine halted, try to resume
                resume_rx();
            }
        }

        void set_next(receive_descriptor *next)
        {
            rdes3 = reinterpret_cast<uint32>(next);
            rdes1 <<= eth::fields::rdes1::rch(1);
        }

        const custom_buffer_ptr &get_buffer()
        {
            return buffer_;
        }

        custom_buffer_ptr release_buffer()
        {
            return std::move(buffer_);
        }

        bool is_host()
        {
            return rdes0.field<eth::fields::rdes0::own>() == 0;
        }

        bool is_first()
        {
            return rdes0.field<eth::fields::rdes0::fs>() == 1;
        }

        bool is_last()
        {
            return rdes0.field<eth::fields::rdes0::ls>() == 1;
        }

        receive_descriptor *get_next()
        {
            return reinterpret_cast<receive_descriptor *>(rdes3.get());
        }

        void transfer_timestamp()
        {
            if (rdes4.field<eth::fields::rdes4::pmt>() != 0)
            {
                const uint64 timestamp = (static_cast<uint64>(rdes7.get()) << 32) | rdes6.get();
                buffer_->enhance().to_type<uint64>(ptp_hardware_to_logical(timestamp));
            }
        }

        custom_buffer_ptr buffer_;
    };

    receive_descriptor *receive_init();

    void rx_walk_descriptors(struct ::netif *interface);
    void tx_walk_descriptors();

    //
    // TX
    //

    struct transmit_descriptor : public eth::enhanced_tx_dma_descriptor
    {
        transmit_descriptor() : buffer_(0)
        {
        }

        // First must be valid! (Otherwise call clear!)
        void set_buffer(pbuf *buf)
        {
            uint32 buf1_size = buf->len;
            buffer_ = buf;
            tdes2 = reinterpret_cast<uint32>(buf->payload);
            tdes1 = eth::fields::tdes1::tbs1(buf1_size) | eth::fields::tdes1::tbs2(0);
        }

        pbuf *get_buffer()
        {
            return buffer_;
        }

        void set_next(transmit_descriptor *next)
        {
            tdes3 = reinterpret_cast<uint32>(next);
            tdes0 <<= eth::fields::tdes0::tch(1);
        }

        transmit_descriptor *get_next()
        {
            return reinterpret_cast<transmit_descriptor *>(tdes3.get());
        }

        void clear()
        {
            buffer_ = 0;
            tdes1 = 0;
        }

        bool is_host()
        {
            return tdes0.field<eth::fields::tdes0::own>() == 0;
        }

        uint64 time() const
        {
            return ptp_hardware_to_logical((static_cast<uint64>(tdes7.get()) << 32) | tdes6.get());
        }

        //
        // Note: the handling for custom buffers
        //       currently relies on them not being chained!
        bool is_custom_buffer()
        {
            return (buffer_->flags & PBUF_FLAG_IS_CUSTOM) != 0;
        }

        custom_buffer_ptr release_custom()
        {
            auto result = ptr_from_pbuf(buffer_);
            buffer_ = nullptr;
            return result;
        }

        // could be a custom buffer
        pbuf *buffer_;
    };

    transmit_descriptor *transmit_init();
    msg_t transmit(pbuf *buf);

} // namespace eth::lwip

#endif

#endif /* ETH_LWIP_DRIVER_UTIL_HPP_ */
