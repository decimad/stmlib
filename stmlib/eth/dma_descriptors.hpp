/*
 * eth_dma.hpp
 *
 *  Created on: 16.12.2014
 *      Author: Michael
 */

#ifndef ETH_DMA_HPP_
#define ETH_DMA_HPP_

#include <stmlib/bits.hpp>

namespace eth
{

    namespace fields
    {

        // page 1154
        namespace tdes0
        {
            // Own bit
            using own = bit::field<31>;
            // Interrupt on completion
            using ic = bit::field<30>;
            // Last segment
            using ls = bit::field<29>;
            // First segment
            using fs = bit::field<28>;
            // Disable CRC
            using dc = bit::field<27>;
            // Disable pad
            using dp = bit::field<26>;
            // Transmit time stamp enable
            using ttse = bit::field<25>;
            // Checksum insertion control
            using cic = bit::field<23, 22>;
            // Transmit end of ring
            using ter = bit::field<21>;
            // Second address chained
            using tch = bit::field<20>;
            // Transmit stamp status
            using ttss = bit::field<17>;
            // IP header error
            using ihe = bit::field<16>;
            // Error summary
            using es = bit::field<15>;
            // Jabber timeout
            using jt = bit::field<14>;
            // Frame flushed
            using ff = bit::field<13>;
            // IP payload error
            using ipe = bit::field<12>;
            // Loss of carrier
            using lca = bit::field<11>;
            // No carrier
            using nc = bit::field<10>;
            // Late collision
            using lco = bit::field<9>;
            // Excessive collision
            using ec = bit::field<8>;
            // VLAN frame
            using vf = bit::field<7>;
            // Collision count
            using cc = bit::field<6, 3>;
            // Excessive deferral
            using ed = bit::field<2>;
            // Undeflow error
            using uf = bit::field<1>;
            // Deferred bit
            using db = bit::field<0>;
        } // namespace tdes0

        // page 1156
        namespace tdes1
        {
            // Transmit buffer 2 size
            using tbs2 = bit::field<28, 16>;
            // Transmit buffer 1 size
            using tbs1 = bit::field<12, 0>;
        } // namespace tdes1

        // page 1157
        namespace tdes2
        {
            // Transmit buffer 1 address pointer / Transmit frame time stamp low
            using tbap1 = bit::field<31, 0>;
        }; // namespace tdes2

        // page 1157
        namespace tdes3
        {
            // Transmit buffer 2 address pointer / next descriptor address / Transmit frame
            // time stamp high
            using tbap2 = bit::field<31, 0>;
        }; // namespace tdes3

        namespace tdes4
        {
        };

        namespace tdes5
        {
        };

        namespace tdes6
        {
            // Transmit frame time stamp low
            using ttsl = bit::field<31, 0>;
        }; // namespace tdes6

        namespace tdes7
        {
            // Transmit frame time stamp high
            using ttsh = bit::field<31, 0>;
        }; // namespace tdes7

        namespace rdes0
        {
            // Own bit
            using own = bit::field<31>;
            // Destination address filter fail
            using afm = bit::field<30>;
            // Frame length (only set by dma in last descriptor of a frame, check ls field)
            using fl = bit::field<29, 16>;
            // Error summary
            using es = bit::field<15>;
            // Descriptor error
            using de = bit::field<14>;
            // Source address filter fail
            using saf = bit::field<13>;
            // Length error
            using le = bit::field<12>;
            // Overflow error
            using oe = bit::field<11>;
            // VLAN tag
            using vlan = bit::field<10>;
            // First descriptor
            using fs = bit::field<9>;
            // Last descriptor
            using ls = bit::field<8>;
            // IPv header checksum error / time stamp invalid
            using iphce = bit::field<7>;
            using tsv = bit::field<7>;
            // Late collision
            using lco = bit::field<6>;
            // Frame type
            using ft = bit::field<5>;
            // Receice watchdog timer
            using rwt = bit::field<4>;
            // Receive error
            using re = bit::field<3>;
            // Dribble bit error
            using dbe = bit::field<2>;
            // CRC error
            using ce = bit::field<1>;
            // Payload checksum error / extended status available
            using pce = bit::field<0>;
            using esa = bit::field<0>;
        } // namespace rdes0

        namespace rdes1
        {
            // Disable interrupt on completion
            using dic = bit::field<31>;
            // Receive buffer 2 size
            using rbs2 = bit::field<28, 16>;
            // Receive end of ring
            using rer = bit::field<15>;
            // Second address chained
            using rch = bit::field<14>;
            // Receive buffer 1 size
            using rbs1 = bit::field<12, 0>;
        } // namespace rdes1

        namespace rdes2
        {
            // Receive buffer 1 address pointer / Receive frame time stamp low
            using rbap1 = bit::field<31, 0>;
            using rtsl = bit::field<31, 0>;
        } // namespace rdes2

        namespace rdes3
        {
            // Receive buffer 2 address pointer (next desc) / Receive frame time stamp high
            using rbap2 = bit::field<31, 0>;
            using rtsh = bit::field<31, 0>;
        } // namespace rdes3

        namespace rdes4
        {
            // PTP version
            using pv = bit::field<13>;
            // PTP frame type
            using pft = bit::field<12>;
            // PTP message type
            using pmt = bit::field<11, 8>;
            // IPv6 packet received
            using ipv6pr = bit::field<7>;
            // IPv4 packet received
            using ipv4r = bit::field<6>;
            // IP checksum bypassed
            using ipcb = bit::field<5>;
            // IP payload error
            using pppe = bit::field<4>;
            // IP header error
            using iphe = bit::field<3>;
            // IP payload type
            using ippt = bit::field<2, 0>;
        } // namespace rdes4

        namespace rdes5
        {
            // reserved
        }

        namespace rdes6
        {
            // Receive frame time stamp low
            using rtsl = bit::field<31, 0>;
        } // namespace rdes6

        namespace rdes7
        {
            // Receive frame time stamp high
            using rtsh = bit::field<31, 0>;
        } // namespace rdes7

    } // namespace fields

    struct tx_dma_descriptor
    {
        bit::register_base tdes0;
        bit::basic_register<uint32> tdes1;
        bit::basic_register<uint32> tdes2;
        bit::basic_register<uint32> tdes3;
    };

    struct enhanced_tx_dma_descriptor : public tx_dma_descriptor
    {
        bit::basic_register<uint32> tdes4;
        bit::basic_register<uint32> tdes5;
        bit::basic_register<uint32> tdes6;
        bit::basic_register<uint32> tdes7;
    };

    // When you have multiple descriptors contiguous (or by a configurable offset,
    // field dsl in register dmabmr) in memory, you don't have to use the second
    // address as a chain-next link so you can use 2 data buffers per descriptor.
    // (This is called implicit chaining). In this case the last descriptor has to
    // set the "receive end of ring, rer" in rdes1.
    struct rx_dma_descriptor
    {
        bit::register_base rdes0;
        bit::basic_register<uint32> rdes1;
        bit::basic_register<uint32> rdes2;
        bit::basic_register<uint32> rdes3;
    };

    struct enhanced_rx_dma_descriptor : public rx_dma_descriptor
    {
        bit::basic_register<uint32> rdes4;
        bit::basic_register<uint32> rdes5;
        bit::basic_register<uint32> rdes6;
        bit::basic_register<uint32> rdes7;
    };

    template <typename EmbeddedType>
    struct EmbeddingTxDmaFrame : public tx_dma_descriptor
    {
        EmbeddingTxDmaFrame()
        {
            void *pos = get_data_ptr(payload);
            uint16 size = get_data_size(payload);

            using namespace fields;

            this->tdes0 = tdes0::own(0) | tdes0::ic(0) | tdes0::ls(0) | tdes0::fs(0) | tdes0::dc(0) | tdes0::dp(0) | tdes0::ttse(0)
                          | tdes0::cic(3) | tdes0::ter(0) | tdes0::tch(0) | tdes0::ttss(0) | tdes0::ihe(0) | tdes0::es(0) | tdes0::jt(0)
                          | tdes0::ff(0) | tdes0::ipe(0) | tdes0::lca(0) | tdes0::nc(0) | tdes0::lco(0) | tdes0::ec(0) | tdes0::vf(0)
                          | tdes0::cc(0) | tdes0::ed(0) | tdes0::uf(0) | tdes0::db(0);

            this->tdes1 <<= tdes1::tbs2(0) | tdes1::tbs1(size);
            this->tdes2 = reinterpret_cast<uint32>(pos);
            this->tdes3 = 0;
        }

        EmbeddedType payload;
    };

    template <typename EmbeddedType, unsigned int Size>
    struct EmbeddingTxDmaRing
    {
        using FrameType = EmbeddingTxDmaFrame<EmbeddedType>;

        EmbeddingTxDmaRing() : front_(&frames_[0]), start_(&frames_[0])
        {
            for (uint32 i = 0; i < Size - 1; ++i)
            {
                frames_[i].set_next(frames_[i + 1]);
            }
            frames_[Size - 1].set_next(frames_[0]);
        }

        EmbeddedType *try_acquire()
        {
            return 0;
        }

        EmbeddedType *acquire();

        void release_frame(EmbeddedType *last)
        {
            // last->set_last();
        }

        FrameType *start_;
        FrameType *front_;
        FrameType frames_[Size];
    };

} // namespace eth

#endif /* ETH_DMA_HPP_ */
