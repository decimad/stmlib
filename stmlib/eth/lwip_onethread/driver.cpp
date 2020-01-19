/*
 * eth_lwip.cpp
 *
 *  Created on: 17.12.2014
 *      Author: Michael
 */

#include <stmlib_config.hpp>
#ifdef STMLIB_LWIP_ONETHREAD

#include <stmlib/eth/dma_descriptors.hpp>
#include <stmlib/eth/lwip_onethread/dma.hpp>
#include <stmlib/stmtypes.hpp>

#include <lwip/netif.h>
#include <stmlib/eth.hpp>
#include <stmlib/eth_registers.hpp>
#include <stmlib/nvic.hpp>

#include <algorithm>
#include <stmlib/trace.h>

namespace eth::lwip
{
    //	namespace phy {

    namespace bmcr
    {
        using Reset = bit::field<15, 15, uint16>;
        using Loopback = bit::field<14, 14, uint16>;
        using SpeedSelect = bit::field<13, 13, uint16>;
        using AutoEnable = bit::field<12, 12, uint16>;
        using PowerDown = bit::field<11, 11, uint16>;
        using Isolate = bit::field<10, 10, uint16>;
        using RestartAuto = bit::field<9, 9, uint16>;
        using Duplex = bit::field<8, 8, uint16>;
        using CollTest = bit::field<7, 7, uint16>;
    } // namespace bmcr

    //	}

    void configure_phy()
    {
        // using namespace phy;

        eth::phy::write_register_blocking(0, 1, bmcr::Reset(1).value_);
        while ((eth::phy::read_register_blocking(0, 1) & bmcr::Reset(1).value_) != 0)
            ;
        eth::phy::write_register_blocking(0, 1,
                                          (bmcr::Reset(0) | bmcr::Loopback(0) | bmcr::AutoEnable(1) | bmcr::Isolate(0)
                                           | bmcr::RestartAuto(1) | bmcr::SpeedSelect(1) | bmcr::Duplex(1))
                                              .value_);

        //		eth::phy::write(0, 1, bmcr::Reset(0).value_, eth::phy::callback_type() );
        //		eth::phy::write(
        //			0, 1,
        //			(bmcr::Reset(0) | bmcr::Loopback(0) | bmcr::AutoEnable(1) | bmcr::Isolate(0) | bmcr::RestartAuto(1) |
        // bmcr::SpeedSelect(1) | bmcr::Duplex(1)).value_, 			eth::phy::callback_type()
        //		);

        //		while(eth::device.macmiiar.field<eth::fields::macmiiar::mb>());
        //		eth::phy::write_register( 0, 1, bmcr::Reset(0) | bmcr::Loopback(0) | bmcr::AutoEnable(1) | bmcr::Isolate(0) |
        // bmcr::RestartAuto(1) | bmcr::SpeedSelect(1) | bmcr::Duplex(1) );
        //		while(eth::device.macmiiar.field<eth::fields::macmiiar::mb>());
    }

    volatile int volatile_helper;

    void start_mac_dma(uint16 phystats, struct netif &interface)
    {
        eth::start_rmii();

        // Warning!
        // Make sure to combine register writes into
        // one operation or delay successive writes
        // See F407 Errata sheet

        // RM page 1146 describes the basic setup procedure
        using namespace eth::fields;

        // Wait for DMA device
        while (eth::device.dmabmr.field<dmabmr::sr>())
            ;

        eth::ptp_initialize();

        // Setup DMA bus behaviour
        eth::device.dmabmr = dmabmr::mb(0) | dmabmr::aab(1) | dmabmr::fpm(0) | dmabmr::usp(0) | dmabmr::rdp(4) | dmabmr::fb(1)
                             | dmabmr::pm(0) | dmabmr::pbl(4) | dmabmr::edfe(1) | // enhanced descriptors for ptp timestamps
                             dmabmr::dsl(0) | dmabmr::da(0) | dmabmr::sr(0);

        // Set DMA RX descriptor address
        dma_receive_init();
        dma_transmit_init();

        eth::device.maca0hr
            = eth::fields::maca0hr::mo(true) | eth::fields::maca0hr::maca0h((interface.hwaddr[5] << 8) | interface.hwaddr[4]);
        eth::device.maca0lr
            = (interface.hwaddr[3] << 24) | (interface.hwaddr[2] << 16) | (interface.hwaddr[1] << 8) | (interface.hwaddr[0] << 0);

        eth::device.macffr <<= eth::fields::macffr::hpf(1) | eth::fields::macffr::pam(1);

        eth::device.dmaomr = dmaomr::ftf(1);
        while (eth::device.dmaomr.field<dmaomr::ftf>())
            ;

        for (volatile_helper = 0; volatile_helper < 2048; ++volatile_helper)
            ;

        // Setup DMA and start RX state machine
        eth::device.dmaomr = dmaomr::dtcefd(0) | dmaomr::rsf(1) | dmaomr::dfrf(0) | dmaomr::tsf(1) | dmaomr::ttc(0) | dmaomr::fef(1)
                             | // später auf 0
                             dmaomr::fugf(0) | dmaomr::rtc(0) | dmaomr::osf(0) | dmaomr::st(1) | dmaomr::sr(1);

        // Setup irq and masking
        nvic::set_priority(nvic::IRQ::ETH, 255);
        nvic::clear_pending(nvic::IRQ::ETH);
        nvic::enable(nvic::IRQ::ETH);
        eth::device.dmaier <<= dmaier::nise(1) | dmaier::rie(1) | dmaier::tie(1);

        // Start receiving
        eth::device.maccr
            <<= maccr::dm((phystats >> 2) & 1) | maccr::fes(((phystats ^ 2) >> 1) & 1) | maccr::ipco(1) | maccr::re(1) | maccr::te(1);
    }

    void resume_rx()
    {
        eth::device.dmasr <<= eth::fields::dmasr::nis(1) | eth::fields::dmasr::rbus(1);
        eth::device.dmarpdr = 1;
    }

} // namespace eth::lwip

#endif
