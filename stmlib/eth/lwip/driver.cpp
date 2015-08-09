/*
 * eth_lwip.cpp
 *
 *  Created on: 17.12.2014
 *      Author: Michael
 */
#include <stmlib_config.hpp>

#ifndef STMLIB_LWIP_ONETHREAD

#include <stmlib/stmtypes.hpp>
#include <stmlib/eth/dma_descriptors.hpp>
#include <stmlib/eth/lwip/descriptors.hpp>

#include <stmlib/nvic.hpp>
#include <stmlib/eth.hpp>
#include <stmlib/eth_registers.hpp>
#include <thread.hpp>

#include <stmlib/gpio.hpp>
#include <stmlib/rcc.hpp>

#include <lwip/def.h>
#include <lwip/sys.h>
#include <lwip/tcpip.h>
#include <lwip/icmp.h>
#include <lwip/udp.h>
#include <lwip/opt.h>
#include <lwip/err.h>
#include <netif/etharp.h>
#include <lwip/dhcp.h>
#include <lwip/pbuf.h>
#include <lwip/igmp.h>

#include <evtimer.h>

#include <algorithm>
#include <stmlib/trace.h>

// FIXME
#include <microptp/ports/cortex_m4/port.hpp>

namespace eth { namespace lwip {

	namespace phy {

		namespace bmcr {
			using Reset = bit::field<15,15,uint16>;
			using Loopback = bit::field<14,14,uint16>;
			using SpeedSelect = bit::field<13,13,uint16>;
			using AutoEnable = bit::field<12,12,uint16>;
			using PowerDown = bit::field<11,11,uint16>;
			using Isolate = bit::field<10,10,uint16>;
			using RestartAuto = bit::field<9,9,uint16>;
			using Duplex = bit::field<8,8,uint16>;
			using CollTest = bit::field<7,7,uint16>;
		}

	}

	void configure_phy() {
		using namespace phy;
		eth::phy_write_register( 0, 1, bmcr::Reset(0) | bmcr::Loopback(0) | bmcr::AutoEnable(1) | bmcr::Isolate(0) | bmcr::RestartAuto(1) | bmcr::SpeedSelect(1) | bmcr::Duplex(1) );
	}

	volatile int i = 0;

	void start_mac_dma(uint16 phystats, struct netif& interface) {
		auto* first_armed = receive_init();
		auto* tx_front = transmit_init();

		eth::start_rmii();

		// Warning!
		// Make sure to combine register writes into
		// one operation or delay successive writes
		// See F407 Errata sheet

		// RM page 1146 describes the basic setup procedure
		using namespace eth::fields;

		// Wait for DMA device
		while(eth::device.dmabmr.field<dmabmr::sr>());

		eth::ptp_initialize();

		// Setup DMA bus behaviour
		eth::device.dmabmr =
				dmabmr::mb(0) |
				dmabmr::aab(1) |
				dmabmr::fpm(0) |
				dmabmr::usp(0) |
				dmabmr::rdp(4) |
				dmabmr::fb(1) |
				dmabmr::pm(0) |
				dmabmr::pbl(4) |
				dmabmr::edfe(1) |	// enhanced descriptors for ptp timestamps
				dmabmr::dsl(0) |
				dmabmr::da(0) |
				dmabmr::sr(0);

		// Set DMA RX descriptor address
		eth::device.dmardlar =
				reinterpret_cast<uint32>(first_armed);

		eth::device.dmatdlar =
				reinterpret_cast<uint32>(tx_front);

		eth::device.maca0hr = eth::fields::maca0hr::mo(true) | eth::fields::maca0hr::maca0h((interface.hwaddr[5]<<8)|interface.hwaddr[4]);
		eth::device.maca0lr = (interface.hwaddr[3]<<24) | (interface.hwaddr[2]<<16) | (interface.hwaddr[1]<<8) | (interface.hwaddr[0]<<0);

		eth::device.macffr <<=
			eth::fields::macffr::hpf(1) |
			eth::fields::macffr::pam(1);

		eth::device.dmaomr = dmaomr::ftf(1);
		while(eth::device.dmaomr.field<dmaomr::ftf>());

		for(i=0;i<2048;++i);

		// Setup DMA and start RX state machine
		eth::device.dmaomr =
				dmaomr::dtcefd(0) |
				dmaomr::rsf(1) |
				dmaomr::dfrf(0) |
				dmaomr::tsf(1) |
				dmaomr::ttc(0) |
				dmaomr::fef(1) | // spï¿½ter auf 0
				dmaomr::fugf(0) |
				dmaomr::rtc(0) |
				dmaomr::osf(0) |
				dmaomr::st(1) |
				dmaomr::sr(1);

		// Setup irq and masking
		nvic::set_priority(nvic::IRQ::ETH, 255);
		nvic::clear_pending(nvic::IRQ::ETH);
		nvic::enable(nvic::IRQ::ETH);
		eth::device.dmaier <<=  dmaier::nise(1) | dmaier::rie(1) | dmaier::tie(1);

		// Start receiving
		eth::device.maccr  <<=
				maccr::dm((phystats>>2)&1) |
				maccr::fes(((phystats^2)>>1)&1) |
				maccr::ipco(1) |
				maccr::re(1) |
				maccr::te(1);
	}

	void resume_rx() {
		eth::device.dmasr <<= eth::fields::dmasr::nis(1) | eth::fields::dmasr::rbus(1);
		eth::device.dmarpdr = 1;
	}

	namespace lwip_funcs {

		// output-funktion für LwIP
		err_t low_level_output(struct netif *netif, struct pbuf *p) {
			return transmit(p);
		}

		err_t ethernetif_init(struct netif* netif) {
			netif->output = etharp_output;
			netif->name[0] = 'e';
			netif->name[1] = 'n';
			netif->mtu = 1500;
			netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;

			netif->output = etharp_output;
			netif->linkoutput = low_level_output;

			eth::start_phy();
			configure_phy();

			return ERR_OK;
		}

	}

	namespace pins {

		using red = gpio::ph<2>;
		using yellow = gpio::ph<3>;
		using green = gpio::ph<4>;
		using button = gpio::ph<5>;

	}


	const uint32 RxAvailableMask = 0x1;
	const uint32 TxCompletedMask = 0x2;

	const uint8 mac_addr[6] = {
			0x00, 0xFA, 0x5F, 0x48, 0x59, 0x66
	};

	static struct netif interface;

	struct ip_addr {
		ip_addr(uint32 a, uint32 b, uint32 c, uint32 d)
			: addr{ (a << 0) | (b << 8) | (c << 16) | (d<<24) }
		{
		}

		ip_addr_t addr;
	};

	ip_addr ip(0,0,0,0);
	ip_addr netmask(255,255,255,0);
	ip_addr gateway(0,0,0,0);

	uptp::Config cfg;
	uptp::SystemPort ptp_port(cfg);

	msg_t LwIpThreadFunc(void* arg)
	{
		(void) arg;
		bool started_phy_read = false;
		bool link_status = false;

		rcc::enable_gpio<pins::red, pins::yellow, pins::green, pins::button>();
		gpio::configure<pins::red, pins::yellow, pins::green>(gpio::output());
		gpio::configure<pins::button>(gpio::input, gpio::pull_up());

		netif_init();
		etharp_init();
		udp_init();
		tcpip_init(NULL,NULL);

		std::copy(mac_addr, mac_addr+6, interface.hwaddr);
		std::copy(mac_addr, mac_addr+6, cfg.mac_addr.begin());

		interface.hwaddr_len = 6;

		netif_add(&interface, &ip.addr, &netmask.addr, &gateway.addr, NULL, lwip_funcs::ethernetif_init, ::ethernet_input );
		netif_set_default(&interface);
		netif_set_up(&interface);

		bool dhcp_bound = false;

		unsigned int history = 0;
		bool enabled = false;

		while(true) {
			auto eventmask = chEvtWaitAnyTimeout( RxAvailableMask | TxCompletedMask, 100 );

			if( eventmask & RxAvailableMask ) {
				rx_walk_descriptors(&interface);
			}

			if( eventmask & TxCompletedMask ) {
				tx_walk_descriptors();
			}


			unsigned int val = pins::button::get();
			history = (history << 1) | val;

			if((history&0xF) == 12 /*~3&0xF*/ ) {
				if(enabled) {
					ptp_port.post_thread_command(uptp::SystemPort::ThreadCommands::DisableClock);
					pins::green::reset();
				} else {
					ptp_port.post_thread_command(uptp::SystemPort::ThreadCommands::EnableClock);
					pins::green::set();
				}

				enabled = !enabled;
			}

			if(started_phy_read) {
				auto phystats = eth::phy_read_register_finish();
				if(!link_status && (phystats & 1)) {
					start_mac_dma(phystats, interface);
					netif_set_link_up(&interface);
					dhcp_start(&interface);

					ptp_port.start();
					link_status = true;
				} else if(link_status && !(phystats&1)){
					//link_status = false;
					//netif_set_link_down(&interface);
				}

				started_phy_read = false;
			} else {
				eth::phy_read_register_start(0x10, 1);
				started_phy_read = true;
			}

			if(link_status && interface.dhcp->state == DHCP_STATE_BOUND && !dhcp_bound) {
				ptp_port.ip_addr_changed(interface.ip_addr);
				dhcp_bound = true;
			}
		}

		return 0;
	}

	util::static_thread< LwIpThreadFunc, 2048 > LwIpThread;

	void start() {
		LwIpThread.run();
	}

} }

template<>
void isr<nvic::IRQ::ETH>()
{
	CH_IRQ_PROLOGUE();

	auto dma_status = eth::device.dmasr.snapshot();

	if(dma_status.field<eth::fields::dmasr::rs>()) {
		// At least 1 frame was received
		chSysLockFromIsr();
		eth::device.dmasr <<= eth::fields::dmasr::nis(1) | eth::fields::dmasr::rs(1);	// clear rs bit
		eth::lwip::LwIpThread.set_event_i(eth::lwip::RxAvailableMask);
		chSysUnlockFromIsr();
	}

	if(dma_status.field<eth::fields::dmasr::ts>()) {
		chSysLockFromIsr();
		eth::device.dmasr <<= eth::fields::dmasr::nis(1) | eth::fields::dmasr::ts(1);	// clear ts bit
		eth::lwip::LwIpThread.set_event_i(eth::lwip::TxCompletedMask);
		chSysUnlockFromIsr();
	}

	CH_IRQ_EPILOGUE();
}

#endif
