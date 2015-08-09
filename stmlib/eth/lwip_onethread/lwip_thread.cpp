#include <stmlib_config.hpp>
#ifdef STMLIB_LWIP_ONETHREAD

#include <stmlib/eth/lwip_onethread/lwip_thread.hpp>
#include <stmlib/eth/lwip_onethread/driver.hpp>
#include <stmlib/eth/lwip_onethread/dma.hpp>
#include <stmlib/gpio.hpp>
#include <stmlib/nvic.hpp>
#include <stmlib/rcc.hpp>
#include <lwip/init.h>
#include <lwip/udp.h>
#include <lwip/timers.h>
#include <lwip/dhcp.h>
#include <lwip/mem.h>
#include <netif/etharp.h>
#include <cstdint>
#include <climits>

// Note: We're using LWIP NO_SYS mode with NO_SYS_CUSTOM_TIMERS
//       This way we only need to run the timers on lwip's behalf and that's all to it, one thread for all custom services and lwip.

namespace eth { namespace lwip {
	
	namespace pins {

		using red = gpio::ph<2>;
		using yellow = gpio::ph<3>;
		using green = gpio::ph<4>;
		using button = gpio::ph<5>;

	}

	LwipThread::LwipThread()
		: ptp_clock_(ptp_config_)
	{
	}

	err_t low_level_output(struct netif *netif, struct pbuf *p) {
		return transmit(p);
	}

	err_t LwipThread::ethernetif_init(netif* net)
	{	
		net->output     = etharp_output;
		net->linkoutput = low_level_output;
		net->name[0]    = 'e';
		net->name[1]    = 'n';
		net->mtu        = 1500;
		net->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;

		NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, 100000000);

		eth::start_phy();
		configure_phy();

		return ERR_OK;
	}

	err_t LwipThread::ethernetif_init_static(netif* net)
	{
		// called by lwip during the process of netif_add
		return static_cast<LwipThread*>(net->state)->ethernetif_init(net);
	}
	
	void LwipThread::set_ip(ip_addr ip)
	{
		ip_ = ip;
	}

	void LwipThread::set_mac_address(const uint8 (&arr)[6])
	{
		std::copy(arr, arr+6, mac_addr_);
	}

	void LwipThread::set_gateway(ip_addr gateway)
	{
		gateway_ = gateway;
	}

	void LwipThread::set_netmask(ip_addr netmask)
	{
		netmask_ = netmask;
	}

	void LwipThread::init_lwip()
	{
		lwip_init();

		std::copy(mac_addr_, mac_addr_ + 6, interface_.hwaddr);
		std::copy(mac_addr_, mac_addr_ + 6, ptp_config_.mac_addr.begin());

		interface_.hwaddr_len = 6;

		netif_add(&interface_, &ip_.addr, &netmask_.addr, &gateway_.addr, this, &LwipThread::ethernetif_init_static, ::ethernet_input);
		netif_set_default(&interface_);
		netif_set_up(&interface_);

		sys_timeouts_init();
	}

	void LwipThread::on_event(ThreadEvents event)
	{
		switch (event) {
		case ThreadEvents::Received:
			rx_walk_descriptors(&interface_);
			break;
		case ThreadEvents::Transmitted:
			tx_walk_descriptors();
			break;
		}
	}

	void LwipThread::add_timeout_thread(uint32 when, void(*fn)(void*), void* arg)
	{
		unprecise_timers_.emplace(fn, arg, when);
	}

	void LwipThread::remove_timeout_thread(void(*fn)(void*), void* arg)
	{
		auto it = std::find_if(unprecise_timers_.begin(), unprecise_timers_.end(), [=](const detail::timeout_data& ref) -> bool { return ref.arg == arg && ref.fn == fn; });
		if (it != unprecise_timers_.end()) {
			unprecise_timers_.erase(it);
		}
	}

	void LwipThread::process_timers()
	{
		uint32 now = chTimeNow();
		if(unprecise_timers_.size()) {
			auto next = unprecise_timers_.root().when;

			while (unprecise_timers_.size() && detail::time_overflow_compare(unprecise_timers_.root().when, now)) {
				auto fun = unprecise_timers_.root().fn;
				auto arg = unprecise_timers_.root().arg;
				unprecise_timers_.pop();
				fun(arg);
			}
		}
	}

	uint32 LwipThread::get_next_timer()
	{
		if (unprecise_timers_.size()) {
			return unprecise_timers_.root().when - chTimeNow();
		} else {
			return UINT_MAX;
		}
	}

	// Pins used for state leds and toggle button
	namespace ptp_pins {

		using red = gpio::ph<2>;
		using yellow = gpio::ph<3>;
		using green = gpio::ph<4>;
		using button = gpio::ph<5>;

	}

	void LwipThread::init_ptp()
	{
		rcc::enable_gpio<pins::red, pins::yellow, pins::green, pins::button>();
		gpio::configure<pins::red, pins::yellow, pins::green>(gpio::output());
		gpio::configure<pins::button>(gpio::input, gpio::pull_up());

		ptp_button_debounce_ = 0;
		ptp_clock_enabled_ = false;
	}

	void LwipThread::process_ptp()
	{
		unsigned int val = pins::button::get();
		ptp_button_debounce_ = (ptp_button_debounce_ << 1) | val;

		if((ptp_button_debounce_&0xF) == 12 /*~3&0xF*/ ) {
			if(ptp_clock_enabled_) {
				ptp_clock_.command(uptp::SystemPort::ThreadCommands::DisableClock);
				pins::green::reset();
			} else {
				ptp_clock_.command(uptp::SystemPort::ThreadCommands::EnableClock);
				pins::green::set();
			}

			ptp_clock_enabled_ = !ptp_clock_enabled_;
		}
	}

	void LwipThread::post_event(ThreadEvents ev)
	{
		chSysLock();
		auto msgptr = message_pool_.make();
		chSysUnlock();
		if(msgptr) {
			msgptr->payload_.to_type<ThreadEvents>(ev);
			thread_message* bareptr = msgptr.get_payload();
			msgptr->lifetime_ = std::move(msgptr);
			chSysLock();
			mailbox_.post(reinterpret_cast<msg_t>(bareptr), TIME_INFINITE);
			chSysUnlock();
		}
	}

	void LwipThread::post_event_i(ThreadEvents ev)
	{
		auto msgptr = message_pool_.make();
		if(msgptr) {
			msgptr->payload_.to_type<ThreadEvents>(ev);
			thread_message* bareptr = msgptr.get_payload();
			msgptr->lifetime_ = std::move(msgptr);
			mailbox_.postI(reinterpret_cast<msg_t>(bareptr));
		}
	}

	msg_t LwipThread::operator()()
	{
		init_lwip();
		init_ptp();
		
		bool started_phy_read = false;
		bool link_status = false;
		bool dhcp_bound = false;

		while (true) {
			msg_t msg;
			auto result = mailbox_.fetch(&msg, std::min<uint32>(get_next_timer(), MS2ST(100)));

			if (result == RDY_OK) {
				// we received a message
				util::pool_ptr<thread_message> ptr = std::move(static_cast<thread_message*>(reinterpret_cast<void*>(msg))->lifetime_);

				if (ptr->payload_.is<ThreadEvents>()) {
					on_event(ptr->payload_.as<ThreadEvents>());
				}

				chSysLock();
				ptr.clear();
				chSysUnlock();
			}

			if(started_phy_read) {
				auto phystats = eth::phy_read_register_finish();
				if(!link_status && (phystats & 1)) {
					start_mac_dma(phystats, interface_);
					netif_set_link_up(&interface_);
					dhcp_start(&interface_);
					link_status = true;
					trace_printf(0, "Starting DHCP!\n");
				} else if(link_status && !(phystats&1)){
					//link_status = false;
					//netif_set_link_down(&interface);
				}

				started_phy_read = false;
			} else {
				eth::phy_read_register_start(0x10, 1);
				started_phy_read = true;
			}

			if(link_status && interface_.dhcp->state == DHCP_STATE_BOUND && !dhcp_bound) {
				trace_printf(0, "DHCP bound!\n");
				ptp_clock_.ip_addr_changed(interface_.ip_addr);
				dhcp_bound = true;
			}

			process_timers();
			process_ptp();

		}

	}

} }

template<>
void isr<nvic::IRQ::ETH>()
{
	CH_IRQ_PROLOGUE();

	auto dma_status = eth::device.dmasr.snapshot();

	if (dma_status.field<eth::fields::dmasr::rs>()) {
		// At least 1 frame was received
		chSysLockFromIsr();
		eth::device.dmasr <<= eth::fields::dmasr::nis(1) | eth::fields::dmasr::rs(1);	// clear rs bit
		eth::lwip::LwipThread::get().post_event_i(eth::lwip::ThreadEvents::Received);
		chSysUnlockFromIsr();
	}

	if (dma_status.field<eth::fields::dmasr::ts>()) {
		chSysLockFromIsr();
		eth::device.dmasr <<= eth::fields::dmasr::nis(1) | eth::fields::dmasr::ts(1);	// clear ts bit
		eth::lwip::LwipThread::get().post_event_i(eth::lwip::ThreadEvents::Transmitted);
		chSysUnlockFromIsr();
	}

	CH_IRQ_EPILOGUE();
}

extern "C" {

	// These are called from Lwip thread context.
	void lwip_thread_timeout(uint32 when, void(*fn)(void*), void* arg)
	{
		// convert to systicks
		when = chTimeNow() + MS2ST(when);
		eth::lwip::LwipThread::get().add_timeout_thread(when, fn, arg);
	}

	void lwip_thread_untimeout(void (*handler)(void*), void *arg)
	{
		eth::lwip::LwipThread::get().remove_timeout_thread(handler, arg);
	}

}

#endif
