#include <stmlib_config.hpp>
#ifdef STMLIB_LWIP_ONETHREAD

#include <stmlib/eth/lwip_onethread/lwip_thread.hpp>
#include <stmlib/eth/lwip_onethread/driver.hpp>
#include <stmlib/eth/lwip_onethread/dma.hpp>
#include <stmlib/gpio.hpp>
#include <stmlib/nvic.hpp>
#include <stmlib/rcc.hpp>
#include <stmlib/trace.h>
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
#if STMLIB_RUN_PTP
		: ptp_clock_(ptp_config_)
#endif
	{
	}

	err_t low_level_output(struct netif *netif, struct pbuf *p) {
		(void) netif;
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

		interface_.hwaddr_len = 6;

		netif_add(&interface_, &ip_.addr, &netmask_.addr, &gateway_.addr, this, &LwipThread::ethernetif_init_static, ::ethernet_input);
		netif_set_default(&interface_);
		netif_set_up(&interface_);

		sys_timeouts_init();
	}

	void LwipThread::on_event(ThreadEvents event)
	{
		switch (event) {
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

	void LwipThread::update_timeout_thread(uint32 when, void(*fn)(void*), void* arg)
	{
		auto it = std::find_if(unprecise_timers_.begin(), unprecise_timers_.end(), [=](const detail::timeout_data& ref) -> bool { return ref.arg == arg && ref.fn == fn; });
		if (it != unprecise_timers_.end()) {
			it->when = when;
			unprecise_timers_.restore(it);
		}
	}

	void LwipThread::process_timers()
	{
		uint32 now = chTimeNow();

		while (unprecise_timers_.size() && detail::time_overflow_compare<uint32>(unprecise_timers_.top_element().when, now)) {
			auto fun = unprecise_timers_.top_element().fn;
			auto arg = unprecise_timers_.top_element().arg;
			unprecise_timers_.pop();
			fun(arg);
		}
	}

	uint32 LwipThread::get_next_timer()
	{
		if (unprecise_timers_.size()) {
			return unprecise_timers_.top_element().when - chTimeNow();
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

#if STMLIB_RUN_PTP
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
#endif

	LwipThread::message_ptr LwipThread::acquire_message()
	{
		chSysLock();
		auto ptr = message_pool_.make();
		chSysUnlock();
		return std::move(ptr);
	}

	LwipThread::message_ptr LwipThread::acquire_message_i()
	{
		return message_pool_.make();
	}

	enum class EventMasks : uint32 {
		Rx = 1,
		Tx = 2,
		Msg = 4
	};

	void LwipThread::post_message(message_ptr msg)
	{
		if (mailbox_.push(std::move(msg))) {
			chEvtSignal(get_thread(), (eventmask_t)EventMasks::Msg);
		}
	}

	void LwipThread::post_message_i(message_ptr msg)
	{
		if (mailbox_.push(std::move(msg))) {
			chEvtSignalI(get_thread(), (eventmask_t)EventMasks::Msg);
		}
	}

	void LwipThread::post_event(ThreadEvents ev)
	{
		auto msg = acquire_message();
		if (msg) {
			msg->to_type<ThreadEvents>(ev);
			post_message(std::move(msg));
		}
	}

	void LwipThread::post_event_i(ThreadEvents ev)
	{
		auto msg = acquire_message();

		if (msg) {
			msg->to_type<ThreadEvents>(ev);
			mailbox_.push(std::move(msg));
		}
	}

	void LwipThread::on_rx() {
		chEvtSignalI(get_thread(), (eventmask_t)EventMasks::Rx);
	}

	void LwipThread::on_tx() {
		chEvtSignalI(get_thread(), (eventmask_t)EventMasks::Tx);
	}

	msg_t LwipThread::operator()()
	{
		init_lwip();

#if STMLIB_RUN_PTP
		init_ptp();
#endif
		
		bool started_phy_read = false;
		bool link_status = false;
		bool dhcp_bound = false;

		while (true) {
			auto result = chEvtWaitAnyTimeout((eventmask_t)EventMasks::Msg | (eventmask_t)EventMasks::Rx | (eventmask_t)EventMasks::Tx, std::min<uint32>(get_next_timer(), MS2ST(100)));
			switch (result) {
			case (eventmask_t)EventMasks::Msg:
/* We don't define commands yet.
 * Need to define a way which supports
 * ulib::function or the like
 * Would also be nice to find a general way
 * to inject entities into a thread from outside.
 * (at compile time)
				{
					while (mailbox_.size()) {
						auto ptr = std::move(mailbox_.front());
						chSysLock();
						mailbox_.pop();
						chSysUnlock();
						// do something with ptr
						chSysLock();
						ptr.clear();
						chSysUnlock();
					}
				}
*/
				break;
			case (eventmask_t)EventMasks::Rx:
				// lwip input functions need the netif.
				rx_walk_descriptors(&interface_);
				break;
			case (eventmask_t)EventMasks::Tx:
				tx_walk_descriptors();
				break;
			}

			// separate phy read command from the actual read
			// in order to avoid blocking while waiting for
			// the answer.
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
#if STMLIB_RUN_PTP
				ptp_clock_.network_changed(interface_.ip_addr, reinterpret_cast<std::array<uint8,6>&>(interface_.hwaddr));
#endif
				dhcp_bound = true;
			}

			process_timers();

#if STMLIB_RUN_PTP
			process_ptp();
#endif
		}


	}

	LwipThread singleton;

	LwipThread& LwipThread::get()
	{
		return singleton;
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
		eth::lwip::LwipThread::get().on_rx();
		chSysUnlockFromIsr();
	}

	if (dma_status.field<eth::fields::dmasr::ts>()) {
		chSysLockFromIsr();
		eth::device.dmasr <<= eth::fields::dmasr::nis(1) | eth::fields::dmasr::ts(1);	// clear ts bit
		eth::lwip::LwipThread::get().on_tx();
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
