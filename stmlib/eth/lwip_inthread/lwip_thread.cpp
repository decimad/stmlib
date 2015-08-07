#include <stmlib/eth/lwip/lwip_thread.hpp>
#include <lwip/udp.h>
#include <netif/etharp.h>

// Note: We're reimplementing lwip's tcp thread here so we can avoid all the unnecessary thread conntext changes

namespace eth { namespace lwip {
	
	LwipThread::LwipThread()
	{
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
		static_cast<LwipThread*>(net->state)->ethernetif_init(net);
	}
	
	void LwipThread::set_ip(ip_addr ip)
	{
		ip_ = ip;
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
		netif_init();
		etharp_init();
		udp_init();
		tcpip_init(NULL, NULL);

		std::copy(mac_addr_, mac_addr_ + 6, interface_.hwaddr);
		std::copy(mac_addr_, mac_addr_ + 6, ptp_config_.mac_addr.begin());

		interface_.hwaddr_len = 6;

		netif_add(&interface_, &ip_.addr, &netmask_.addr, &gateway_.addr, this, &LwipThread::ethernetif_init_static, ::ethernet_input);
		netif_set_default(&interface_);
		netif_set_up(&interface_);
	}

	void LwipThread::on_event(ThreadEvents event)
	{
		switch (event) {
		case ThreadEvents::Received:
			rx_walk_descriptors();
			break;
		case ThreadEvents::Transmitted:
			tx_walk_descriptors();
			break;
		}
	}

	void LwipThread::on_tcpip_msg(tcpip_msg& msg)
	{

	}

	void LwipThread::process_timers()
	{
		auto now = chTimeNow();
		while (unprecise_timers_.size() && detail::time_overflow_compare(unprecise_timers_.root().when, now)) {
			auto fun = unprecise_timers_.root().fn;
			auto arg = unprecise_timers_.root().arg;
			unprecise_timers_.pop();
			fun(arg);		
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

	void LwipThread::init_ptp()
	{

	}

	void LwipThread::process_ptp()
	{

	}

	msg_t LwipThread::operator()()
	{
		init_lwip();
		init_ptp();
		
		while (true) {
			msg_t msg;
			auto result = mailbox_.fetch(&msg, std::min<uint32>(get_next_timer(), MS2ST(100)));

			if (result == RDY_OK) {
				// we received a message
				util::pool_ptr<thread_message> ptr = std::move(static_cast<thread_message*>(reinterpret_cast<void*>(msg))->lifetime_);

				if (ptr->payload_.is<ThreadEvents>()) {
					on_event(ptr->payload_.as<ThreadEvents>());
				} else if (ptr->payload_.is<tcpip_msg>()) {
					on_tcpip_msg(ptr->payload_.as<tcpip_msg>());
				} else if (ptr->payload_.is<tcpip_msg*>()) {
					// named "static" tcpip_msg in lwip
					on_tcpip_msg(*ptr->payload_.as<tcpip_msg*>());
				}
			}

			process_timers();
			process_ptp();

		}

	}

} }