#pragma once
#include <microptp/ports/cortex_m4_inthread/cortex_m4.hpp>
#include <microptp/util/static_union.hpp>
#include <thread.hpp>
#include <lwip/tcpip.h>
#include <microlib/pool.hpp>
#include <microlib/static_heap.hpp>
#include <cpp_wrappers/ch.hpp>

namespace eth {

	namespace lwip {

		enum class ThreadEvents {
			Received,
			Transmitted
		};

		struct ip_addr {
			ip_addr()
			{
			}

			ip_addr(uint32 a, uint32 b, uint32 c, uint32 d)
				: addr{ (a << 0) | (b << 8) | (c << 16) | (d << 24) }
			{
			}

			ip_addr_t addr;
		};
		
		namespace detail {

			struct timeout_data {
				void(*fn)(void*);
				void* arg;
				uint32 when;
			};

			template< typename T, typename S = typename std::enable_if<std::is_unsigned<T>::value>::type >
			bool time_overflow_compare(T a, T b)
			{
				return T(b - a) <= (std::numeric_limits<T>::max() / 2);
			}

			struct time_overflow_compare_struct {
				bool operator()(const timeout_data& a, const timeout_data& b) {
					return time_overflow_compare(a.when, b.when);
				}
			};

		}

		// IP-Thread which runs a PTP-Clock
		class LwipThread : public util::static_thread_crtp<LwipThread, 4096>
		{
		public:
			LwipThread();
			msg_t operator()();

			// singleton anti-pattern
			static LwipThread& get()
			{
				static LwipThread thread;
				return thread;
			}

			// pre-run config
		public:
			void set_ip(ip_addr ip);
			void set_netmask(ip_addr netmask);
			void set_gateway(ip_addr gateway);

			// in-thread routines
		public:
			void add_timeout_thread(uint32 when, void (*fn)(void*), void* arg);
			void remove_timeout_thread(void(*fn)(void*), void* arg);

		private:
			static err_t ethernetif_init_static(struct netif* netif);
			err_t ethernetif_init(netif* netif);

			void init_lwip();
			void init_ptp();

			uint32 get_next_timer();
			void process_timers();

			void process_ptp();

			void on_event(ThreadEvents event);
			void on_tcpip_msg(tcpip_msg& msg);

		private:
			struct thread_message {
				util::static_union< ThreadEvents, tcpip_msg > payload_;
				util::pool_ptr<thread_message> lifetime_;
			};

			util::pool< thread_message, 8 > message_pool_;
			util::static_heap< detail::timeout_data, 8, detail::time_overflow_compare_struct > unprecise_timers_;

			chibios_rt::MailboxBuffer<32> mailbox_;

			ip_addr ip_; //(0, 0, 0, 0);
			ip_addr netmask_; //(255, 255, 255, 0);
			ip_addr gateway_; //(0, 0, 0, 0);
			uint8 mac_addr_[6];

			netif interface_;

			uptp::Config ptp_config_;
			uptp::SystemPort ptp_clock_;
		};
		
	}

}