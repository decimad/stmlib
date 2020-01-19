#ifndef STMLIB_ETH_LWIP_ONETHREAD_LWIP_THREAD_HPP__
#define STMLIB_ETH_LWIP_ONETHREAD_LWIP_THREAD_HPP__

#include <stmlib/config.hpp>
#ifdef STMLIB_LWIP_ONETHREAD

#if STMLIB_RUN_PTP
#include "microptp/ports/cortex_m4_onethread/microptp_port.hpp"
#endif

#include <cpp_wrappers/ch.hpp>
#include <lwip/ip.h>
#include <lwip/tcpip.h>
#include <microlib/circular_buffer.hpp>
#include <microlib/pool.hpp>
#include <microlib/static_heap.hpp>
#include <microlib/util.hpp>
#include <microlib/variant.hpp>
#include <stmlib/eth.hpp>
#include <stmlib/stmtypes.hpp>
#include <thread.hpp>

namespace eth::lwip
{
    enum class ThreadEvents
    {
    };

    struct phy_completion
    {
        eth::phy::phy_completion_ptr ptr;
    };

    struct ip_addr
    {
        ip_addr()
        {
        }

        ip_addr(uint32 a, uint32 b, uint32 c, uint32 d) : addr{(a << 0) | (b << 8) | (c << 16) | (d << 24)}
        {
        }

        ip_addr_t addr;
    };

    namespace detail
    {

        struct timeout_data
        {
            timeout_data() : fn(0), arg(0), when(0){};
            timeout_data(const timeout_data &) = default;
            timeout_data(timeout_data &&) = default;
            timeout_data &operator=(const timeout_data &) = default;
            timeout_data(void (*fn_)(void *), void *arg_, uint32 when_, uint32 interval_)
                : fn(fn_), arg(arg_), when(when_), interval(interval_)
            {
            }

            void (*fn)(void *);
            void *arg;
            uint32 when;
            uint32 interval; // == 0 => one shot timer
        };

        template <typename T, typename S = typename ulib::enable_if_t<std::is_unsigned<T>::value>>
        bool time_overflow_compare(T a, T b)
        {
            return T(b - a) <= (std::numeric_limits<T>::max() / 2);
        }

        struct time_overflow_compare_struct
        {
            bool operator()(const timeout_data &a, const timeout_data &b)
            {
                return time_overflow_compare(a.when, b.when);
            }
        };

    } // namespace detail

    // IP-Thread which runs a PTP-Clock
    class LwipThread : public util::static_thread_crtp<LwipThread, 4096>
    {
      public:
        LwipThread();
        msg_t operator()();

        static LwipThread singleton_thread_;

        // singleton anti-pattern
        static LwipThread &get();

        // pre-run config
      public:
        void set_ip(ip_addr ip);
        void set_netmask(ip_addr netmask);
        void set_gateway(ip_addr gateway);
        // void set_mac_address(const uint8 (&arr)[6]);

        template <typename T>
        void set_mac_address(const T &arr)
        {
            std::copy(std::begin(arr), std::end(arr), std::begin(mac_addr_));
        }

        // in-thread routines
      public:
        // I'd like a different timeout interface eventually, but this one is
        // better suited for lwip for now. Work on the mailing list is underway
        // to change this.
        void add_timeout_thread(uint32 when, void (*fn)(void *), void *arg, uint32 interval = 0);
        void remove_timeout_thread(void (*fn)(void *), void *arg);
        void update_timeout_thread(uint32 when, void (*fn)(void *), void *arg, uint32 interval = 0);

        // Event Handling
      public:
        using thread_message = ulib::variant<ThreadEvents, phy_completion>;
        using message_ptr = ulib::pool_ptr<thread_message>;
        message_ptr acquire_message();
        message_ptr acquire_message_i();

        void post_message(message_ptr msg);
        void post_message_i(message_ptr msg);

        void post_event(ThreadEvents);
        void post_event_i(ThreadEvents);

        void on_rx();
        void on_tx();

      private:
        void start_phy_clock();
        void on_phy_clock_started(eth::phy::serial_interface_op *);

        static err_t ethernetif_init_static(struct netif *netif);
        err_t ethernetif_init(netif *netif);

        void init_lwip();
        void init_ptp();

        uint32 get_next_timer();
        void process_timers();

        void process_ptp();

        void on_event(ThreadEvents event);

        // link status tracking
      private:
        void start_link_status_tracking();
        void on_link_status_timer();
        static void on_link_status_timer_static(void *);
        void on_link_status_read_completed(eth::phy::serial_interface_op *op);

        void on_link_rise(uint16 phystats);
        void on_link_fall();

        bool link_status_;

      private:
        ulib::pool<thread_message, 8> message_pool_;
        ulib::circular_buffer2<ulib::pool_ptr<thread_message>, 8> mailbox_;
        ulib::static_heap<detail::timeout_data, 16, detail::time_overflow_compare_struct> unprecise_timers_;

        ip_addr ip_;
        ip_addr netmask_; //(255, 255, 255, 0);
        ip_addr gateway_; //(0, 0, 0, 0);
        uint8 mac_addr_[6];

        netif interface_;

#if STMLIB_RUN_PTP
        uptp::Config ptp_config_;
        uptp::SystemPort ptp_clock_;
        uint8 ptp_button_debounce_;
        bool ptp_clock_enabled_;
#endif
    };

} // namespace eth::lwip

#endif

#endif
