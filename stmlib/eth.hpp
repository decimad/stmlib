/*
 * eth.hpp
 *
 *  Created on: 14.12.2014
 *      Author: Michael
 */

#ifndef ETH_HPP_
#define ETH_HPP_

#include <microlib/functional.hpp>
#include <microlib/pool.hpp>
#include <stmlib/bits.hpp>

namespace eth
{

    void start_phy();
    void start_rmii();

    void ptp_initialize();
    void ptp_uninitialize();

    void ptp_init_time(int32 seconds, int32 subseconds);
    void ptp_update_time(uint32 seconds, uint32 subseconds);
    void ptp_start_timestamp();
    void ptp_stop_timestamp();

    void ptp_set_subsecond_increment(uint8 increment);
    void ptp_set_addend(uint32 addend);
    void ptp_discipline(int32 ppb);

    uint32 ptp_subseconds_to_nanos(uint32 subs);
    uint32 ptp_nanos_to_subseconds(uint32 nanos);

    uint64 ptp_hardware_to_logical(uint64 hardware);
    uint64 ptp_logical_to_hardware(uint64 logical);

    void ptp_get_time(uint32 &seconds, uint32 &subseconds);

    namespace phy
    {

        void write_register(uint8 regnum, uint8 address, uint16 value);
        void write_register(uint8 regnum, uint8 address, bit::masked_value<uint16> value);

        uint16 read_register_blocking(uint8 regnum, uint8 address);
        void write_register_blocking(uint8 regnum, uint8 address, uint16 value);

        void read_register(uint8 regnum, uint8 address);

        void read_register_start(uint8 regnum, uint8 address);
        uint16 read_register_finish();

        template <unsigned int Top, unsigned int Bottom = Top>
        using phy_field = bit::field<Top, Bottom, uint16>;

        template <uint8 Address, uint8 Page = 0>
        class phy_register : public bit::basic_register<uint16>
        {
          public:
            static constexpr uint8 address = Address;
            static constexpr uint8 page = Page;
            static constexpr uint8 regnum = (Page << 5) | address;

            phy_register(uint16 val) : basic_register(val)
            {
            }

            static constexpr bool is_extended()
            {
                return address >= 0x14;
            }
        };

        namespace fields
        {

            namespace bmcr
            {

                using reset = phy_field<15>;
                using loopback = phy_field<14>;
                using speed_select = phy_field<13>;
                using auto_negotiate_enable = phy_field<12>;
                using power_down = phy_field<11>;
                using isolate = phy_field<10>;
                using restart_auto_negotiate = phy_field<9>;
                using duplex_mode = phy_field<8>;
                using collision_test = phy_field<7>;
                using unidirectional_enable = phy_field<5>;

            }; // namespace bmcr

            namespace bmsr
            {

                using base100_t4 = phy_field<15>;
                using base100_tx_full_dpled = phy_field<14>;
                using base100_tx_half_duplex = phy_field<13>;
                using base10_t_full_duplex = phy_field<12>;
                using base10_t_half_duplex = phy_field<11>;
                using unidirectional_ability = phy_field<7>;
                using mf_preamble_suppress = phy_field<6>;
                using auto_negotiate_compl = phy_field<5>;
                using remote_field = phy_field<4>;
                using auto_negotiate_ability = phy_field<3>;
                using link_status = phy_field<2>;
                using jabber_detect = phy_field<1>;
                using extended_capability = phy_field<0>;

            } // namespace bmsr

            namespace phy_idr1
            {
                using oui_msb = phy_field<15, 0>;
            }

            namespace phy_idr2
            {
                using oui_lsb = phy_field<15, 10>;
                using vndr_mdl = phy_field<9, 4>;
                using mdl_rev = phy_field<3, 0>;
            } // namespace phy_idr2

            namespace anar
            {
                using np = phy_field<15>;
                using rf = phy_field<14>;
                using asm_dir = phy_field<11>;
                using pause = phy_field<10>;
                using t4 = phy_field<9>;
                using tx_df = phy_field<8>;
                using tx = phy_field<7>;
                using ten_fd = phy_field<6>;
                using ten = phy_field<5>;
                using selector = phy_field<4, 0>;
            } // namespace anar

            namespace anlpar
            {
                using np = phy_field<15>;
                using ack = phy_field<14>;
                using rf = phy_field<13>;
                using asm_dir = phy_field<11>;
                using pause = phy_field<10>;
                using t4 = phy_field<9>;
                using tx_fd = phy_field<8>;
                using tx = phy_field<7>;
                using ten_fd = phy_field<6>;
                using ten = phy_field<5>;
                using selector = phy_field<4, 0>;
                // fixme: next page?
            } // namespace anlpar

            namespace aner
            {
                using pdf = phy_field<4>;
                using lp_np_able = phy_field<3>;
                using np_able = phy_field<2>;
                using page_rx = phy_field<1>;
                using lp_an_able = phy_field<0>;
            } // namespace aner

            namespace annptr
            {
                using np = phy_field<15>;
                using mp = phy_field<13>;
                using ack2 = phy_field<12>;
                using tog_tx = phy_field<11>;
                using code = phy_field<10, 0>;
            } // namespace annptr

            namespace physts
            {
                using mdix_mode = phy_field<14>;
                using recv_error_latch = phy_field<13>;
                using polarity_status = phy_field<12>;
                using false_carr_sense_latch = phy_field<11>;
                using signal_detect = phy_field<10>;
                using descrambler_lock = phy_field<9>;
                using page_received = phy_field<8>;
                using mii_interrupt = phy_field<7>;
                using remote_fault = phy_field<6>;
                using jabber_detect = phy_field<5>;
                using auto_neg_complete = phy_field<4>;
                using loopback_status = phy_field<3>;
                using duplex_status = phy_field<2>;
                using speed_status = phy_field<1>;
                using link_status = phy_field<0>;
            } // namespace physts

            namespace micr
            {
                using ptp_int_sel = phy_field<3>;
                using tint = phy_field<2>;
                using inten = phy_field<1>;
                using int_oe = phy_field<0>;
            } // namespace micr

            namespace misr
            {
                using lq_int = phy_field<15>;
                using ed_int = phy_field<14>;
                using link_int = phy_field<13>;
                using spd_dup_int = phy_field<12>;
                using dup_ptp_int = phy_field<11>;
                using anc_int = phy_field<10>;
                using fhf_int = phy_field<9>;
                using rhf_pcf_int = phy_field<8>;
                using lq_int_en = phy_field<7>;
                using ed_int_en = phy_field<6>;
                using link_int_en = phy_field<5>;
                using spd_int_en = phy_field<4>;
                using dup_ptp_int_en = phy_field<3>;
                using anc_int_en = phy_field<2>;
                using fhf_ctr_int_en = phy_field<1>;
                using rhf_pcf_int_en = phy_field<0>;
            } // namespace misr

            namespace pagesel
            {
                using page_sel = phy_field<2, 0>;
            }
        } // namespace fields

        namespace registers
        {

            using bmcr = phy_register<0x00>;
            using bmsr = phy_register<0x01>;
            using phyidr1 = phy_register<0x02>;
            using phyidr2 = phy_register<0x03>;
            using anar = phy_register<0x04>;
            using anlpar = phy_register<0x05>;
            using aner = phy_register<0x06>;
            using annptr = phy_register<0x07>;
            using physts = phy_register<0x10>;
            using micr = phy_register<0x11>;
            using misr = phy_register<0x12>;
            using pagesel = phy_register<0x13>;

        } // namespace registers

        using intptr_t = int;
        using uintptr_t = unsigned int;
        using uint16 = unsigned short;

        struct serial_interface_op
        {
            ulib::function<void(serial_interface_op *)> callback;
            uint16 value;
            uint8 regnum;
            uint8 address;
        };

        using phy_completion_ptr = ulib::pool_ptr<serial_interface_op>;

        using callback_type = ulib::function<void(serial_interface_op *)>;

        void poll_idle();
        void flush();
        serial_interface_op *read(uint8 regnum, uint8 address, callback_type callback);
        serial_interface_op *write(uint8 regnum, uint8 address, uint16 value, callback_type callback);

        serial_interface_op *read_extended(uint8 regnum, uint8 page, uint8 address, callback_type callback);
        serial_interface_op *write_extended(uint8 regnum, uint8 page, uint8 address, uint16 value, callback_type callback);

        template <typename Register>
        serial_interface_op *write(uint8 address, bit::masked_value<uint16> val, callback_type callback = callback_type())
        {
            if (Register::is_extended())
            {
                return write_extended(Register::address, Register::page, address, val.value_, std::move(callback));
            }
            else
            {
                return write(Register::address, address, val.value_, std::move(callback));
            }
        }

        template <typename Register>
        serial_interface_op *write(uint8 address, uint16 val, callback_type callback = callback_type())
        {
            if (Register::is_extended())
            {
                return write_extended(Register::address, Register::page, address, val, std::move(callback));
            }
            else
            {
                return write(Register::address, address, val, std::move(callback));
            }
        }

        template <typename Register>
        serial_interface_op *read(uint8 address, callback_type callback)
        {
            if (Register::is_extended())
            {
                return read_extended(Register::address, Register::page, address, std::move(callback));
            }
            else
            {
                return read(Register::address, address, std::move(callback));
            }
        }

    } // namespace phy
} // namespace eth

#endif /* ETH_HPP_ */
