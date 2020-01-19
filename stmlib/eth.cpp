/*
 * eth.cpp
 *
 *  Created on: 15.12.2014
 *      Author: Michael
 */

#include "ch.h"
#include <fixed/fixed.hpp>
#include <microlib/functional.hpp>
#include <microlib/pool.hpp>
#include <microlib/static_deque.hpp>
#include <microlib/variant.hpp>
#include <stmlib/eth.hpp>
#include <stmlib/eth/dma_descriptors.hpp>
#include <stmlib/eth/lwip/driver.hpp>
#include <stmlib/eth_registers.hpp>
#include <stmlib/gpio.hpp>
#include <stmlib/rcc.hpp>
#include <stmlib/rcc_registers.hpp>
#include <stmlib/syscfg_registers.hpp>
#include <stmlib/trace.h>

void send_phy_completion(eth::phy::phy_completion_ptr ptr);

namespace pins
{
    using namespace gpio;

    // datasheet 47ff

    using mdc = pc<1>;
    using mdio = pa<2>;

    // using pps_out = pb<5>; // pb<5> or pg<8>

    namespace mii
    {

        using crs = pa<0>; // pa<0> or ph<2>
        using col = ph<3>; // ph<3> or pa<3>

        using rx_clk = pa<1>;
        using rx_er = pb<10>;
        using rx_dv = pa<7>;
        using rx_d0 = pc<4>;
        using rx_d1 = pc<5>;
        using rx_d2 = pb<0>; // pb<0> or ph<6>
        using rx_d3 = pb<1>; // pb<1> or ph<7>

        using rx = pin_list<rx_clk, rx_er, rx_dv, rx_d0, rx_d1, rx_d2, rx_d3>;

        using tx_clk = pc<3>;
        using tx_en = pg<11>; // pb<11> or pg<11>
        using tx_d0 = pg<13>; // pb<12> or pg<13>
        using tx_d1 = pg<14>; // pb<13> or pg<14>
        using tx_d2 = pc<2>;
        using tx_d3 = pb<8>;

        using tx = pin_list<tx_clk, tx_en, tx_d0, tx_d1, tx_d2, tx_d3>;
    } // namespace mii
    namespace rmii
    {
#ifdef TARGET_FO_CONTROLLER
        using crs = pa<7>;
        using ref_clk = pa<1>;
        using rx_d0 = pc<4>;
        using rx_d1 = pc<5>;
        using tx_en = pb<11>; // pb<11> or pg<11>
        using tx_d0 = pb<12>; // pb<12> or pg<13>
        using tx_d1 = pb<13>; // pb<13> or pg<14>

        using pps = pb<5>; // pb<5> or pg<8>

        using tx = pin_list<ref_clk, tx_en, tx_d0, tx_d1>;
        using rx = pin_list<ref_clk, crs, rx_d0, rx_d1, pps>;
#else
        using crs = pa<7>;
        using ref_clk = pa<1>;
        using rx_d0 = pc<4>;
        using rx_d1 = pc<5>;
        using tx_en = pg<11>; // pb<11> or pg<11>
        using tx_d0 = pg<13>; // pb<12> or pg<13>
        using tx_d1 = pg<14>; // pb<13> or pg<14>

        using pps = pg<8>; // pb<5> or pg<8>

        using tx = pin_list<ref_clk, tx_en, tx_d0, tx_d1>;
        using rx = pin_list<ref_clk, crs, rx_d0, rx_d1, pps>;
#endif
    } // namespace rmii
} // namespace pins

namespace eth
{
    using namespace fields;

    namespace phy
    {

        using phy_pool_type = ulib::pool<serial_interface_op, 8>;
        using phy_queue_type = ulib::static_deque<phy_pool_type::pointer_type, 8>;
        phy_pool_type phy_pool;
        phy_queue_type phy_queue;

        void phy_start_front()
        {
            chSysLock();
            auto front_ptr = phy_queue.front().get();
            chSysUnlock();

            if (front_ptr->address & 0xF0)
            {
                read_register(front_ptr->regnum, front_ptr->address & 0x0F);
            }
            else
            {
                write_register(front_ptr->regnum, front_ptr->address, front_ptr->value);
            }
        }

        namespace
        {
            uint8 current_page = 0;
            bool transfer_started = false;
        } // namespace

        bool switch_extended_page(uint8 address, uint8 page)
        {
            if (current_page != page)
            {
                if (write<registers::pagesel>(address, fields::pagesel::page_sel(page), callback_type()))
                {
                    current_page = page;
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return true;
            }
        }

        serial_interface_op *read(uint8 regnum, uint8 address, ulib::function<void(serial_interface_op *)> callback)
        {
            chSysLock();
            auto ptr = phy_pool.make();
            chSysUnlock();

            if (ptr)
            {
                ptr->callback = callback;
                ptr->regnum = regnum;
                ptr->address = address | 0xF0; // mark read
                auto *retval = ptr.get();

                chSysLock();
                // bool was_empty = phy_queue.empty();
                phy_queue.push_back(std::move(ptr));
                // if( was_empty ) {
                //	phy_start_front();
                //}
                chSysUnlock();

                return retval;
            }
            else
            {
                return nullptr;
            }
        }

        serial_interface_op *read_extended(uint8 regnum, uint8 page, uint8 address, ulib::function<void(serial_interface_op *)> callback)
        {
            if (switch_extended_page(address, page))
            {
                return read(regnum, address, std::move(callback));
            }
            else
            {
                return nullptr;
            }
        }

        serial_interface_op *write_extended(uint8 regnum, uint8 page, uint8 address, uint16 value,
                                            ulib::function<void(serial_interface_op *)> callback)
        {
            if (switch_extended_page(address, page))
            {
                return write(regnum, address, value, std::move(callback));
            }
            else
            {
                return nullptr;
            }
        }

        serial_interface_op *write(uint8 regnum, uint8 address, uint16 value, ulib::function<void(serial_interface_op *)> callback)
        {
            chSysLock();
            auto ptr = phy_pool.make();
            chSysUnlock();

            if (ptr)
            {
                ptr->callback = callback;
                ptr->regnum = regnum;
                ptr->value = value;
                ptr->address = address | 0x00; // mark write
                auto *retval = ptr.get();

                chSysLock();
                // bool was_empty = phy_queue.empty();
                phy_queue.push_back(std::move(ptr));
                // if( was_empty ) {
                //	phy_start_front();
                //}
                chSysUnlock();

                return retval;
            }
            else
            {
                return nullptr;
            }
        }

        void finish_front()
        {
            auto value = device.macmiidr.get();

            chSysLock();
            auto front = std::move(phy_queue.front());
            phy_queue.pop_front();
            chSysUnlock();

            if (front->address & 0xF0)
            {
                front->value = value;
            }

            if (front->callback)
            {
                send_phy_completion(std::move(front));
            }
        }

        void handle_interrupt()
        {
            finish_front();
        }

        void poll_idle()
        {
            if (!phy_queue.empty())
            {
                if (!transfer_started)
                {
                    phy_start_front();
                    transfer_started = true;
                }

                if (transfer_started)
                {
                    while (device.macmiiar.field<macmiiar::mb>())
                        ;
                    finish_front();
                    transfer_started = false;
                }
            }
        }

        // busy flushing all pending operations (and what they might trigger on completion)
        void flush()
        {
            while (!phy_queue.empty())
            {
                poll_idle();
            }
        }

        void write_register(uint8 regnum, uint8 address, uint16 value)
        {
            // trace_printf(0, "Idle: Writing to register %u value %u\n", (unsigned int)regnum, (unsigned int)value);
            device.macmiidr = macmiidr::md(value);
            device.macmiiar = macmiiar::pa(address) | macmiiar::mr(regnum) | macmiiar::cr(4) | macmiiar::mw(1) | macmiiar::mb(1);
        }

        void write_register(uint8 regnum, uint8 address, bit::masked_value<uint16> value)
        {
            // trace_printf(0, "Idle: Writing to register %u value %u\n", (unsigned int)regnum, (unsigned int)value.value_);
            // while(device.macmiiar.field<macmiiar::mb>());
            device.macmiidr <<= macmiidr::md(value.value_);
            device.macmiiar = macmiiar::pa(address) | macmiiar::mr(regnum) | macmiiar::cr(4) | macmiiar::mw(1) | macmiiar::mb(1);
        }

        void read_register(uint8 regnum, uint8 address)
        {
            device.macmiiar <<= macmiiar::pa(address) | macmiiar::mr(regnum) | macmiiar::cr(4) | macmiiar::mw(false) | macmiiar::mb(1);
        }

        uint16 read_register_blocking(uint8 regnum, uint8 address)
        {
            while (device.macmiiar.field<macmiiar::mb>())
                ;
            device.macmiiar <<= macmiiar::pa(address) | macmiiar::mr(regnum) | macmiiar::cr(4) | macmiiar::mw(false) | macmiiar::mb(1);
            while (device.macmiiar.field<macmiiar::mb>())
                ;
            return device.macmiidr.get();
        }

        void write_register_blocking(uint8 regnum, uint8 address, uint16 value)
        {
            // trace_printf(0, "Idle: Writing to register %u value %u\n", (unsigned int)regnum, (unsigned int)value.value_);
            while (device.macmiiar.field<macmiiar::mb>())
                ;
            device.macmiidr <<= macmiidr::md(value);
            device.macmiiar = macmiiar::pa(address) | macmiiar::mr(regnum) | macmiiar::cr(4) | macmiiar::mw(1) | macmiiar::mb(1);
            while (device.macmiiar.field<macmiiar::mb>())
                ;
        }

        void read_register_start(uint8 regnum, uint8 address)
        {
            while (device.macmiiar.field<macmiiar::mb>())
                ;
            device.macmiiar <<= macmiiar::pa(address) | macmiiar::mr(regnum) | macmiiar::cr(4) | macmiiar::mw(false) | macmiiar::mb(1);
        }

        uint16 read_register_finish()
        {
            while (device.macmiiar.field<macmiiar::mb>())
                ;
            return device.macmiidr.get();
        }

    } // namespace phy

    void set_rmii()
    {
        using namespace rcc::fields;
        // set mac mode to rmii
        // check rm page 287

        // enable syscfg
        rcc::device.apb2enr <<= apb2enr::syscfgen(true);

        rcc::device.ahb1rstr <<= ahb1rstr::ethmacrst(1);
        syscfg::device.pmc <<= syscfg::fields::pmc::mii_rmii_sel(1);
        rcc::device.ahb1rstr <<= ahb1rstr::ethmacrst(0);

        device.dmabmr <<= dmabmr::sr(1);
        while (device.dmabmr.field<dmabmr::sr>())
            ;
    }

    void config_mdio()
    {
        using namespace gpio;
        rcc::enable_gpio<pins::mdc, pins::mdio>();
        configure<pins::mdc, pins::mdio>(alternate_output(AlternateMode::Eth, OutputType::PushPull, Speed::Fast));
        configure<pins::mdio>(pull_up());
    }

    void config_rmii_pins()
    {
        using namespace gpio;
        rcc::enable_gpio<pins::rmii::tx, pins::rmii::rx>();
        configure<pins::rmii::rx>(alternate_output(AlternateMode::Eth));
        configure<pins::rmii::tx>(alternate_output(AlternateMode::Eth));
    }

    void start_phy()
    {
        rcc::device.ahb1enr <<= rcc::fields::ahb1enr::ethmacen(true);
        config_mdio();
    }

    void start_rmii()
    {
        config_rmii_pins();
        rcc::device.ahb1enr <<= rcc::fields::ahb1enr::ethmacen(true) | rcc::fields::ahb1enr::ethmacrxen(true)
                                | rcc::fields::ahb1enr::ethmactxen(true) | rcc::fields::ahb1enr::ethmacptpen(true);
        set_rmii();
    }

    void ptp_start_timestamp()
    {
        device.ptptscr = fields::ptptscr::tse(1) | fields::ptptscr::tsfcu(1) // Fine method
                         | fields::ptptscr::tsssr(0)                         // Rollover fixed point
                         | fields::ptptscr::tsptppsv2e(1)                    // ptp v2
                         | fields::ptptscr::tssipv4fe(1)                     // ipv4
                         | fields::ptptscr::tsseme(1)                        // stamp only event messages
                         | fields::ptptscr::tscnt(0)                         // ordinary clock
            ;
    }

    const auto addend = 857828500ul
#ifdef TARGET_FO_CONTROLLER
//			- 13000000
#endif
        ;

    void ptp_initialize()
    {
        // Step 1.
        device.macimr <<= fields::macimr::tstim(1);

        // Step 2.
        device.ptptscr <<= fields::ptptscr::tse(1) | fields::ptptscr::tssarfe(1) | fields::ptptscr::tsptppsv2e(1);

        // Step 3.
        ptp_set_subsecond_increment(64);

        // Step 4. and 5.
        ptp_set_addend(addend);

        // Step 6.
        device.ptptscr <<= fields::ptptscr::tsfcu(1);

        // activate pps...
        device.ptpppscr = 10;

        // Step 7. and 8.
        ptp_init_time(0, 0);
    }

    void ptp_discipline(int32 ppb)
    {
#ifndef TARGET_FO_CONTROLLER
        ptp_set_addend(addend + (110 * ppb) / 128);
#else
        ptp_set_addend(addend + (110 * ppb) / 128);
#endif
    }

    void ptp_uninitialize()
    {
        // rcc::device.ahb1enr <<= rcc::fields::ahb1enr::ethmacptpen(false);
    }

    void ptp_init_time(int32 seconds, int32 subseconds)
    {
        while (device.ptptscr.field<fields::ptptscr::tssti>())
            ;
        device.ptptslur = subseconds;
        device.ptptshur = seconds;
        device.ptptscr <<= fields::ptptscr::tssti(1);
    }

    void ptp_update_time(uint32 seconds, uint32 subseconds)
    {
        while (device.ptptscr.field<fields::ptptscr::tsstu>() || device.ptptscr.field<fields::ptptscr::tssti>())
            ;
        device.ptptshur = seconds;
        device.ptptslur = subseconds;
        device.ptptscr <<= fields::ptptscr::tsstu(1);
    }

    void ptp_set_subsecond_increment(uint8 increment)
    {
        device.ptpssir = increment;
    }

    void ptp_set_addend(uint32 addend)
    {
        while (device.ptptscr.field<fields::ptptscr::tsaru>())
            ;
        device.ptptsar = addend;
        device.ptptscr <<= fields::ptptscr::tsaru(1);
        while (device.ptptscr.field<fields::ptptscr::tsaru>())
            ;
    }

    constexpr auto to_nano_factor = fix::ufixed<2, 30>::from(1000000000. / (1ull << 31));
    constexpr auto to_subsecond_factor = fix::ufixed<2, 30>::from(double(1ull << 31) / 1000000000.);

    uint32 ptp_subseconds_to_nanos(uint32 subs)
    {
        return fix::mul<fix::fits<32, 0>>(fix::integer(subs), to_nano_factor).to<uint32>();
    }

    uint32 ptp_nanos_to_subseconds(uint32 nanos)
    {
        return fix::mul<fix::fits<32, 0>>(fix::integer(nanos), to_subsecond_factor).to<uint32>();
    }

    uint64 ptp_hardware_to_logical(uint64 hardware)
    {
        return (hardware & 0xFFFFFFFF00000000ull) | ptp_subseconds_to_nanos(hardware & 0xFFFFFFFF);
    }

    uint64 ptp_logical_to_hardware(uint64 logical)
    {
        return (logical & 0xFFFFFFFF00000000ull) | ptp_nanos_to_subseconds(logical & 0xFFFFFFFF);
    }

    void ptp_stop_timestamp()
    {
        device.ptptscr <<= fields::ptptscr::tse(0);
    }

    void ptp_get_time(uint32 &seconds, uint32 &subseconds)
    {
        seconds = device.ptptshr.get();
        subseconds = device.ptptslr.get();
    }

} // namespace eth
