/*
 * core.hpp
 *
 *  Created on: 12.12.2014
 *      Author: Michael
 */

#ifndef CORE_HPP_
#define CORE_HPP_

#include <stmlib/bits.hpp>

/*
 * This header defines registers for the
 * 	- System control block (pm page 206)
 *  - System tick (pm page )
 *  - FPU (pm page )
 *
 *  The CMSIS package provides more exhaustive capabilities
 *  for the same purpose.
 */

namespace core
{

    namespace fields
    {

        // Auxillary control register (pm page 207)
        namespace actlr
        {
            using disoofp = bit::field<9>;
            using disfpca = bit::field<8>;
            using disfold = bit::field<2>;
            using disdefwbuf = bit::field<1>;
            using dismcycint = bit::field<0>;
        } // namespace actlr

        // CPUID base register (pm page 208)
        namespace cpuid
        {
            using implementer = bit::field<31, 24>;
            using variant = bit::field<23, 20>;
            using partno = bit::field<15, 4>;
            using revision = bit::field<3, 0>;
        } // namespace cpuid

        // Interrupt control and state register (pm page 210)
        namespace icsr
        {
            // NMI set pending bit
            using nmipendset = bit::field<31>;
            // PendSV set-pending bit
            using pendsvset = bit::field<28>;
            // PendSV clear-pending bit
            using pendsvclr = bit::field<27>;
            // SysTick exception set-pending bit
            using pendstset = bit::field<26>;
            // SyStick exception clear-pending bit
            using pendstclr = bit::field<25>;
            // Interrupt pending flag, excluding NMI and Faults
            using isrpending = bit::field<22>;
            // Pending vector
            using vectpending = bit::field<17, 12>;
            // Return to base level (1 when exception was preempted)
            using rettobase = bit::field<11>;
            // Active vector
            using vectactive = bit::field<8, 0>;
        } // namespace icsr

        // Vector table offset register (pm page 212)
        namespace vtor
        {
            // Vector table base offset field
            using tbloff = bit::field<29, 9>;
        } // namespace vtor

        // Application interrupt and reset control register (pm page 212)
        namespace aircr
        {
            // Register key	(must write 0x5FA to this field on every write to this register)
            using vectkey = bit::field<31, 16>;
            // Data endianess
            using endianess = bit::field<15>;
            // Interrupt priority grouping field (see pm page 213)
            using prigroup = bit::field<10, 8>;
            // System reset request
            using sysresetreq = bit::field<2>;
        } // namespace aircr

        // System control register (pm page 214)
        namespace scr
        {
            // Send event on pending bit
            using seveonpend = bit::field<4>;
            // Sleep deepd
            using sleepdeep = bit::field<2>;
            // Sleep on exit
            using sleeponexit = bit::field<1>;
        } // namespace scr

        // Configuration and control register (pm page 215)
        namespace ccr
        {
            // Stack alignment
            using stkalign = bit::field<9>;
            // BFHFNMIGN
            using bfhfnmign = bit::field<8>;
            // Fault on division by 0
            using div0trp = bit::field<4>;
            // Fault on unaligned access
            using unaligntrp = bit::field<3>;
            // Enable user access to nvic software interrupt trigger
            using usersetmpend = bit::field<1>;
            //
            using nonbasethrdena = bit::field<0>;
        } // namespace ccr

        // System handler priority registers (pm page 217)
        namespace shpr1
        {
            using usage_fault = bit::field<23, 16>;
            using bus_fault = bit::field<15, 8>;
            using mem_fault = bit::field<8, 0>;
        } // namespace shpr1

        namespace shpr2
        {
            using svcall = bit::field<31, 24>;
        }

        namespace shpr3
        {
            using systick = bit::field<31, 24>;
            using pendsv = bit::field<23, 16>;
        } // namespace shpr3

        // System handler control and state register (pm page 219)
        struct shcsr : bit::register_base
        {
            using usgfaultena = bit::field<18>;
            using busfaultena = bit::field<17>;
            using memfaultena = bit::field<16>;
            using svcallpended = bit::field<15>;
            using busfaultpended = bit::field<14>;
            using memfaultpended = bit::field<13>;
            using usgfaultpended = bit::field<12>;
            using systickact = bit::field<11>;
            using pendsvact = bit::field<10>;
            using monitoract = bit::field<8>;
            using svcallact = bit::field<7>;
            using usgfaultact = bit::field<3>;
            using busfaultact = bit::field<1>;
            using memfaultact = bit::field<0>;
        };

        // Following registers seem most useful in a capable debugger
        // Configurable fault status register (pm page 221)
        namespace cfsr
        {

        }

        // MemManage fault status register (pm page 221) (part of cfsr)
        namespace mmsr
        {

        }

        // BusFault status register (pm page 221) (part of cfsr)
        namespace bfsr
        {

        }

        // UsageFault status register (pm page 221) (part of cfsr)
        namespace ufsr
        {

        }

        // Hard fault status register (pm page 225)
        namespace hfsr
        {

        }

        // Memory management fault address register (pm page 226)
        namespace mmar
        {

        }

        // Bus fault address register (pm page 226)
        namespace bfar
        {

        }

        // Auxillary fault status register (pm page 227)
        namespace afsr
        {

        }

        /* System Tick */

        // SysTick control register (program reload -> clear current -> set control)
        namespace stk_ctrl
        {
            using countflag = bit::field<16>; // read only, resets on read
            using clksource = bit::field<2>;
            using tickint = bit::field<1>;
            using enable = bit::field<0>;
        } // namespace stk_ctrl

        // SysTick reload value register
        namespace stk_load
        {
            using reload = bit::field<23, 0>;
        }

        // SysTick current value register
        namespace stk_val
        {
            using current = bit::field<23, 0>;
        }

        // SysTick calibration value register
        namespace stk_calib
        {
            using noref = bit::field<31>;
            using skew = bit::field<30>;
            using tenms = bit::field<23, 0>;
        } // namespace stk_calib

    } // namespace fields

    struct register_map
    {
        bit::register_base actlr;
        uint32 zzreserved0;
        bit::register_base stk_ctrl;
        bit::register_base stk_load;
        bit::register_base stk_val;
        bit::register_base stk_calib;
        uint8 zzreserved1[3296];
        bit::register_base cpuid;
        bit::register_base icsr;
        bit::register_base vtor;
        bit::register_base aircr;
        bit::register_base scr;
        bit::register_base ccr;
        bit::register_base shpr1;
        bit::register_base shpr2;
        bit::register_base shpr3;
        fields::shcsr shcsr;
        bit::register_base cfsr;
        bit::register_base hfsr;
        uint32 zzreserved2;
        bit::register_base mmar;
        bit::register_base bfar;
        bit::register_base afsr;
    };

    extern "C" register_map __core__device_0xE000E008;
    static auto &device = __core__device_0xE000E008;

} // namespace core

#endif /* CORE_HPP_ */
