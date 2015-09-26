/*
 * rcc.hpp
 *
 *  Created on: 24.11.2014
 *      Author: Michael
 */

#ifndef RCC_HPP_
#define RCC_HPP_

#define RCC_USE_GPIO true

#include <stmlib/rcc_registers.hpp>
#include <stmlib/stmtypes.hpp>
#include <stmlib/bits.hpp>
#include <stmlib/core.hpp>

#if RCC_USE_GPIO
#include <stmlib/gpio.hpp>
#endif

#include <stmlib/flash_registers.hpp>
#include <stmlib/rcc/pll_parameters.hpp>

namespace rcc {

#if RCC_USE_GPIO
	template< typename... PinsListPack >
	void enable_gpio() {
		using namespace gpio;
		using port_array = typename gpio::detail::port_array_from_pins< gpio::detail::empty_port_array, PinsListPack... >::type;
		using namespace ulib::meta;

		uint32 mask = 0;

		if( get<port_array,0>::value != 0) {
			mask |= 1 << 0;
		}

		if( get<port_array,1>::value != 0) {
			mask |= 1 << 1;
		}

		if( get<port_array,2>::value != 0) {
			mask |= 1 << 2;
		}

		if( get<port_array,3>::value != 0) {
			mask |= 1 << 3;
		}

		if( get<port_array,4>::value != 0) {
			mask |= 1 << 4;
		}

		if( get<port_array,5>::value != 0) {
			mask |= 1 << 5;
		}

		if( get<port_array,6>::value != 0) {
			mask |= 1 << 6;
		}

		if( get<port_array,7>::value != 0) {
			mask |= 1 << 7;
		}

		// rcc base 0x4002 3800
		// register offset 0x30
		device.ahb1enr |= mask;
	}
#endif

	template< unsigned int source_clock, unsigned int target_clock, unsigned int system_tick = 1000 >
	struct clock_values {
		// see rcc/pll_parameters.hpp for parameter calculations.
		static constexpr detail::sol pll = detail::calculate_pll_config(source_clock, target_clock);
		
		static constexpr int p = detail::encode_p(pll.p);
		static constexpr int m = pll.m;
		static constexpr int n = pll.n;
		static constexpr int ahb_prescaler = detail::encode_a(pll.a);
		
		static constexpr int q = detail::calculate_q(source_clock, pll);
		
		static constexpr int fast_apb_prescaler = detail::calculate_fast_apb(source_clock, pll);
		static constexpr int slow_apb_prescaler = detail::calculate_slow_apb(source_clock, pll);

		static constexpr int systick_reload = detail::calculate_systick_reload(source_clock, pll, system_tick);
	};

	template< unsigned int source_clock, unsigned int target_clock, unsigned int system_tick = 1000 >
	void configure_hse()
	{
		using values = clock_values<source_clock, target_clock, system_tick>;
		using namespace fields;

		device.cr <<= cr::hseon(1);
		while(!device.cr.field<cr::hserdy>());

		// Setup main PLL config
		device.pllcfgr <<= pllcfgr::pllq(values::q) |
						   pllcfgr::pllsrc(1)		|	// HSE source
						   pllcfgr::pllp(values::p) |
						   pllcfgr::plln(values::n) |
						   pllcfgr::pllm(values::m);

		device.cr <<= cr::pllon(1);  // Enable PLL

        // Set APB1 and APB2 divisors, since their current value might overclock them after system clock source change
		// If current clock is higher than target clock, those prescalers will have higher values, don't touch them
		// If current clock is lower than target clock, those prescalers will have lower value, so replace them prior to change
		auto snapshot = device.cfgr.snapshot();

		const auto slow_apb_prescaler = snapshot.field<cfgr::ppre2>();
		const auto fast_apb_prescaler = snapshot.field<cfgr::ppre1>();

		if(slow_apb_prescaler < values::slow_apb_prescaler) {
			device.cfgr <<= cfgr::ppre2(values::slow_apb_prescaler);
		}

		if(fast_apb_prescaler < values::fast_apb_prescaler) {
			device.cfgr <<= cfgr::ppre1(values::fast_apb_prescaler);
		}

		while( !device.cr.field<cr::pllrdy>() ); // Wait for PLL stability

		// Set Flash Waitstates
		using namespace flash::fields;
		flash::device.acr = acr::latency(5) | acr::prften(1) | acr::icen(1) | acr::dcen(1);

		// Change main clock source
		device.cfgr <<= cfgr::sw(2);

		while( device.cfgr.field<cfgr::sws>() != 2 ); // wait for pll

		device.cfgr <<= cfgr::hpre(values::ahb_prescaler);

		if(slow_apb_prescaler > values::slow_apb_prescaler) {
			device.cfgr <<= cfgr::ppre2(values::slow_apb_prescaler);
		}

		if(fast_apb_prescaler > values::fast_apb_prescaler) {
			device.cfgr <<= cfgr::ppre1(values::fast_apb_prescaler);
		}

		// Set up SystemTick timer (ChibiOS expects 1kHz frequency)
		core::device.stk_load = values::systick_reload;
		core::device.stk_val = 0;
		core::device.stk_ctrl =
			core::fields::stk_ctrl::tickint(1) |
			core::fields::stk_ctrl::clksource(1) |
			core::fields::stk_ctrl::enable(1);
	}

}

#endif /* RCC_HPP_ */
