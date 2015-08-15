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

namespace rcc {

#if RCC_USE_GPIO
	template< typename... PinsListPack >
	void enable_gpio() {
		using namespace gpio;
		using port_array = typename detail::port_array_from_pins< gpio::detail::empty_port_array, PinsListPack... >::type;
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

	template< unsigned int Value, unsigned int Pos = 0 >
	struct uppermost {
		static const unsigned int value = uppermost<(Value >> 1), Pos + 1>::value;
	};

	template< unsigned int Pos >
	struct uppermost< 1, Pos > {
		static const unsigned int value = Pos;
	};

	constexpr unsigned int div_upper( uint64 a, uint64 b )
	{
		return (a%b == 0) ? (a/b) : (a/b + 1);
	}

	constexpr unsigned int div_lower( uint64 a, uint64 b )
	{
		return a/b;
	}

	constexpr unsigned int div_rounded( uint64 a, uint64 b )
	{
		return (2*(a%b) > b) ? (a/b + 1) : (a/b);
	}

	constexpr uint64 calc_clock( uint64 oscillator_freq, uint64 m, uint64 p, uint64 n )
	{
		return oscillator_freq * n / (m*2*(p+1));
	}

	constexpr unsigned int cabs( int32 value ) {
		return (value < 0) ? -value : value;
	};

	constexpr unsigned int log2_lower( uint64 value )
	{
		return (value == 1) ? 1 : ((value > 1) ? (1 + log2_lower(value >> 1)) : -1);
	}

	constexpr unsigned int log2_upper( uint64 value )
	{
		return ((1 << log2_lower(value)) == value) ? log2_lower(value) : log2_lower(value) + 1;
	}

	constexpr unsigned int encode_exponent( unsigned int exponent, unsigned int width )
	{
		return (exponent == 0) ? 0 : ((1<<(width-1)) | (exponent-1));
	}

	template< unsigned int external_clock, unsigned int target_clock, unsigned int system_tick_clock = 1000 >
	struct pll_values
	{
		// hse -> /m -> 1-2MHz => PLL(*p/n) -> SW -> System Clock -> AHB_PRESCALER(1) -> AHB1 / HCLK
		//                                                                      -> /8 -> Systick
		// -> APB_PRESCALER

		// f_in = f_ext / m, f_in should be 2MHz

		// f_out = f_in * p / n;
		// n = input divider, possible values { 2, 4, 6, 8 }
		// p = feedback divider, possible values [192, 432]

		//


		// pll ready => Bit 25 of reg::cr
		// enable pll => Bit 24 of reg::cr

		// hse ready => Bit 17 of reg::cr
		// enable hse => Bit 16 of reg::cr

		// pllq: usb division factor, generate 48MHz pllcfgr[27:24]
		// pllsrc: source clock for main pll, pllcfgr[22], 0 = hsi, 1 = hse
		// pllp: main division factor, generate target_clock pllcfgr[17:16], 0: 2, 1: 4, 2: 6, 3: 8
		// plln: main multi factor, generate target_clock pllcfgr[14:6], 192 <= value <= 432
		// pllm: pll division factor, generate 2MHz, pllcfgr[5:0], 2 <= value

		// ppre2: apb2 divisor (ahb -> apb2), generate 84MHz, cfgr[15:13], 0xx => 1, 100 => 2, 101 => 4, ...
		// ppre2: apb1 divisor (ahb -> apb1), generate 48MHz, cfgr[12:10], 0xx => 1, 100 => 2, 101 => 4, ...
		// hpre: ahb prescaler (system => ahb), generate >= 25MHz for ethernet. cfgr[7:4] 0xxx => 1, 1000 => 2, 1001 => 4, 1010 => 8

		// sws: system clock switch status, cfgr[3:2], 00: hsi, 01: hse, 10: pll
		// sw: system clock switch, cfgr[1:0], 00: hsi, 01: hse, 10 : pll

		// pre-pll divider (f_in = f_oscillator / m)
		static const unsigned int pllm = div_upper(external_clock, 2000000);
		static const unsigned int input_clock = div_rounded(external_clock, pllm);

		// pll input divider (f_out = f_in * plln / pllp)
		static const unsigned int lower_pllp = div_upper(192 * input_clock, target_clock*2)-1;
		static const unsigned int upper_pllp = div_lower(432 * input_clock, target_clock*2)-1;

		// pll return divider
		// since 192*3 > 432 there are only at most 2 possible configurations
		static const unsigned int lower_plln = div_rounded(target_clock, div_rounded(input_clock, (lower_pllp+1)*2));
		static const unsigned int upper_plln = div_rounded(target_clock, div_rounded(input_clock, (upper_pllp+1)*2));

		static const unsigned int lower_diff = cabs(calc_clock(external_clock, pllm, lower_pllp, lower_plln) - target_clock);
		static const unsigned int upper_diff = cabs(calc_clock(external_clock, pllm, upper_pllp, upper_plln) - target_clock);

		// chose p-n-combination with lowest difference to target_clock, if equal prefer smaller input divider.
		static const unsigned int plln = (lower_diff <= upper_diff) ? lower_plln : upper_plln;
		static const unsigned int pllp = (lower_diff <= upper_diff) ? lower_pllp : upper_pllp; // the register encodes 0 as being /2

		static const uint64 result_clock = calc_clock(external_clock, pllm, pllp, plln);

		static const unsigned int pllq = div_upper(external_clock*plln, pllm*48000000);

		static constexpr unsigned int hpre  = encode_exponent( log2_upper(1), 3 );
		static constexpr unsigned int ppre1 = encode_exponent( log2_upper(div_rounded(result_clock, 84000000)), 2);
		static constexpr unsigned int ppre2 = encode_exponent( log2_upper(div_rounded(result_clock, 42000000)), 2);

		static constexpr unsigned int sysreload = result_clock / system_tick_clock;
	};

	template< unsigned int source_clock, unsigned int target_clock, unsigned int system_tick = 1000 >
	void configure_hse()
	{
		// configure pll
		using pll = pll_values< source_clock, target_clock, system_tick >;

		// pll ready => Bit 25 of reg::cr
		// enable pll => Bit 24 of reg::cr

		// hse ready => Bit 17 of reg::cr
		// enable hse => Bit 16 of reg::cr

		// pllq: usb division factor, generate 48MHz pllcfgr[27:24]
		// pllsrc: source clock for main pll, pllcfgr[22], 0 = hsi, 1 = hse
		// pllp: main division factor, generate target_clock pllcfgr[17:16], 0 => 2, 1 => 4 etc.
		// plln: main multi factor, generate target_clock pllcfgr[14:6], 192 <= value <= 432
		// pllm: pll division factor, generate 2MHz, pllcfgr[5:0], 2 <= value

		using namespace fields;

		device.cr <<= cr::hseon(1);
		while(!device.cr.field<cr::hserdy>());

		// Setup main PLL config
		// GCC warns here: operator<<= returns a reference to the written register, but
		// we don't access it, so the dereference is optimized away, there is no
		// read access to the volatile register because of that.


		device.pllcfgr <<= pllcfgr::pllq(pll::pllq) |
						   pllcfgr::pllsrc(1) |
						   pllcfgr::pllp(pll::pllp) |
						   pllcfgr::plln(pll::plln) |
						   pllcfgr::pllm(pll::pllm);

		device.cr <<= cr::pllon(1);  // Enable PLL

        // Set APB1 and APB2 divisors, since their current value might overclock them after system clock source change
		// If current clock is higher than target clock, those prescalers will have higher values, don't touch them
		// If current clock is lower than target clock, those prescalers will have lower value, so replace them prior to change
		auto snapshot = device.cfgr.snapshot();

		const auto apb2_prescaler = snapshot.field<cfgr::ppre2>(); // field<15,13>::read(regs.cfgr);
		const auto apb1_prescaler = snapshot.field<cfgr::ppre1>(); // field<12,10>::read(regs.cfgr);

		if(apb2_prescaler < pll::ppre2) {
			device.cfgr <<= cfgr::ppre2(pll::ppre2);
		}

		if(apb1_prescaler < pll::ppre1) {
			device.cfgr <<= cfgr::ppre1(pll::ppre1);
		}

		while( !device.cr.field<cr::pllrdy>() ); // Wait for PLL stability

		// Set Flash Waitstates
		using namespace flash::fields;
		flash::device.acr = acr::latency(5) | acr::prften(1) | acr::icen(1) | acr::dcen(1);

		// Change main clock source
		device.cfgr <<= cfgr::sw(2);

		while( device.cfgr.field<cfgr::sws>() != 2 ); // wait for pll

		device.cfgr <<= cfgr::hpre(pll::hpre);

		if(apb2_prescaler > pll::ppre2) {
			device.cfgr <<= cfgr::ppre2(pll::ppre2);
		}

		if(apb1_prescaler > pll::ppre1) {
			device.cfgr <<= cfgr::ppre1(pll::ppre1);
		}

		core::device.stk_load = pll::sysreload;
		core::device.stk_val = 0;
		core::device.stk_ctrl =
			core::fields::stk_ctrl::tickint(1) |
			core::fields::stk_ctrl::clksource(1) |
			core::fields::stk_ctrl::enable(1);

		// ppre2: apb2 divisor (ahb -> apb2), generate 84MHz, cfgr[15:13], 0xx => 1, 100 => 2, 101 => 4, ...
		// ppre1: apb1 divisor (ahb -> apb1), generate 48MHz, cfgr[12:10], 0xx => 1, 100 => 2, 101 => 4, ...
		// hpre: ahb prescaler (system => ahb), generate >= 25MHz for ethernet. cfgr[7:4] 0xxx => 1, 1000 => 2, 1001 => 4, 1010 => 8

		// sws: system clock switch status, cfgr[3:2], 00: hsi, 01: hse, 10: pll
		// sw: system clock switch, cfgr[1:0], 00: hsi, 01: hse, 10 : pll

		// wait until hse is ready
		// enable pll
		// wait until pll is ready
		// switch to pll

		// setup systick timer
		// activate systick timer & interrupt
	}

}

#endif /* RCC_HPP_ */
