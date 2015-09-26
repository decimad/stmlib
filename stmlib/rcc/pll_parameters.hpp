#ifndef STMLIB_RCC_PLL_PARAMETERS_HPP__
#define STMLIB_RCC_PLL_PARAMETERS_HPP__

namespace rcc {

	namespace detail {

		constexpr int log2_floor_float(double value) {
			return
				(value == 2) ? 1 :
				((value > 2) ? (1 + log2_floor_float(value / 2)) : 0);
		}

		constexpr int log2_ceil_float(double value) {
			return
				(value == 2) ? 1 :
				((value > 2) ? (1 + log2_ceil_float(value / 2)) : 1);
		}

		constexpr int log2_floor(double value)
		{
			return
				(value == 1) ? 0 :
				((value >= 1) ? log2_floor_float(value) : (-log2_ceil_float(1. / value)));
		}

		constexpr int log2_ceil(double value)
		{
			return
				(value == 1) ? 0 :
				((value > 1) ? log2_ceil_float(value) : (-log2_floor_float(1. / value)));
		}

		constexpr double myabs(double val)
		{
			return (val >= 0) ? val : -val;
		}

		constexpr int clamp(int val, int min, int max)
		{
			return (val < min) ? min : (
				(val > max) ? max : val
				);
		}

		constexpr int next_power_of_two(int value, int shifts = 0)
		{
			return (value > 1) ? next_power_of_two(value >> 1, shifts + 1)
				: (1 << (shifts + 1));
		}

		constexpr int prior_power_of_two(int value, int shifts = 0)
		{
			return (value > 1) ? prior_power_of_two(value >> 1, shifts + 1)
				: (1 << (shifts));
		}

		// sol
		// literal (constexpr) struct holding a PLL solution
		// with prescaler factors for A, P, M, N
		// note: these factors are not yet encoded according to the reference manual spec
		struct sol {
			constexpr sol(int n_, int m_, int p_, int a_)
				: n(n_), m(m_), p(p_), a(a_) {}

			constexpr double calc(double src) const
			{
				return (src * n) / (m*p*a);
			}

			int n;
			int m;
			int p;
			int a;
		};

		// calculate result frequency, given oscillator frequency
		// and a PLL solution
		constexpr double eval(sol s, double src)
		{
			return (src * s.n) / (s.m*s.p*s.a);
		}

		// Compare 2 PLL solutions and return the better one
		constexpr sol better_sol(double source, double dest, const sol a, const sol b)
		{
			return myabs(eval(a, source) - dest) < myabs(eval(b, source) - dest) ?
				a : b;
		}

		//
		// N search
		//

		// Natural encoding, must be in interval [192, 432] (rm page 223)

		constexpr sol n_search_pruned(double source, double dest, int m, int a, int p, int n, int nmax, sol best)
		{
			return (n > nmax) ?
				best
				:
				// next iteration with n+1
				n_search_pruned(source, dest, m, a, p, n + 1, nmax, better_sol(source, dest, sol(n, m, p, a), best))
				;
		}

		constexpr int prune_n_min(double source, double dest, int m, int a, int p)
		{
			return clamp(dest * (m*p*a) / source, 192, 432);
		}

		constexpr int prune_n_max(double source, double dest, int m, int a, int p)
		{
			return clamp(dest * (m*p*a) / source + 1, 192, 432);
		}

		constexpr sol n_search(double source, double dest, int m, int a, int p, sol best)
		{
			return n_search_pruned(source, dest, m, a, p, prune_n_min(source, dest, m, a, p), prune_n_max(source, dest, m, a, p), best);
		}

		//
		// P search
		//
		
		// Encoding: (rm page 223)
		// 00 => / 2
		// 01 => / 4
		// ...
		// 11 => / 8
		constexpr int encode_p(int p)
		{
			return p / 2 - 1;
		}

		constexpr sol p_search_pruned(double source, double dest, int m, int a, int p, int pmax, sol best)
		{
			return (p > pmax) ?
				best :
				// next iteration with p+2
				p_search_pruned(source, dest, m, a, p + 2, pmax, better_sol(source, dest, best, n_search(source, dest, m, a, p, best)));
		}

		constexpr int prune_p_min(double source, double dest, int m, int a)
		{
			return clamp(int(source * 192 / (a * m) / dest) / 2 * 2, 2, 8);
		}

		constexpr int prune_p_max(double source, double dest, int m, int a)
		{
			return clamp(int(source * 432 / (a * m) / dest + 1) / 2 * 2, 2, 8);
		}

		constexpr sol p_search(double source, double dest, int m, int a, sol best)
		{
			return p_search_pruned(source, dest, m, a, prune_p_min(source, dest, m, a), prune_p_max(source, dest, m, a), best);
		}

		//
		// A search (AHB prescaler)
		// We prefer smaller A because that leads to smaller clock frequencies behind the pll device
	
		// A must be a power of two.
		// Encoding: (rm page 226)
		// 0xxx => no prescaling
		// 1000 => /2
		// 1001 => /4
		// 1010 => /8
		// 1011 => /16
		// 1100 => /64   CAUTION, /32 is missing
		// 1101 => /128
		// 1110 => /256
		// 1111 => /512
		constexpr int encode_a(int a) {
			// handle the missing / 32
			return (a == 1) ? 0 : 8 | (
				(a <= 16) ? 
					log2_ceil(a) : (log2_ceil(a)-1)
			);
		}
		
		constexpr int next_a(int a) {
			return (a == 16) ? 64 : (a * 2);
		}

		constexpr int avoid_gap_min(int a) {
			return (a == 32) ? 16 : a;
		}

		constexpr int avoid_gap_max(int a) {
			return (a == 32) ? 64 : a;
		}

		constexpr sol a_search_pruned(double source, double dest, int m, int a, int amax, sol best)
		{
			return (a > 512) ?
				best :
				// next iteration with a*2
				a_search_pruned(source, dest, m, next_a(a), amax, better_sol(source, dest, best, p_search(source, dest, m, a, best)));
		}

		// A must be a power of two => after pruning we need to round to next/prior power of two
		constexpr int prune_a_min(double source, double dest, int m)
		{
			return avoid_gap_min(clamp(prior_power_of_two(source * 192 / (512 * m) / dest), 1, 512));
		}

		constexpr int prune_a_max(double source, double dest, int m)
		{
			return avoid_gap_max(clamp(next_power_of_two(source * 432 / (1 * m) / dest), 1, 512));
		}

		constexpr sol a_search(double source, double dest, int m, sol best)
		{
			return a_search_pruned(source, dest, m, prune_a_min(source, dest, m), prune_a_max(source, dest, m), best);
		}

		//
		// M search
		//

		// Natural encoding, must be in interval [2, 63] (rm page 223)
		// Also 1MHz <= source / m <= 2MHz, where source / m = 2MHz is preferred.

		constexpr sol m_search_pruned(double source, double dest, int m, int mmax, sol best)
		{
			return (m > mmax) ?
				best :
				// next iteration with m+1
				m_search_pruned(source, dest, m + 1, mmax, better_sol(source, dest, best, a_search(source, dest, m, best)))
				;
		}

		constexpr int prune_m_min(double source)
		{
			return clamp(source / 2000000, 2, 63);
		}

		constexpr int prune_m_max(double source)
		{
			return clamp(source / 1000000, 2, 63);
		}

		constexpr sol m_search(double source, double dest, sol best)
		{
			return m_search_pruned(source, dest, prune_m_min(source), prune_m_max(source), best);
		}
		

		// Entry function for PLL-Solution search
		// m has a hard restriction, a should be as small as possible, that's why we search in the order
		// m => a => p => n
		constexpr sol calculate_pll_config(double source_freq, double dest_freq)
		{
			return m_search(source_freq, dest_freq, sol(192, 2, 2, 1));
		}

		//
		// 48 MHz domain divider
		//

		constexpr int calculate_q(double source_freq, sol solution) {
			return source_freq * solution.n / solution.m / 48000000;
		}

		//
		// APB prescalers
		//
	
		constexpr int encode_apb_prescaler(double value) {
			return (value <= 1) ? 0 : (4 | (detail::log2_ceil(value) - 1));
		}

		// Need to retain a maximum of 84 MHz on the fast APB
		// The fast APB prescaler can onle be a power of 2 (see Reference manual p. 225)
		constexpr int calculate_fast_apb(double source_freq, sol solution)
		{
			return encode_apb_prescaler(source_freq * solution.n / solution.m / solution.p / solution.a / 84000000);
		}

		// Need to retain a maximum of 42 MHz on the slow APB
		// The slow APB prescaler can onle be a power of 2 (see Reference manual p. 225)
		constexpr int calculate_slow_apb(double source_freq, sol solution)
		{
			return encode_apb_prescaler(source_freq * solution.n / solution.m / solution.p / solution.a / 42000000);
		}

		//
		// Systick reload value
		//

		constexpr int calculate_systick_reload(double source_clock, sol pll, int system_tick_frequency)
		{
			return eval(pll, source_clock) / 8 / system_tick_frequency;
		}

	}

}


#endif
