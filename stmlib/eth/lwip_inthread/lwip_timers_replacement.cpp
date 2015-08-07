#include <stmlib/eth/lwip/lwip_thread.hpp>

/*
#include <stm/util/types.hpp>
#include <stm/util/static_heap.hpp>
#include <ch.h>
#include <thread.hpp>
#include <stm/util/pool.hpp>
#include <microptp/util/static_union.hpp>
*/

/*
	Replacement for lwip timers
*/

extern "C" {

	// These are called from Lwip thread context.

	void sys_timeout(uint32 when, void(*fn)(void*), void* arg)
	{
		// convert to systicks
		when = chTimeNow() + MS2ST(when);

		eth::lwip::LwipThread::get().add_timeout_thread(when, fn, arg);
	}

	void sys_untimeout(void (*handler)(void*), void *arg)
	{
		eth::lwip::LwipThread::get().remove_timeout_thread(handler, arg);
	}

}



