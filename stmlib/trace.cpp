/*
 * trace.c
 *
 *  Created on: 26.12.2014
 *      Author: Michael
 */

#include <ch.h>
#include <stdarg.h>
#include <stdio.h>
#include <stmlib/itm_registers.hpp>
#include <stmlib/trace.h>


namespace
{

    char buffer[256];

}

extern "C"
{

    void trace_init()
    {
        itm::device.tcr = itm::fields::tcr::enable(1);
        itm::device.ter = 1;
    }

    void trace_write(unsigned char port, const char *data, unsigned int size)
    {
        for (unsigned int i = 0; i < size; ++i)
        {
            // Wait for readyness
            while (itm::device.port[port].get() == 0)
                ;
            itm::device.port[port].byte<0>() = *data++;
        }
    }

    int trace_printf(unsigned char port, const char *fmt, ...)
    {
        va_list arglist;

        // We're using a global buffer, need to synchronize
        chSysLock();

        va_start(arglist, fmt);
        int ret = vsnprintf(buffer, sizeof(buffer), fmt, arglist);
        va_end(arglist);

        if (ret > 0)
        {
            trace_write(port, buffer, (size_t)ret);
        }

        chSysUnlock();

        return ret;
    }

    int trace_printf_i(unsigned char port, const char *fmt, ...)
    {
        va_list arglist;

        // We're using a global buffer, need to synchronize
        chSysLockFromISR();

        va_start(arglist, fmt);
        int ret = vsnprintf(buffer, sizeof(buffer), fmt, arglist);
        va_end(arglist);

        if (ret > 0)
        {
            trace_write(port, buffer, (size_t)ret);
        }

        chSysUnlockFromISR();
        return ret;
    }
}
