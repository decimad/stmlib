/*
 * trace.hpp
 *
 *  Created on: 26.12.2014
 *      Author: Michael
 */

#ifndef INCLUDE_STM_TRACE_H_
#define INCLUDE_STM_TRACE_H_

/*
 * This code needs to accessible from C and C++ libs.
 *
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

void trace_init();

void trace_write( unsigned char port, const char* data, unsigned int size );

int trace_printf( unsigned char port, const char* fmt, ... );	// From normal thread code
int trace_printf_i( unsigned char port, const char* fmt, ... );	// From interrupt service routines

#ifdef __cplusplus
}
#endif


#endif /* INCLUDE_STM_TRACE_H_ */
