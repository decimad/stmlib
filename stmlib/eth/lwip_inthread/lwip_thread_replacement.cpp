#include "lwipopts.h"
#include <lwip/tcpip.h>
#include <netif/etharp.h>
#include <stmlib/eth/lwip/lwip_thread.hpp>

/*
	Replacement for lwip thread message sending.
*/

namespace eth { namespace lwip {

	// copied from lwip/tcpip.c
	void handle_tcpip_message(tcpip_msg& msg)
	{
		switch (msg.type) {
#if LWIP_NETCONN
		case TCPIP_MSG_API:
			LWIP_DEBUGF(TCPIP_DEBUG, ("tcpip_thread: API message %p\n", (void *)msg));
			msg.msg.apimsg->function(&(msg.msg.apimsg->msg));
			break;
#endif /* LWIP_NETCONN */

#if !LWIP_TCPIP_CORE_LOCKING_INPUT
		case TCPIP_MSG_INPKT:
			LWIP_DEBUGF(TCPIP_DEBUG, ("tcpip_thread: PACKET %p\n", (void *)msg));
#if LWIP_ETHERNET
			if (msg.msg.inp.netif->flags & (NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET)) {
				ethernet_input(msg.msg.inp.p, msg.msg.inp.netif);
			} else
#endif /* LWIP_ETHERNET */
			{
				ip_input(msg.msg.inp.p, msg.msg.inp.netif);
			}
			break;
#endif /* LWIP_TCPIP_CORE_LOCKING_INPUT */

#if LWIP_NETIF_API
		case TCPIP_MSG_NETIFAPI:
			LWIP_DEBUGF(TCPIP_DEBUG, ("tcpip_thread: Netif API message %p\n", (void *)msg));
			msg->msg.netifapimsg->function(&(msg->msg.netifapimsg->msg));
			break;
#endif /* LWIP_NETIF_API */

#if LWIP_TCPIP_TIMEOUT
		case TCPIP_MSG_TIMEOUT:
			LWIP_DEBUGF(TCPIP_DEBUG, ("tcpip_thread: TIMEOUT %p\n", (void *)msg));
			sys_timeout(msg.msg.tmo.msecs, msg.msg.tmo.h, msg.msg.tmo.arg);
			break;
		case TCPIP_MSG_UNTIMEOUT:
			LWIP_DEBUGF(TCPIP_DEBUG, ("tcpip_thread: UNTIMEOUT %p\n", (void *)msg));
			sys_untimeout(msg.msg.tmo.h, msg.msg.tmo.arg);
			break;
#endif /* LWIP_TCPIP_TIMEOUT */

		case TCPIP_MSG_CALLBACK:
			LWIP_DEBUGF(TCPIP_DEBUG, ("tcpip_thread: CALLBACK %p\n", (void *)msg));
			msg.msg.cb.function(msg.msg.cb.ctx);
			break;

		case TCPIP_MSG_CALLBACK_STATIC:
			LWIP_DEBUGF(TCPIP_DEBUG, ("tcpip_thread: CALLBACK_STATIC %p\n", (void *)msg));
			msg.msg.cb.function(msg.msg.cb.ctx);
			break;

		default:
			LWIP_DEBUGF(TCPIP_DEBUG, ("tcpip_thread: invalid message: %d\n", msg->type));
			LWIP_ASSERT("tcpip_thread: invalid message", 0);
			break;
		}
	}

} }

extern "C" {




}