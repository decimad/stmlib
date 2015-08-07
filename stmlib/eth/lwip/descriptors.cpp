/*
 * eth_lwip_util.cpp
 *
 *  Created on: 20.12.2014
 *      Author: Michael
 */

#include <stmlib/eth/lwip/descriptors.hpp>

#include <microlib/pool.hpp>
#include <microlib/intrusive_ringbuffer.hpp>
#include <microlib/intrusive_stack.hpp>

#include <lwip/def.h>
#include <lwip/sys.h>
#include <lwip/tcpip.h>
#include <lwip/udp.h>
#include <lwip/opt.h>
#include <lwip/err.h>
#include <netif/etharp.h>
#include <lwip/dhcp.h>
#include <lwip/pbuf.h>

#include <stmlib/eth.hpp>

#ifdef _DEBUG
#include <stmlib/trace.h>
#endif

namespace eth { namespace lwip {

	// =================================
	// Custom buffers pool
	//

	using rx_pool_type = util::pool< custom_buffer_type, 32 >;
	rx_pool_type rx_buffer_pool;

	void reuse_buffer( custom_buffer_ptr ptr );

	custom_buffer_ptr ptr_from_pbuf(pbuf* buf)
	{
		return custom_buffer_ptr::from_type_ptr(reinterpret_cast<custom_buffer_type*>(buf));
	}

	void custom_buffer_release( pbuf* buf )
	{
		reuse_buffer(ptr_from_pbuf(buf));
	}

	void custom_buffer_reuse_chain(custom_buffer_ptr ptr)
	{
		while(ptr) {
			auto next = custom_buffer_ptr::from_type_ptr(ptr->get_next());
			reuse_buffer(std::move(ptr));
			ptr = std::move(next);
		}
	}

	// =================================
	// RX DMA Ring
	//

	struct lwip_stats {
		unsigned int tx_count = 0;
		unsigned int rx_count = 0;
	};

	lwip_stats stats;

	const uint8 num_rx_dma_descriptors = 8;
	util::intrusive_ringbuffer<receive_descriptor, 8> rx_descriptor_ring;

	void intrusive_ring_set_next( receive_descriptor* descr, receive_descriptor* next )
	{
		descr->set_next(next);
	}

	receive_descriptor* intrusive_ring_get_next( receive_descriptor* descr )
	{
		return descr->get_next();
	}

	receive_descriptor* receive_init() {
		receive_descriptor* desc;
		custom_buffer_ptr buff;
		while((desc = rx_descriptor_ring.idle_front()) && (buff = rx_buffer_pool.make())) {
			desc->arm(std::move(buff));
			rx_descriptor_ring.commit();
		}

		return rx_descriptor_ring.committed_front();
	}

	// =================================================================================================
	// =================================================================================================
	// Critical intersection following
	// Buffers can be released from all kind of threads, descriptors are recycled on the lwip driver
	// thread. Currently guarded by chSysLock... but this introduces jitter... Maybe there's a better
	// solution? (info: Mutex and (Binary-)Semaphore use chSysLock in chibi)

	void reuse_buffer( custom_buffer_ptr ptr )
	// called from any thread
	{
		chSysLock();
		if(!rx_descriptor_ring.full()) {
			auto* desc = rx_descriptor_ring.idle_front();
			rx_descriptor_ring.commit();
			desc->arm(std::move(ptr));
		}
		chSysUnlock();
	}

	custom_buffer_ptr request_buffer()
	// called from any thread
	{
		chSysLock();
		auto result = rx_buffer_pool.make();
		chSysUnlock();
		return result;
	}

	// Release committed front and try to recommit
	void rx_descriptor_recycle()
	// called from driver thread only
	{
		chSysLock();
		auto buffer = rx_buffer_pool.make();

		rx_descriptor_ring.release();
		if(buffer) {
			auto* descr = rx_descriptor_ring.idle_front();
			rx_descriptor_ring.commit();
			descr->arm(std::move(buffer));
		} else {
#ifdef _DEBUG
			trace_printf(0, "RX Buffers drained!\n");
#endif
		}
		chSysUnlock();
	}

	// =================================================================================================
	// =================================================================================================


	void spawn_frame(struct netif* interface, custom_buffer_type* buff);

	namespace {
		custom_buffer_type* rx_assembly_front = nullptr;
		custom_buffer_type* rx_assembly_back  = nullptr;
		uint16            rx_assembly_size  = 0;
	}

	void rx_walk_descriptors( struct netif* interface ) {
		using namespace eth::fields;

		// Note: while we're walking the descriptors, the dma is still running
		// and we can end up reading the first segments of a frame that is not yet
		// completely transmitted.
		// That means we can enter and leave this function in
		// intermediate states.

		while( !rx_descriptor_ring.empty() && rx_descriptor_ring.committed_front()->is_host() ) {
			// Take snapshot of control register, copy buffer address and hand it off for reuse
			auto* current_descriptor = rx_descriptor_ring.committed_front();
			auto  snapshot = current_descriptor->rdes0.snapshot();
			current_descriptor->transfer_timestamp();
			custom_buffer_ptr current_buffer = current_descriptor->release_buffer();
			rx_descriptor_recycle();	// this must work in 100% of all cases concurrently to the pbuf_free function, so I keep them close together

			// Do segment inspection
			auto bare_ptr = current_buffer.release_type();	// unsafe from here...

			if(snapshot.field<rdes0::fs>()) {
				rx_assembly_front = rx_assembly_back = bare_ptr;
				rx_assembly_size  = 0;
			} else {
				rx_assembly_back->pbuf.next = &bare_ptr->pbuf;
				rx_assembly_back = bare_ptr;
			}

			auto new_total = snapshot.field<eth::fields::rdes0::fl>();
			bare_ptr->pbuf.len = new_total - rx_assembly_size;
			rx_assembly_size = new_total;

			if(snapshot.field<eth::fields::rdes0::ls>()) {
				rx_assembly_back->pbuf.next = nullptr;
				auto* ptr = rx_assembly_front;
				while(ptr) {
					ptr->pbuf.tot_len = rx_assembly_size;
					ptr = ptr->get_next();
				}
				++stats.rx_count;
				spawn_frame(interface, rx_assembly_front);
				rx_assembly_front = rx_assembly_back = nullptr;
				rx_assembly_size = 0;
			}
		}
	}

	void spawn_frame(struct netif* interface, custom_buffer_type* buff)
	{
		struct eth_hdr* ptr = static_cast<struct eth_hdr*>(buff->data());

		switch (htons(ptr->type)) {
			/* IP or ARP packet? */
			case ETHTYPE_IP:
				interface->input(&buff->pbuf, interface);
				break;
			case ETHTYPE_ARP:
			  /* full packet send to tcpip_thread to process */
				interface->input(&buff->pbuf, interface);
				break;
			default:
				custom_buffer_reuse_chain(custom_buffer_ptr::from_type_ptr(buff));
		}
	}

	// =================================
	// TX DMA Ring
	//

	util::intrusive_ringbuffer< transmit_descriptor, 8 > tx_descriptor_ring;

	void intrusive_ring_set_next( transmit_descriptor* descr, transmit_descriptor* next )
	{
		descr->set_next(next);
	}

	transmit_descriptor* intrusive_ring_get_next( transmit_descriptor* descr )
	{
		return descr->get_next();
	}

	transmit_descriptor* transmit_init() {
		return tx_descriptor_ring.idle_front();
	}

	msg_t transmit( pbuf* data ) {
	// -> tcpip thread only (-> not called concurrently)
	// accesses tx_descriptor_ring.idle_front()
	//          tx_descriptor_committed_front()

		if(data == nullptr || tx_descriptor_ring.full()) {
			return ERR_MEM;
		}

		pbuf* current_pbuf = data;

		transmit_descriptor* current_descr = tx_descriptor_ring.idle_front();

		while( current_pbuf ) {
			current_descr->set_buffer(current_pbuf);

			auto *next_buf   = current_pbuf->next;
			auto *next_descr = current_descr->get_next();

			if( next_buf &&
					(next_descr == tx_descriptor_ring.idle_front()		 // we walked around the ring
				  || next_descr == tx_descriptor_ring.committed_front()) // we walked to the end of free descriptors
			   ) {
				return ERR_MEM;
			} else {
				current_descr = next_descr;
				current_pbuf = next_buf;
			}
		}

		// we got a complete frame together
		using namespace eth::fields;

		auto* descr = tx_descriptor_ring.idle_front();

		if(data->next == nullptr) {
			// fast case
			tx_descriptor_ring.commit();
			descr->tdes0 = tdes0::cic(3) | tdes0::tch(1) | tdes0::fs(1) | tdes0::ls(1) | tdes0::ic(1) | tdes0::own(1) | tdes0::ttse(1);
		} else {
			tx_descriptor_ring.commit();
			descr->tdes0 = tdes0::cic(3) | tdes0::tch(1) | tdes0::fs(1) | tdes0::own(1);

			while( descr->get_next() != current_descr ) {
				tx_descriptor_ring.commit();
				descr->tdes0 = tdes0::tch(1) | tdes0::own(1);
			}

			tx_descriptor_ring.commit();
			descr->tdes0 = tdes0::tch(1) | tdes0::own(1) | tdes0::ls(1) | tdes0::ic(1);
		}

		eth::device.dmasr <<= dmasr::nis(1) | dmasr::tbus(1);
		eth::device.dmatpdr = 1;

		// everything went well, hold on to the pbuf.
		++stats.tx_count;
		pbuf_ref(data);
		return ERR_OK;
	}

	void tx_walk_descriptors() {
	// -> lwip thread (not called concurrently)

		// Although the pbufs are still chained inside the descriptors,
		// multiple chains might be sequenced, so free everything piecewise
		while( tx_descriptor_ring.committed_front() && tx_descriptor_ring.committed_front()->is_host() ) {
			auto* descriptor = tx_descriptor_ring.committed_front();
			if(descriptor->is_custom_buffer()) {
				auto custom_ptr = descriptor->release_custom();
				uint64 timestamp = descriptor->time();
				tx_descriptor_ring.release();
				if(custom_ptr->has<transmit_callback>()) {
					// FIXME: We really need to synchronize here.
					// (Postponed until the custom buffer payload is
					//  reimplemented in a way that doesn't use static union
					//  so enhancements decrease the available buffer space
					//  in a more customizable fashion)
					auto& clb = custom_ptr->access<transmit_callback>();
					clb.func(timestamp, std::move(custom_ptr));
				}
			} else {
				pbuf* buf = descriptor->get_buffer();
				tx_descriptor_ring.release();
				buf->next = nullptr;
				buf->ref = 1;
				pbuf_free(buf);
			}
		}
	}

	void lwip_dma_init() {
		receive_init();
		transmit_init();
	}

} }


