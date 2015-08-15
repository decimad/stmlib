/*
 * eth_lwip_util.cpp
 *
 *  Created on: 20.12.2014
 *      Author: Michael
 */

#include <stmlib_config.hpp>
#ifdef STMLIB_LWIP_ONETHREAD

#include <stmlib/eth/lwip_onethread/dma.hpp>

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
#include <stmlib/trace.h>

#ifdef _DEBUG
#include <stmlib/trace.h>
#endif

namespace eth { namespace lwip {

	// Although the RX-normal descriptor documentation states
	// that PTP-timestamp would be written into RDES2 and RDES3,
	// later sections state that you absolutely need to use
	// enhanced descriptors.

	// We're really relying on the custom buffers being stored
	// in pools in rx and tx, tx can handle
	// lwip-pbufs and pool-custom-buffers though.

	struct receive_descriptor : public eth::enhanced_rx_dma_descriptor {
		void arm(custom_buffer_ptr buffer) {
			buffer->reset_rx();
			buffer_ = std::move(buffer);
			rdes1 <<= eth::fields::rdes1::rbs1(buffer_->capacity());
			rdes2 = reinterpret_cast<uint32>(buffer_->data());
			rdes0 = eth::fields::rdes0::own(1);
			if (eth::device.dmachrdr.get() == reinterpret_cast<uint32>(this)) {
				// the dma engine halted, try to resume
				resume_rx();
			}
		}

		void set_next(receive_descriptor* next) {
			rdes3 = reinterpret_cast<uint32>(next);
			rdes1 <<= eth::fields::rdes1::rch(1);
		}

		const custom_buffer_ptr& get_buffer() {
			return buffer_;
		}

		custom_buffer_ptr release_buffer() {
			return std::move(buffer_);
		}

		bool is_host() {
			return rdes0.field<eth::fields::rdes0::own>() == 0;
		}

		bool is_first() {
			return rdes0.field<eth::fields::rdes0::fs>() == 1;
		}

		bool is_last() {
			return rdes0.field<eth::fields::rdes0::ls>() == 1;
		}

		receive_descriptor* get_next() {
			return reinterpret_cast<receive_descriptor*>(rdes3.get());
		}

		void transfer_timestamp() {
			if (rdes4.field<eth::fields::rdes4::pmt>() != 0) {
				const uint64 timestamp = (static_cast<uint64>(rdes7.get()) << 32) | rdes6.get();
				buffer_->enhance().to_type<uint64>(ptp_hardware_to_logical(timestamp));
			}
		}

		custom_buffer_ptr buffer_;
	};

	struct transmit_descriptor : public eth::enhanced_tx_dma_descriptor
	{
		transmit_descriptor()
			: buffer_(0)
		{
		}

		// First must be valid! (Otherwise call clear!)
		void set_buffer(pbuf* buf) {
			uint32 buf1_size = buf->len;
			buffer_ = buf;
			tdes2 = reinterpret_cast<uint32>(buf->payload);
			tdes1 = eth::fields::tdes1::tbs1(buf1_size) | eth::fields::tdes1::tbs2(0);
		}

		pbuf* get_buffer() {
			return buffer_;
		}

		void set_next(transmit_descriptor* next)
		{
			tdes3 = reinterpret_cast<uint32>(next);
			tdes0 <<= eth::fields::tdes0::tch(1);
		}

		transmit_descriptor* get_next()
		{
			return reinterpret_cast<transmit_descriptor*>(tdes3.get());
		}

		void clear()
		{
			buffer_ = 0;
			tdes1 = 0;
		}

		bool is_host() {
			return tdes0.field<eth::fields::tdes0::own>() == 0;
		}

		uint64 time() const
		{
			return ptp_hardware_to_logical((static_cast<uint64>(tdes7.get()) << 32) | tdes6.get());
		}

		//
		// Note: the handling for custom buffers
		//       currently relies on them not being chained!
		bool is_custom_buffer()
		{
			return (buffer_->flags & PBUF_FLAG_IS_CUSTOM) != 0;
		}

		custom_buffer_ptr release_custom()
		{
			auto result = ptr_from_pbuf(buffer_);
			buffer_ = nullptr;
			return result;
		}

		// could be a custom buffer
		pbuf* buffer_;
	};





	// =================================
	// Custom buffers pool
	//

	using rx_pool_type = ulib::pool< custom_buffer_type, 32 >;
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
	ulib::intrusive_ringbuffer<receive_descriptor, 8> rx_descriptor_ring;

	void intrusive_ring_set_next( receive_descriptor* descr, receive_descriptor* next )
	{
		descr->set_next(next);
	}

	receive_descriptor* intrusive_ring_get_next( receive_descriptor* descr )
	{
		return descr->get_next();
	}

	void dma_receive_init() {
		receive_descriptor* desc;
		custom_buffer_ptr buff;
		while((desc = rx_descriptor_ring.idle_front()) && (buff = rx_buffer_pool.make())) {
			desc->arm(std::move(buff));
			rx_descriptor_ring.commit();
		}

		eth::device.dmardlar = reinterpret_cast<uint32>(rx_descriptor_ring.committed_front());
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
//		chSysLock();
		if(!rx_descriptor_ring.full()) {
			auto* desc = rx_descriptor_ring.idle_front();
			rx_descriptor_ring.commit();
			desc->arm(std::move(ptr));
		}
//		chSysUnlock();
	}

	custom_buffer_ptr request_buffer()
	// called from any thread
	{
//		chSysLock();
		auto result = rx_buffer_pool.make();
//		chSysUnlock();
		return result;
	}

	// Release committed front and try to recommit
	void rx_descriptor_recycle()
	// called from driver thread only
	{
//		chSysLock();
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
//		chSysUnlock();
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

	ulib::intrusive_ringbuffer< transmit_descriptor, 8 > tx_descriptor_ring;

	void intrusive_ring_set_next( transmit_descriptor* descr, transmit_descriptor* next )
	{
		descr->set_next(next);
	}

	transmit_descriptor* intrusive_ring_get_next( transmit_descriptor* descr )
	{
		return descr->get_next();
	}

	void dma_transmit_init() {
		eth::device.dmatdlar = reinterpret_cast<uint32>(tx_descriptor_ring.idle_front());
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
		dma_receive_init();
		dma_transmit_init();
	}

} }

#endif
