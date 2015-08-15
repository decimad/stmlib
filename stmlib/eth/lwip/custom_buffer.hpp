#ifndef PTPCLIENT_STM_ETH_ETH_LWIP_CUSTOM_BUFFER_HPP_
#define PTPCLIENT_STM_ETH_ETH_LWIP_CUSTOM_BUFFER_HPP_

#include <microlib/pool.hpp>
#include <microlib/variant.hpp>
#include <microlib/functional.hpp>
#include <lwip/pbuf.h>

namespace eth { namespace lwip {

	using pbuf_custom = ::pbuf_custom;

	void custom_buffer_release( pbuf* buf );

	template< size_t Size >
	struct custom_buffer;

	using custom_buffer_type = custom_buffer<256>;
	using custom_buffer_ptr = ulib::pool_ptr< custom_buffer_type >;

	struct transmit_callback
	{
		// Fixme: we're currently relying on the callback destination
		// to wait until operations complete.
		ulib::function<void(uint64 time, custom_buffer_ptr)> func;
	};

	template< size_t Size /*, size_t Alignment = 4*/ >
	struct custom_buffer :
			public pbuf_custom
	{
		custom_buffer()
		{
			reset();
		}

		// Only PBUF_RAM and PBUF_POOL support relocating the payload pointer
		// to accomodate for udp and ethernet header addition during
		// sending operation.
		void reset()
		{
			pbuf_custom::custom_free_function = custom_buffer_release;
			dishance();
		}

		void reset_rx()
		{
			reset();
			pbuf_alloced_custom(PBUF_RAW, 0, PBUF_RAM, static_cast<pbuf_custom*>(this), &data_, Size);
		}

		void reset_udp()
		{
			reset();
			pbuf_alloced_custom(PBUF_TRANSPORT, 0, PBUF_RAM, static_cast<pbuf_custom*>(this), &data_, Size);
		}

		custom_buffer* get_next() {
			return reinterpret_cast<custom_buffer*>(pbuf.next);
		}

		void* data() {
			return pbuf.payload;
		}

		const void* data() const {
			return pbuf.payload;
		}

		uint16 size() const {
			return pbuf.len;
		}

		uint16 capacity() const {
			return is_enhanced() ? ((char*)enhanced() - (char*)&data_) : Size;
		}

		//
		// it would really be smarter not to use a static_union here
		// but a custom implementation that only strips the space needed
		// for the current stored type, in that case buffers used for
		// transmission could be sized accordingly and could store
		// a larger amount of enhancement data without blocking rx buffers.
		//
		using enhancement_type = ulib::static_union<uint64, transmit_callback>;

		bool is_enhanced() const {
			return pbuf.type == PBUF_POOL;
		}

		template< typename Type >
		bool has() const
		{
			return is_enhanced() && enhanced()->template is<Type>();
		}

		template< typename Type >
		Type& access()
		{
			return enhanced()->template as<Type>();
		}

		void dishance()
		{
			if(is_enhanced()) {
				enhancement_type* ptr = enhanced_ptr();
				ptr->~enhancement_type();
				pbuf.len = Size;
				pbuf.type = PBUF_RAM;
			}
		}

		enhancement_type* enhanced_ptr()
		{
			char* ptr = (reinterpret_cast<char*>(&data_) + sizeof(data_)) - sizeof(enhancement_type);
			return reinterpret_cast<enhancement_type*>(reinterpret_cast<uint32>(ptr) & ~(std::alignment_of<enhancement_type>::value-1));
		}

		enhancement_type* enhanced()
		{
			return enhanced_ptr();
		}

		const enhancement_type* enhanced() const
		{
			return const_cast<custom_buffer*>(this)->enhanced_ptr();
		}

		enhancement_type& enhance()
		{
			auto* ptr = enhanced_ptr();
			if(!is_enhanced()) {
				pbuf.len = reinterpret_cast<char*>(ptr) - reinterpret_cast<char*>(data());
				new (ptr) enhancement_type();
				pbuf.type = PBUF_POOL;
			}
			return *ptr;
		}

		using storage_type = typename std::aligned_storage<Size, 4 /*Alignment*/>::type;
		storage_type data_;
	};

	custom_buffer_ptr request_buffer();
	void custom_buffer_reuse_chain(custom_buffer_ptr ptr);
	custom_buffer_ptr ptr_from_pbuf(pbuf*);

} }

#endif /* PTPCLIENT_STM_ETH_ETH_LWIP_CUSTOM_BUFFER_HPP_ */
