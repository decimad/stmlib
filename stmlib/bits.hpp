/*
 * util.hpp
 *
 *  Created on: 24.11.2014
 *      Author: Michael
 */

#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <type_traits>
#include <cstdint>
#include <stmlib/stmtypes.hpp>

/*
 * Todo:
 * - field should not directly derive from masked_value, since you could use one-bitters in a bitband operation and that
 *   would mean to unshift the value again.
 *
 */

namespace util {

	// Helper templates to check for memory access alignment issues
	template< typename T, unsigned int address > struct check_alignment;
	template< unsigned int address > struct check_alignment< unsigned int, address > { static_assert((address&3) == 0, "misaligned access!"); };
	template< int address > struct check_alignment< int, address > { static_assert(address&3 == 0, "misaligned access!"); };
	template< unsigned int address > struct check_alignment< unsigned short, address > { static_assert((address&1) == 0, "misaligned access!"); };
	template< int address > struct check_alignment< short int, address > { static_assert(address&1 == 0, "misaligned access!"); };
	template< unsigned int address > struct check_alignment< unsigned char, address > {};
	template< int address > struct check_alignment< signed char, address > {};

	//
	// Convert constant address to pointer
	//

	template< typename T, unsigned int address >
	struct pointer {
		constexpr static T* const value = reinterpret_cast<T*>(address);
	};

	//
	// Convert constant address to reference
	//

#if __GNUC__ == 4 && __GNUC_MINOR__ == 9

#else

	template< typename T, uint32 address >
	struct reference
	{
		static constexpr T& value = *reinterpret_cast<T*>(address);
	};



	//template< typename T, unsigned int address >
	//T& reference<T, address>::value = *reinterpret_cast<T*>(address);


	// All addresses are 32bit wide on stm32f4
	template< unsigned int address >
	using reg_ptr = pointer< volatile uint32, address >;

	template< unsigned int address >
	using reg_ref = reference< volatile uint32, address >;

#endif
}

namespace bit {

	//
	// rmw_field
	// masked read-modify-write operation
	//
	template< typename DestType >
	void rmw_field( DestType& dest, unsigned int value, unsigned int mask, unsigned int offset )
	{
		dest = (dest & (~(mask << offset))) | ((value & mask) << offset);
	}

	// Todo: Maybe optimize for 16 and 8 bits through specializations
	// make_mask
	// ex.: make_mask<4>::value == 0xF
	//

	template< unsigned int Bits >
	struct make_mask {
		static const unsigned int value = (make_mask< Bits-1 >::value << 1) | 1;
	};

	template<>
	struct make_mask<0>
	{
		static const unsigned int value = 0;
	};


	//
	//	masked_value
	//

	template< typename T >
	struct masked_value {
		constexpr masked_value( T value, T mask)
			: value_(value), mask_(mask)
		{}

		masked_value& operator<<=(uint8 offset) {
			value_ <<= offset;
			mask_ <<= offset;
			return *this;
		}

		masked_value& operator>>=(uint8 offset) {
			value_ <<= offset;
			mask_ <<= offset;
			return *this;
		}

		const T value_;
		const T mask_;
	};

	template< typename Type >
	constexpr masked_value<Type> operator|( const masked_value<Type>& lhs, const masked_value<Type>& rhs )
	{
		return masked_value<Type>( lhs.value_ | rhs.value_, lhs.mask_|rhs.mask_ );
	}

	template< typename Type >
	constexpr masked_value<Type> operator<<( const masked_value<Type>& mv, uint8 off)
	{
		return masked_value<Type>(mv.value_ << off, mv.mask_ << off);
	}

	template< typename Type >
	constexpr masked_value<Type> operator>>( const masked_value<Type>& mv, uint8 off)
	{
		return masked_value<Type>(mv.value_ >> off, mv.mask_ >> off);
	}

	namespace detail {
		template< unsigned int Width >
		struct TypeFromWidth {
			using type = typename std::conditional < (Width <= 8), uint8,
			typename std::conditional< (Width <= 16), uint16,
			typename std::conditional< (Width <= 32), uint32,
			typename std::conditional< (Width <= 64), uint64, void >::type
			>::type
			>::type
			>::type;
		};

		template< unsigned int Width >
		using smallest_type_t = typename TypeFromWidth<Width>::type;

		template< typename Type, unsigned int Width, unsigned int Count, unsigned int Mask >
		struct replicate_bits {
			static constexpr Type exec(unsigned int value)
			{
				return (replicate_bits< Type, Width, Count - 1, (Mask >> 1) >::exec(value) << Width) | ((Mask & 1) ? value : 0);
			}
		};

		template< typename Type, unsigned int Width, unsigned int Mask >
		struct replicate_bits< Type, Width, 0, Mask >{
			static constexpr Type exec(unsigned int value)
			{
				return (void)value, 0;
			}
		};

		template< typename T, typename NewIntegral >
		struct change_integral_type {
			using base_type = typename std::remove_cv< T >::type;
			static const bool is_volatile = std::is_volatile< T >::value;
			using type = typename std::conditional<is_volatile, volatile NewIntegral, NewIntegral>::type;
		};


	}

	namespace optimized {

		template< uint8... bits >
		struct bit_list {};

		/* inspect mask
		 * count asserted bits in a mask and collect a list of asserted bits
		 *
		 */
		template<typename T, T mask, uint8 pos = sizeof(T)*8, uint8 numbits = 0, typename List = bit_list<> >
		struct inspect_mask;

		template<typename T, T mask, uint8 pos, uint8 numbits, uint8... bits >
		struct inspect_mask<T, mask, pos, numbits, bit_list<bits...> >
			: public std::conditional< (mask & 1) == 1, inspect_mask<T, (mask >> 1), pos-1, numbits+1, bit_list< bits..., sizeof(T)*8-pos >>, inspect_mask<T, (mask>>1), pos-1, numbits, bit_list<bits...> > >::type
		{};

		template<typename T, T mask, uint8 numbits, uint8... Bits >
		struct inspect_mask<T, mask, 0, numbits, bit_list<Bits...> >
		{
			static const unsigned int bitcount = numbits;
			using bitlist = bit_list<Bits...>;
		};

		/*
		 * calc_bitband_address
		 * Calculate the bitband alias region base address of a memory address inside the bitband region
		 */
		constexpr uint32 calc_bitband_address( uint32 Pos, uint32 Bit )
		{
			return 0x42000000 + (Pos-0x40000000) * 32 + Bit;
		}

		/*
		 * modify_bitbanded
		 * Read-modify write in a bitbanded memory address
		 */
		template< typename T, uint32 Address, T Mask, uint8 Bitcount = inspect_mask<T, Mask>::bitcount, typename BitList = typename inspect_mask<T, Mask>::bitlist >
		struct modify_bitbanded;

		template< typename T, uint32 Pos, T Mask, uint8 Bitcount, uint8 Bit0, uint8... Bits >
		struct modify_bitbanded< T, Pos, Mask, Bitcount, bit_list<Bit0, Bits...> >
		{
			static void func(typename std::remove_cv<T>::type value) {
				if(Bitcount == 1) {		// Test for best threshold
					*reinterpret_cast<volatile uint8*>(calc_bitband_address(Pos, Bit0)) = (value>>Bit0) & 1;
					modify_bitbanded< T, Pos, Mask, Bitcount, bit_list< Bits... > >::func(value);
				} else {
					auto& ref = *reinterpret_cast<T*>(Pos);
					ref = (ref & ~Mask) | value;
				}
			}
		};

		template< typename T, uint32 Pos, T Mask, uint8 Bitcount >
		struct modify_bitbanded< T, Pos, Mask, Bitcount, bit_list<> >
		{
			static void func(uint32 value) {
			}
		};

		/*
		 * read bitbanded
		 *
		 */

		// Multiple Bits always read from bitband region
		template< typename T, uint32 Address, T Mask, uint8 MaskOffset, typename BitList = typename inspect_mask<T, Mask>::bitlist >
		struct read_bitbanded {
			static typename std::remove_cv<T>::type func() {
				return ((*reinterpret_cast<T*>(Address))&Mask)>>MaskOffset;
			}
		};

		// Single Bit read from bitband alias region
		template< typename T, uint32 Address, T Mask, uint8 MaskOffset >
		struct read_bitbanded< T, Address, Mask, MaskOffset, bit_list<MaskOffset> > {
			static typename std::remove_cv<T>::type func() {
				return *reinterpret_cast<volatile uint8*>(calc_bitband_address(Address, MaskOffset));
			}
		};

		/*
		 * Masked value for a combination of static masks
		 * (Run-/Compiletime type for register field values)
		 */

		template< typename T, T mask >
		struct static_masked_value {
			constexpr static_masked_value( T value )
				: value_(value)
			{}

			constexpr operator masked_value<T>() const {
				return masked_value<T>(value_, mask);
			}

			T value_;
		};

		template< typename T, T lhs_mask, T rhs_mask >
		constexpr static_masked_value< T, lhs_mask | rhs_mask > operator|( static_masked_value<T, lhs_mask> lhs, static_masked_value<T, rhs_mask> rhs )
		{
			return static_masked_value< T, lhs_mask | rhs_mask >(lhs.value_|rhs.value_);
		}

		template< typename MVType, MVType Mask >
		void operator<<=( volatile uint32& reg, optimized::static_masked_value<MVType, Mask> fvm )
		{
			reg = (reg & ~Mask) | fvm.value_;
		}

	}

// Only uncomment in optimized builds... type bloat.
//#define USE_OPTIMIZED_BITS

	//
	//	field
	//	represents a field in a register, thus implies 32 bit value&mask.
	//

	template< unsigned int Top, unsigned int Bottom = Top, typename Type = uint32 >
	struct field :
#ifdef USE_OPTIMIZED_BITS
			public optimized::static_masked_value< Type, (make_mask< Top-Bottom+1 >::value << Bottom) >
#else
			public optimized::static_masked_value< Type, (make_mask< Top-Bottom+1 >::value << Bottom) >
			//public masked_value<Type>
#endif
	{
		static const unsigned int width = Top-Bottom+1;
		static const unsigned int offset = Bottom;
		static const unsigned int mask  = make_mask< width >::value << offset;

		constexpr field(Type value)
			:
#ifdef USE_OPTIMIZED_BITS
			optimized::static_masked_value< Type, mask >( value << Bottom )
#else
			optimized::static_masked_value< Type, mask >( value << Bottom )
			//masked_value<Type>( value << Bottom, mask )
#endif
		{}

		template< typename T >
		static detail::smallest_type_t< Top-Bottom+1 > read( const T& source )
		{
			return static_cast< detail::smallest_type_t< width > >(( source >> Bottom ) & make_mask< width >::value);
		}
	};

	template< typename DestType, typename MVType >
	void rmw_field( DestType& dest, const masked_value<MVType>& fvm )
	{
		dest = (dest & (~fvm.mask_)) | fvm.value_;
	}

	template< typename MVType >
	void operator<<=( volatile unsigned int& reg, const masked_value<MVType>& fvm )
	{
		rmw_field(reg, fvm);
	}


	// Replicates few bits into many
	// ex.: replicate< 4 /* 4 bits source */, 4 /* at most 4 times */, 11 /* ="1011" */ >(3)
	//      9      1     0   1    1
	// => value: "0011 0000 0011 0011"
	template< unsigned int Width, unsigned int Count, unsigned int Mask = -1 >
	constexpr typename detail::TypeFromWidth< Width * Count >::type replicate(unsigned int value)
	{
		return detail::replicate_bits< typename detail::TypeFromWidth<Width*Count>::type, Width, Count, Mask >::exec( make_mask<Width>::value & value );
	}

	// Replicates few bits into many and returns a masked value
	// ex.: replicate_masked< 4 /* 4 bits source */, 4 /* at most 4 times */, 11 /* =1011 */ >(3)
	//      9      1     0   1    1
	// => value: "0011 0000 0011 0011"
	// => mask:  "1111 0000 1111 1111"
	template< unsigned int Width, unsigned int Count, unsigned int Mask = -1 >
	constexpr masked_value< typename detail::TypeFromWidth< Width * Count >::type > replicate_masked(unsigned int value)
	{
		return masked_value< typename detail::TypeFromWidth< Width * Count >::type >(
				replicate< Width, Count, Mask >(value),
				replicate< Width, Count, Mask >(-1)
		);
	}

	template< typename T >
	struct register_snapshot
	{
		register_snapshot() = default;
		register_snapshot(const register_snapshot&) = default;

		register_snapshot( T value )
			: value_(value)
		{}

		template< typename FieldType >
		detail::smallest_type_t< FieldType::width > field()
		{
			return static_cast< detail::smallest_type_t<FieldType::width> >((value_&FieldType::mask)>>FieldType::offset);
		}

		T value_;
	};

	template< typename T >
	class basic_register {
	public:
		basic_register() {
			static_assert( sizeof(basic_register<T>) == 4, "Need to be of size 4.");
		}

		using argument_type = typename std::remove_cv<T>::type;

		basic_register& operator=( const argument_type val ) {
			value_ = val;
			return *this;
		}

		basic_register& operator|=( const argument_type val ) {
			value_ |= val;
			return *this;
		}

		basic_register& operator&=( const argument_type val ) {
			value_ &= val;
			return *this;
		}

		basic_register& operator^=( const argument_type val ) {
			value_ ^= val;
			return *this;
		}

		argument_type get() const {
			return value_;
		}

		basic_register& operator<<=( const bit::masked_value<argument_type>& val ) {
			value_ = (value_ & ~val.mask_) | val.value_;
			return *this;
		}

		basic_register& operator=( const bit::masked_value<argument_type>& val ) {
			value_ = val.value_;
			return *this;
		}

		template< typename Field >
		argument_type field()
		{
			return ((get()&Field::mask)>>Field::offset);
		}

		register_snapshot<argument_type> snapshot()
		{
			return register_snapshot<argument_type>(get());
		}

		/*
		 * Byte & Halfword access
		 *
		 */

		using byte_type = typename detail::change_integral_type<T, uint8>::type;
		using halfword_type = typename detail::change_integral_type<T, uint16>::type;

		template<uint8 num>
		byte_type& byte() {
			static_assert(num < sizeof(T), "Outside boundaries.");
			return *(reinterpret_cast<byte_type*>(&value_)+num);
		}

		template<uint8 num>
		halfword_type& halfword() {
			static_assert(num*2 < sizeof(T), "Outside boundaries.");
			return *(reinterpret_cast<halfword_type*>(&value_)+num);
		}

	private:
		T value_;
	};

	using register_base = basic_register< volatile uint32 >;	// rename volatile_register or similar...


	/*
	 * For registers in the bit banded region 0x40000000 (all STM32 peripherals)
	 *
	 */

#ifdef USE_OPTIMIZED_BITS
	template< typename T, uint32 Address >
	class basic_bitband_register : public basic_register< T > {
	public:
		static_assert( sizeof(basic_bitband_register<T>) == 4, "Need to be of size 4.");

		template< argument_type BitMask >
		basic_bitband_register& operator<<=(optimized::static_masked_value<argument_type, BitMask> val)
		{
			optimized::modify_bitbanded< T, Address, BitMask >::func( val.value_ );
		}

		template< typename Field >
		argument_type field()
		{
			return optimized::read_bitbanded< T, Address, Field::mask, Field::offset >::func();
		}
	};

	template< uint32 Address >
	using bitband_register = basic_bitband_register< volatile uint32, Address >;

#else
	template< uint32 Address >
	using bitband_register = register_base;
#endif

}

#endif /* UTIL_HPP_ */
