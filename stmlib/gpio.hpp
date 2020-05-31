/*
 * gpio.hpp
 *
 *  Created on: 24.11.2014
 *      Author: Michael
 */

#ifndef GPIO_HPP_
#define GPIO_HPP_

#include <microlib/meta_array.hpp>
#include <stmlib/bits.hpp>

namespace gpio
{
    namespace reg_detail
    {

        using reg = volatile uint32;

        struct gpio_regs
        {
            reg moder;
            reg otyper;
            reg ospeedr;
            reg pupdr;
            reg idr;
            reg odr;
            reg bsrr;
            reg lckr;
            reg afrl;
            reg afrh;
            // reg optcr1; only F42X andF43X
        };

    } // namespace reg_detail

    inline reg_detail::gpio_regs &port_ref(uint8 port)
    {
        return *reinterpret_cast<reg_detail::gpio_regs *>(0x40020000 + port * 0x400);
    }

    extern "C" reg_detail::gpio_regs __gpio__porta_0x40020000;
    extern "C" reg_detail::gpio_regs __gpio__portb_0x40020400;
    extern "C" reg_detail::gpio_regs __gpio__portc_0x40020800;
    extern "C" reg_detail::gpio_regs __gpio__portd_0x40020C00;
    extern "C" reg_detail::gpio_regs __gpio__porte_0x40021000;
    extern "C" reg_detail::gpio_regs __gpio__portf_0x40021400;
    extern "C" reg_detail::gpio_regs __gpio__portg_0x40021800;
    extern "C" reg_detail::gpio_regs __gpio__porth_0x40021C00;
    extern "C" reg_detail::gpio_regs __gpio__porti_0x40022000;

    static auto &porta = __gpio__porta_0x40020000;
    static auto &portb = __gpio__portb_0x40020400;
    static auto &portc = __gpio__portc_0x40020800;
    static auto &portd = __gpio__portd_0x40020C00;
    static auto &porte = __gpio__porte_0x40021000;
    static auto &portf = __gpio__portf_0x40021400;
    static auto &portg = __gpio__portg_0x40021800;
    static auto &porth = __gpio__porth_0x40021C00;
    static auto &porti = __gpio__porti_0x40022000;

    template <uint8 port, uint8... pins>
    struct port_pack
    {
    };

    template <uint8 port, uint8 index>
    struct static_pin
    {
        constexpr static_pin()
        {
        }

        static void atomic_assign(bool value)
        {
            port_ref(port).bsrr = (((!value) & 1) << index) | ((value & 1) << (index + 16));
        }

        static void assign(bool value)
        {
            auto &ref = port_ref(port).odr;
            ref = (ref & (~static_cast<uint32>(1 << index))) | ((value & 1) << index);
        }

        static void set()
        {
            port_ref(port).bsrr = 1 << (index);
        }

        static void reset()
        {
            port_ref(port).bsrr = 1 << (index + 16);
        }

        static bool get()
        {
            return (port_ref(port).idr & (1 << index)) != 0;
        }
    };

    class pin_ref
    {
      public:
        template <uint8 Port, uint8 Pin>
        constexpr pin_ref(const static_pin<Port, Pin> &) : port_(Port), pin_(Pin)
        {
        }

        constexpr pin_ref(uint8 port__, uint8 pin__) : port_(port__), pin_(pin__)
        {
        }

        void atomic_assign(bool value)
        {
            port_ref(port_).bsrr = (((!value) & 1) << pin_) | ((value & 1) << (pin_ + 16));
        }

        void assign(bool value)
        {
            auto &ref = port_ref(port_).odr;
            ref = (ref & (~static_cast<uint32>(1 << pin_))) | ((value & 1) << pin_);
        }

        void set()
        {
            port_ref(port_).bsrr = 1 << (pin_ + 16);
        }

        void reset()
        {
            port_ref(port_).bsrr = 1 << pin_;
        }

        bool get() const
        {
            return (port_ref(port_).bsrr & (1 << pin_)) != 0;
        }

        uint8 port() const
        {
            return port_;
        }

        uint8 index() const
        {
            return pin_;
        }

        template <typename... Args>
        void configure(const Args &... args)
        {
            (Args::run_dynamic(*this, args), ...);
        }

      private:
        uint8 port_;
        uint8 pin_;
    };

    template <typename... Pin>
    struct pin_list
    {
    };

    namespace detail
    {
        template <typename Array, typename... pins>
        struct port_array_from_pins;

        template <typename Array, uint8 port, uint8 pin, typename... tail>
        struct port_array_from_pins<Array, static_pin<port, pin>, tail...>
        {
            using type = typename port_array_from_pins<
                typename ulib::meta::deprecated::set<
                    Array, port, ulib::meta::deprecated::get<Array, port>::value | (static_cast<uint16>(1) << pin)>::type,
                tail...>::type;
        };

        // translate port_pack<> and pin_list<> into a plain list of pins
        template <typename Array, uint8 port, uint8... pins, typename... tail>
        struct port_array_from_pins<Array, port_pack<port, pins...>, tail...>
        {
            using type = typename port_array_from_pins<Array, static_pin<port, pins>..., tail...>::type;
        };

        template <typename Array, typename... pins, typename... tail>
        struct port_array_from_pins<Array, pin_list<pins...>, tail...>
        {
            using type = typename port_array_from_pins<Array, pins..., tail...>::type;
        };

        template <typename Array>
        struct port_array_from_pins<Array>
        {
            using type = Array;
        };

        using empty_port_array = ulib::meta::deprecated::array<uint16, 0, 0, 0, 0, 0, 0, 0, 0>;
    } // namespace detail

    template <typename... PinsListPack, typename... Args>
    void configure(const Args &... args)
    {
        using port_array = typename detail::port_array_from_pins<detail::empty_port_array, PinsListPack...>::type;
        using namespace ulib::meta;

        if (deprecated::get<port_array, 0>::value != 0)
        {
            (Args::template run_static<0, deprecated::get<port_array, 0>::value>(args), ...);
        }

        if (deprecated::get<port_array, 1>::value != 0)
        {
            (Args::template run_static<1, deprecated::get<port_array, 1>::value>(args), ...);
        }

        if (deprecated::get<port_array, 2>::value != 0)
        {
            (Args::template run_static<2, deprecated::get<port_array, 2>::value>(args), ...);
        }

        if (deprecated::get<port_array, 3>::value != 0)
        {
            (Args::template run_static<3, deprecated::get<port_array, 3>::value>(args), ...);
        }

        if (deprecated::get<port_array, 4>::value != 0)
        {
            (Args::template run_static<4, deprecated::get<port_array, 4>::value>(args), ...);
        }

        if (deprecated::get<port_array, 5>::value != 0)
        {
            (Args::template run_static<5, deprecated::get<port_array, 5>::value>(args), ...);
        }

        if (deprecated::get<port_array, 6>::value != 0)
        {
            (Args::template run_static<6, deprecated::get<port_array, 6>::value>(args), ...);
        }

        if (deprecated::get<port_array, 7>::value != 0)
        {
            (Args::template run_static<7, deprecated::get<port_array, 7>::value>(args), ...);
        }
    }

    // Das spart bei mir Compile-Zeit, man k�nnte es nat�rlich auch generisch aufziehen.

    enum class OutputType
    {
        PushPull = 0,
        OpenDrain = 1
    };

    enum class Speed
    {
        Slow = 0,
        Medium = 1,
        Fast = 2,
        High = 3
    };

    enum class Resistor
    {
        PullUp = 1,
        PullDown = 2,
        None = 0
    };

    enum class AlternateMode
    {
        Fsmc = 12,
        Eth = 11
    };

    struct alternate_output
    {
        constexpr alternate_output(AlternateMode alternate_mode_, OutputType output_type_ = OutputType::PushPull,
                                   Speed speed_ = Speed::High)
            : alternate_mode(alternate_mode_), output_type(output_type_), speed(speed_)
        {
        }

        template <uint8 port_num, uint16 pinflags>
        static void run_static(const alternate_output &config)
        {
            using namespace reg_detail;
            auto &port = port_ref(port_num);

            // const uint64 alternatevalue = util::replicate_dynamic64< pinflags >(static_cast<uint8>(config.alternate_mode));
            const auto alternate = bit::replicate_masked<4, 16, pinflags>(static_cast<uint8>(config.alternate_mode));
            port.afrl <<= alternate;
            port.afrh <<= (alternate >> 32);

            port.moder <<= bit::replicate_masked<2, 16, pinflags>(2); // alternate mode
            port.otyper <<= bit::replicate_masked<1, 16, pinflags>(static_cast<uint8>(config.output_type));
            port.ospeedr <<= bit::replicate_masked<2, 16, pinflags>(static_cast<uint8>(config.speed));
        }

        static void run_dynamic(const pin_ref &p, const alternate_output &config)
        {
            using namespace reg_detail;
            auto &port = port_ref(p.port());

            const auto alternate = bit::masked_value<uint64>(static_cast<uint8>(config.alternate_mode), bit::make_mask<4>::value)
                                   << (p.index() * 4);
            port.afrl <<= alternate;
            port.afrh <<= alternate >> 32;

            port.moder <<= bit::masked_value<uint32>(2, bit::make_mask<2>::value) << (p.index() * 2);
            port.otyper <<= bit::masked_value<uint32>(static_cast<uint8>(config.output_type), bit::make_mask<1>::value) << (p.index() * 1);
            port.ospeedr <<= bit::masked_value<uint32>(static_cast<uint8>(config.speed), bit::make_mask<2>::value) << (p.index() * 2);
        }

        AlternateMode alternate_mode;
        OutputType output_type;
        Speed speed;
    };

    struct alternate_input
    {
        constexpr alternate_input(AlternateMode alternate_mode_) : alternate_mode(alternate_mode_)
        {
        }

        template <uint8 port_num, uint16 pinflags>
        static void run_static(const alternate_input &config)
        {
            using namespace reg_detail;
            auto &port = port_ref(port_num);

            // const uint64 alternatevalue = util::replicate_dynamic64< pinflags >(static_cast<uint8>(config.alternate_mode));
            const auto alternate = bit::replicate_masked<4, 16, pinflags>(static_cast<uint8>(config.alternate_mode));
            port.afrl <<= alternate;
            port.afrh <<= (alternate >> 32);

            port.moder <<= bit::replicate_masked<2, 16, pinflags>(2); // alternate mode
        }

        static void run_dynamic(const pin_ref &p, const alternate_input &config)
        {
            using namespace reg_detail;
            auto &port = port_ref(p.port());

            const auto alternate = bit::masked_value<uint64>(static_cast<uint8>(config.alternate_mode), bit::make_mask<4>::value)
                                   << (p.index() * 4);
            port.afrl <<= alternate;
            port.afrh <<= alternate >> 32;

            port.moder <<= bit::masked_value<uint32>(0, bit::make_mask<0>::value) << (p.index() * 2);
        }

        AlternateMode alternate_mode;
    };

    // input maps to input
    struct input_struct
    {
        constexpr input_struct()
        {
        }

        template <uint8 port_num, uint16 pinflags>
        static void run_static(const input_struct &mode)
        {
            (void)mode;

            using namespace reg_detail;
            auto &port = port_ref(port_num);

            port.moder <<= bit::replicate_masked<2, 16, pinflags>(0);
        }

        static void run_dynamic(const pin_ref &p, const input_struct &mode)
        {
            (void)mode;

            using namespace reg_detail;
            auto &port = port_ref(p.port());

            port.moder <<= bit::masked_value<uint32>(0, bit::make_mask<2>::value) << (p.index() * 2);
        }
    };

    const input_struct input;

    // Output maps to output + push&pull
    struct output
    {
        constexpr output(Speed speed_ = Speed::High) : speed(speed_)
        {
        }

        template <uint8 port_num, uint16 pinflags>
        static void run_static(const output &config)
        {
            using namespace reg_detail;
            auto &port = port_ref(port_num);

            port.moder <<= bit::replicate_masked<2, 16, pinflags>(1);
            port.otyper <<= bit::replicate_masked<1, 16, pinflags>(static_cast<uint8>(OutputType::PushPull));
            port.ospeedr <<= bit::replicate_masked<2, 16, pinflags>(static_cast<uint8>(config.speed));
        }

        static void run_dynamic(const pin_ref &p, const output &config)
        {
            using namespace reg_detail;
            auto &port = port_ref(p.port());

            port.moder <<= bit::masked_value<uint32>(1, bit::make_mask<2>::value) << (p.index() * 2);
            port.otyper <<= bit::masked_value<uint32>(static_cast<uint8>(OutputType::PushPull), bit::make_mask<1>::value)
                            << (p.index() * 1);
            port.ospeedr <<= bit::masked_value<uint32>(static_cast<uint8>(config.speed), bit::make_mask<2>::value) << (p.index() * 2);
        }

        Speed speed;
    };

    struct pull_up
    {
        constexpr pull_up()
        {
        }

        template <uint8 port_num, uint16 pinflags>
        static void run_static(const pull_up &config)
        {
            (void)config;

            using namespace reg_detail;
            auto &port = port_ref(port_num);

            port.pupdr <<= bit::replicate_masked<2, 16, pinflags>(static_cast<uint8>(Resistor::PullUp));
        }

        static void run_dynamic(const pin_ref &p, const pull_up &config)
        {
            (void)config;

            using namespace reg_detail;
            auto &port = port_ref(p.port());

            port.pupdr <<= bit::masked_value<uint32>(static_cast<uint8>(Resistor::PullUp), bit::make_mask<2>::value) << (p.index() * 2);
        }
    };

    // inout maps to output + open-drain
    struct inout
    {
        constexpr inout(Speed speed_ = Speed::High) : speed(speed_)
        {
        }

        template <uint8 port_num, uint16 pinflags>
        static void run_static(const inout &config)
        {
            using namespace reg_detail;
            auto &port = port_ref(port_num);

            port.moder <<= bit::replicate_masked<2, 16, pinflags>(1);
            port.otyper <<= bit::replicate_masked<1, 16, pinflags>(static_cast<uint8>(OutputType::OpenDrain));
            port.ospeedr <<= bit::replicate_masked<2, 16, pinflags>(static_cast<uint8>(config.speed));
        }

        static void run_dynamic(const pin_ref &p, const inout &config)
        {
            using namespace reg_detail;
            auto &port = port_ref(p.port());

            port.moder <<= bit::masked_value<uint32>(1, bit::make_mask<2>::value) << (p.index() * 2);
            port.otyper <<= bit::masked_value<uint32>(static_cast<uint8>(OutputType::OpenDrain), bit::make_mask<1>::value)
                            << (p.index() * 1);
            port.ospeedr <<= bit::masked_value<uint32>(static_cast<uint8>(config.speed), bit::make_mask<2>::value) << (p.index() * 2);
        }

        Speed speed;
    };

    // analog maps to analog

    template <uint8 pin>
    using pa = static_pin<0, pin>;

    template <uint8 pin>
    using pb = static_pin<1, pin>;

    template <uint8 pin>
    using pc = static_pin<2, pin>;

    template <uint8 pin>
    using pd = static_pin<3, pin>;

    template <uint8 pin>
    using pe = static_pin<4, pin>;

    template <uint8 pin>
    using pf = static_pin<5, pin>;

    template <uint8 pin>
    using pg = static_pin<6, pin>;

    template <uint8 pin>
    using ph = static_pin<7, pin>;

    template <uint8 pin>
    using pi = static_pin<8, pin>;

    template <uint8... pins>
    using pack_a = port_pack<0, pins...>;

    template <uint8... pins>
    using pack_b = port_pack<1, pins...>;

    template <uint8... pins>
    using pack_c = port_pack<2, pins...>;

    template <uint8... pins>
    using pack_d = port_pack<3, pins...>;

    template <uint8... pins>
    using pack_e = port_pack<4, pins...>;

    template <uint8... pins>
    using pack_f = port_pack<5, pins...>;

    template <uint8... pins>
    using pack_g = port_pack<6, pins...>;

    template <uint8... pins>
    using pack_h = port_pack<7, pins...>;

    template <uint8... pins>
    using pack_i = port_pack<8, pins...>;

} // namespace gpio

#endif /* GPIO_HPP_ */
