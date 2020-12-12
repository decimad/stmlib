/*
 * gpio.hpp
 *
 *  Created on: 24.11.2014
 *      Author: Michael
 */

#ifndef GPIO_HPP_
#define GPIO_HPP_

#include <microlib/meta_vlist.hpp>
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
            reg optcr1;  // only F42X andF43X
        };

        constexpr uint32 port_base_address(uint8 port)
        {
            return 0x40020000 + port * 0x400;
        }

    } // namespace reg_detail


    inline reg_detail::gpio_regs &port_ref(uint8 port)
    {
        return *reinterpret_cast<reg_detail::gpio_regs *>(reg_detail::port_base_address(port));
    }

    static auto& porta [[maybe_unused]] = *(reg_detail::gpio_regs*)(reg_detail::port_base_address(0));
    static auto& portb [[maybe_unused]] = *(reg_detail::gpio_regs*)(reg_detail::port_base_address(1));
    static auto& portc [[maybe_unused]] = *(reg_detail::gpio_regs*)(reg_detail::port_base_address(2));
    static auto& portd [[maybe_unused]] = *(reg_detail::gpio_regs*)(reg_detail::port_base_address(3));
    static auto& porte [[maybe_unused]] = *(reg_detail::gpio_regs*)(reg_detail::port_base_address(4));
    static auto& portf [[maybe_unused]] = *(reg_detail::gpio_regs*)(reg_detail::port_base_address(5));
    static auto& portg [[maybe_unused]] = *(reg_detail::gpio_regs*)(reg_detail::port_base_address(6));
    static auto& porth [[maybe_unused]] = *(reg_detail::gpio_regs*)(reg_detail::port_base_address(7));
    static auto& porti [[maybe_unused]] = *(reg_detail::gpio_regs*)(reg_detail::port_base_address(8));

    template <uint8 port, uint8... pins>
    struct port_pack
    {
    };

    //! \brief Compile time fixed GPIO pin object
    //! \param port   Port index [0 .. 8] for ports [a .. i]
    //! \param pin    Pin index [0 .. 15]
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

    //! \brief Runtime dynamic GPIO pin object
    class dynamic_pin
    {
      public:
        //! Assign GPIO pin reference from compile time static pin
        template <uint8 Port, uint8 Pin>
        constexpr dynamic_pin(const static_pin<Port, Pin> &) : port_(Port), pin_(Pin)
        {
        }

        constexpr dynamic_pin(uint8 port__, uint8 pin__) : port_(port__), pin_(pin__)
        {
        }

        // Fixme: compare assembly with plain 'assign' and see if we need two functions for this
        void atomic_assign(bool value)
        {
            port_ref(port_).bsrr = ((value ^ 1) << pin_) | (value << (pin_ + 16));
        }

        // Fixme, see 'atomic_assign'
        void assign(bool value)
        {
            auto &ref = port_ref(port_).odr;
            ref = (ref & (~static_cast<uint32>(1 << pin_))) | ((value & 1) << pin_);
        }

        //! Set GPO pin
        void set()
        {
            port_ref(port_).bsrr = 1 << (pin_ + 16);
        }

        //! Clear GPO pin
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
        using namespace ulib::meta;
        using concepts::ValueList;

        template <ValueList Array, typename... pins>
        struct port_array_from_pins;

        template <ValueList Array, uint8 port, uint8 pin, typename... tail>
        struct port_array_from_pins<Array, static_pin<port, pin>, tail...>
        {
            using type = typename port_array_from_pins<
                value_list::set<Array, port, value_list::get<Array, port> | (static_cast<uint16>(1) << pin)>,
                tail...
            >::type;
        };

        // translate port_pack<> and pin_list<> into a plain list of pins
        template <ValueList Array, uint8 port, uint8... pins, typename... tail>
        struct port_array_from_pins<Array, port_pack<port, pins...>, tail...>
        {
            using type = typename port_array_from_pins<Array, static_pin<port, pins>..., tail...>::type;
        };

        template <ValueList Array, typename... pins, typename... tail>
        struct port_array_from_pins<Array, pin_list<pins...>, tail...>
        {
            using type = typename port_array_from_pins<Array, pins..., tail...>::type;
        };

        template <ValueList Array>
        struct port_array_from_pins<Array>
        {
            using type = Array;
        };

        using empty_port_array = ulib::meta::vlist<uint16, 0, 0, 0, 0, 0, 0, 0, 0>;
    } // namespace detail

    template <typename... PinsListPack, typename... Args>
    void configure(const Args &... args)
    {
        using namespace ulib::meta;
        using port_array = typename detail::port_array_from_pins<detail::empty_port_array, PinsListPack...>::type;

        if constexpr (value_list::get<port_array, 0> != 0)
        {
            (Args::template run_static<0, value_list::get<port_array, 0>>(args), ...);
        }

        if constexpr (value_list::get<port_array, 1> != 0)
        {
            (Args::template run_static<1, value_list::get<port_array, 1>>(args), ...);
        }

        if constexpr (value_list::get<port_array, 2> != 0)
        {
            (Args::template run_static<2, value_list::get<port_array, 2>>(args), ...);
        }

        if constexpr (value_list::get<port_array, 3> != 0)
        {
            (Args::template run_static<3, value_list::get<port_array, 3>>(args), ...);
        }

        if constexpr (value_list::get<port_array, 4> != 0)
        {
            (Args::template run_static<4, value_list::get<port_array, 4>>(args), ...);
        }

        if constexpr (value_list::get<port_array, 5> != 0)
        {
            (Args::template run_static<5, value_list::get<port_array, 5>>(args), ...);
        }

        if constexpr (value_list::get<port_array, 6> != 0)
        {
            (Args::template run_static<6, value_list::get<port_array, 6>>(args), ...);
        }

        if constexpr (value_list::get<port_array, 7> != 0)
        {
            (Args::template run_static<7, value_list::get<port_array, 7>>(args), ...);
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

        static void run_dynamic(const dynamic_pin &p, const alternate_output &config)
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

        static void run_dynamic(const dynamic_pin &p, const alternate_input &config)
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

        static void run_dynamic(const dynamic_pin &p, const input_struct &mode)
        {
            (void)mode;

            using namespace reg_detail;
            auto &port = port_ref(p.port());

            port.moder <<= bit::masked_value<uint32>(0, bit::make_mask<2>::value) << (p.index() * 2);
        }
    };

    [[maybe_unused]] const input_struct  input;

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

        static void run_dynamic(const dynamic_pin &p, const output &config)
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

        static void run_dynamic(const dynamic_pin &p, const pull_up &config)
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

        static void run_dynamic(const dynamic_pin &p, const inout &config)
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

    template< typename Pad, AlternateMode mode, OutputType type = OutputType::PushPull, Speed speed = Speed::Slow, Resistor resistor = Resistor::None >
    struct static_altout_config {
        void init()
        {
            configure<Pad>(alternate_output(mode, type, speed), pull_up(resistor));
        }
    };

    template< typename Pad, AlternateMode mode, Resistor resistor = Resistor::None >
    struct static_altin_config {
        void init()
        {
            configure<Pad>(alternate_input(mode), pull_up(resistor));
        }
    };

    template< typename Pad, Speed speed = Speed::Slow, Resistor resistor = Resistor::None >
    struct static_gpo_config {
        void init()
        {
            configure<Pad>(output(speed), pull_up(resistor));
        }

        static void set(bool value)
        {
            Pad::set(value);
        }

        static bool get()
        {
            return Pad::get();
        }
    };

    template< typename Pad, Resistor resistor = Resistor::None >
    struct static_gpi_config {
        static void init()
        {
            configure<Pad>(input, pull_up(resistor));
        }

        static void get()
        {
            Pad::get();
        }
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
