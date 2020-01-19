/*
 * interrupts.hpp
 *
 *  Created on: 17.12.2014
 *      Author: Michael
 */

#ifndef INTERRUPTS_HPP_
#define INTERRUPTS_HPP_

#include <stmlib/bits.hpp>
#include <stmlib/stmtypes.hpp>

#undef WWDG
#undef FLASH
#undef RCC
#undef DMA1_Stream0
#undef DMA1_Stream1
#undef DMA1_Stream2
#undef DMA1_Stream3
#undef DMA1_Stream4
#undef DMA1_Stream5
#undef DMA1_Stream6
#undef DMA1_Stream7
#undef ADC
#undef CAN1_TX
#undef CAN1_RX
#undef TIM2
#undef TIM3
#undef TIM4
#undef TIM5
#undef SPI1
#undef SPI2
#undef SPI3
#undef USART1
#undef USART2
#undef USART3
#undef UART4
#undef UART5
#undef TIM6
#undef TIM7
#undef DMA2_Stream0
#undef DMA2_Stream1
#undef DMA2_Stream2
#undef DMA2_Stream3
#undef DMA2_Stream4
#undef DMA2_Stream5
#undef DMA2_Stream6
#undef DMA2_Stream7
#undef ETH
#undef ETH_WKUP
#undef USART6
#undef I2C3_EV
#undef I2C3_ER
#undef OTG_HS_EP1_OUT
#undef OTF_HS_EP1_IN
#undef OTG_HS_WKUP
#undef OTG_HS
#undef DCMI
#undef CRYP
#undef HASH_RNG
#undef FPU
#undef SysTick
#undef SDIO

namespace nvic
{

    enum class Exception
    {
        NMI = 2,
        HardFault = 3,
        MemManage = 4,
        BusFault = 5,
        UsageFault = 6,
        SVCall = 11,
        PendSV = 14,
        SysTick = 15
    };

    enum class IRQ
    {
        /*0*/ WWDG = 0,
        PVD,
        TAMPER,
        RTC_WKUP,
        FLASH,
        RCC,
        EXTI0,
        EXTI1,
        EXTI2,
        EXTI3,
        /*10*/ EXTI4,
        DMA1_Stream0,
        DMA1_Stream1,
        DMA1_Stream2,
        DMA1_Stream3,
        DMA1_Stream4,
        DMA1_Stream5,
        DMA1_Stream6,
        ADC,
        CAN1_TX,
        /*20*/ CAN1_RX0,
        CAN1_RX1,
        CAN1_SCE,
        EXTI9_5,
        TIM1_BRK,
        TIM1_UP,
        TIM1_TRG_COM,
        TIM1_CC,
        TIM2,
        TIM3,
        /*30*/ TIM4,
        I2C1_EV,
        I2C1_ER,
        I2C2_EV,
        I2C2_ER,
        SPI1,
        SPI2,
        USART1,
        USART2,
        USART3,
        /*40*/ EXTI15_10,
        RTC_ALARM,
        OTG_FS_WKUP,
        TIM8_BRK,
        TIM8_UP,
        TIM8_TRG_COM,
        TIM8_CC,
        DMA1_Stream7,
        FSMC,
        SDIO,
        /*50*/ TIM5,
        SPI3,
        UART4,
        UART5,
        TIM6,
        TIM7,
        DMA2_Stream0,
        DMA2_Stream1,
        DMA2_Stream2,
        DMA2_Stream3,
        /*60*/ DMA2_Stream4,
        ETH,
        ETH_WKUP,
        CAN2_TX,
        CAN2_RX0,
        CAN2_RX1,
        CAN2_SCE,
        OTG_FS,
        DMA2_Stream5,
        DMA2_Stream6,
        /*70*/ DMA2_Stream7,
        USART6,
        I2C3_EV,
        I2C3_ER,
        OTG_HS_EP1_OUT,
        OTF_HS_EP1_IN,
        OTG_HS_WKUP,
        OTG_HS,
        DCMI,
        CRYP,
        HASH_RNG,
        FPU,

        NUM_IRQs,
        Undefined
    };

    struct register_map
    {
        volatile uint32 iser[3]; // 0x000 + 3*4
        uint8 zzreserved1[116];  // 0x00C
        volatile uint32 icer[3]; // 0x080 + 3*4
        uint8 zzreserved2[116];  // 0x08C
        volatile uint32 ispr[3]; // 0x100 + 3*4
        uint8 zzreserved3[116];  // 0x10C
        volatile uint32 icpr[3]; // 0x180
        uint8 zzreserved4[116];  // 0x18C
        volatile uint32 iabr[3]; // 0x200
        uint8 zzreserved5[244];  // 0x20C
        volatile uint32 ipr[21]; // 0x300
        uint8 zzreserved6[2732]; // 0x354
        volatile uint32 stir;    // 0xE00
    };

#if (__GNUC__ == 4 && __GNUC_MINOR__ == 9) || (__GNUC__ > 4)

    extern "C" register_map __nvic__device_0xE000E100;
    static auto &device = __nvic__device_0xE000E100;

#else

    static auto &device = util::reference<register_map, 0xE000E100>::value;

#endif

    inline void enable(IRQ irq)
    {
        const uint8 irq_ = static_cast<uint8>(irq);
        device.iser[irq_ / 32] |= 1 << (irq_ % 32);
    }

    inline void disable(IRQ irq)
    {
        const uint8 irq_ = static_cast<uint8>(irq);
        device.icer[irq_ / 32] |= 1 << (irq_ % 32);
    }

    inline void set_pending(IRQ irq)
    {
        const uint8 irq_ = static_cast<uint8>(irq);
        device.ispr[irq_ / 32] |= 1 << (irq_ % 32);
    }

    inline void clear_pending(IRQ irq)
    {
        const uint8 irq_ = static_cast<uint8>(irq);
        device.icpr[irq_ / 32] |= 1 << (irq_ % 32);
    }

    inline bool is_active(IRQ irq)
    {
        const uint8 irq_ = static_cast<uint8>(irq);
        return device.iabr[irq_ / 32] & (1 << (irq_ % 32));
    }

    inline void set_priority(IRQ irq, uint8 prio)
    {
        const uint8 irq_ = static_cast<uint8>(irq);
        device.ipr[irq_ / 4] = (device.ipr[irq_ / 4] & ~(0xFF << ((irq_ % 4) << 3))) | (prio << ((irq_ % 4) << 3));
        // device.ipr[irq_/4] = -1;
    }

    inline uint8 get_priority(IRQ irq)
    {
        const uint8 irq_ = static_cast<uint8>(irq);
        return (device.ipr[irq_ / 4] >> (irq_ % 4)) & 0xFF;
    }

    inline void trigger(IRQ irq)
    {
        device.stir = static_cast<int8>(irq);
    }

    // Default interrupt handler (global namespace for convenience)
    template <IRQ id>
    void isr();

} // namespace nvic

extern "C" void unimplemented_handler();

#endif /* INTERRUPTS_HPP_ */
