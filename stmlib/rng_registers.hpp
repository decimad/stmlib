// !Generated by regml2cpp!
#ifndef __RNG_INCLUDED
#define __RNG_INCLUDED

#include <stmlib/bits.hpp>

namespace rng
{
    namespace fields
    {
        // RNG control register [offset: 0x00, reset: 0x0000 0000]
        namespace cr
        {
            // Interrupt enable (rm page 752)
            using ie = bit::field<3>;
            // Random number generator enable (rm page 752)
            using rngen = bit::field<2>;
        } // namespace cr
        // RNG status register [offset: 0x04, reset: 0x0000 0000]
        namespace sr
        {
            // Seed error interrupt status (rm page 752)
            using seis = bit::field<6>;
            // Clock error interrupt status (rm page 752)
            using ceis = bit::field<5>;
            // Seed error current status (rm page 753)
            using secs = bit::field<2>;
            // Clock error current status (rm page 753)
            using cecs = bit::field<1>;
            // Data ready (rm page 753)
            using drdy = bit::field<0>;
        } // namespace sr
        // RNG data register [offset: 0x08, reset: 0x0000 0000]
        namespace dr
        {
            // Random data (rm page 753)
            using rndata = bit::field<31, 0>;
        } // namespace dr
    }     // namespace fields

    struct register_map
    {
        // CR: RNG control register (rm page 752)
        bit::bitband_register<0x50060800> cr;
        // SR: RNG status register (rm page 752)
        bit::bitband_register<0x50060804> sr;
        // DR: RNG data register (rm page 753)
        bit::bitband_register<0x50060808> dr;
    };

    extern "C" register_map __rng__device_0x50060800;
    static auto &device = __rng__device_0x50060800;

} // namespace rng

#endif
