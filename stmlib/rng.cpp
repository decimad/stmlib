#include <stmlib/rcc_registers.hpp>
#include <stmlib/rng.hpp>
#include <stmlib/rng_registers.hpp>


namespace rng
{
    void start()
    {
        rcc::device.ahb2enr <<= rcc::fields::ahb2enr::rngen(1);
        device.cr <<= fields::cr::rngen(true);
    }

    uint32 rand()
    {
        while (!device.sr.field<fields::sr::drdy>())
            ;
        return device.dr.get();
    }

    void stop()
    {
        device.cr <<= fields::cr::rngen(false);
    }
} // namespace rng

extern "C" unsigned int stm_rand()
{
    return rng::rand();
}
