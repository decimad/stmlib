#ifndef RNG_HPP__
#define RNG_HPP__

#ifdef __cplusplus
#include <stmlib/stmtypes.hpp>

namespace rng
{

    void start();
    uint32 rand();
    void stop();

} // namespace rng
#else

void stm_rand_start(void);
unsigned int stm_rand(void);
void stm_rand_stop(void);

#endif

#endif
