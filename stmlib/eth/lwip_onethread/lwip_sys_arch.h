#ifndef STMLIB_ETH_LWIP_ONETHREAD_LWIP_SYS_ARCH_H__
#define STMLIB_ETH_LWIP_ONETHREAD_LWIP_SYS_ARCH_H__

#ifdef __cplusplus
extern "C"
{
#endif

    void lwip_thread_timeout(unsigned int, void (*)(void *), void *);
    void lwip_thread_untimeout(void (*)(void *), void *);

#ifdef __cplusplus
}
#endif

#endif
