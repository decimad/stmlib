/*
 * dp83640.hpp
 *
 *  Created on: 04.11.2016
 *      Author: Michael
 */

#ifndef SYSTEM_STMLIB_STMLIB_ETH_LWIP_ONETHREAD_DP83640_HPP_
#define SYSTEM_STMLIB_STMLIB_ETH_LWIP_ONETHREAD_DP83640_HPP_

#include <stmlib/eth.hpp>

namespace drivers::dp83640
{

    using eth::phy::phy_field;
    using eth::phy::phy_register;

    namespace registers
    {

        // ptp base registers - page 4
        using ptp_ctl = phy_register<0x14, 4>;
        using ptp_tdr = phy_register<0x15, 4>;
        using ptp_sts = phy_register<0x16, 4>;
        using ptp_tsts = phy_register<0x17, 4>;
        using ptp_ratel = phy_register<0x18, 4>;
        using ptp_rateh = phy_register<0x19, 4>;
        using ptp_rdcksum = phy_register<0x1a, 4>;
        using ptp_wrcksum = phy_register<0x1b, 4>;
        using ptp_txts = phy_register<0x1c, 4>;
        using ptp_rxts = phy_register<0x1d, 4>;
        using ptp_ests = phy_register<0x1e, 4>;
        using ptp_edata = phy_register<0x1f, 4>;

        // ptp configuration registers - page 5
        using ptp_trig = phy_register<0x14, 5>;
        using ptp_evnt = phy_register<0x15, 5>;
        using ptp_txcfg0 = phy_register<0x16, 5>;
        using ptp_txcfg1 = phy_register<0x17, 5>;
        using ptp_psf_cfg0 = phy_register<0x18, 5>;
        using ptp_rxcfg0 = phy_register<0x19, 5>;
        using ptp_rxcfg1 = phy_register<0x1a, 5>;
        using ptp_rxcfg2 = phy_register<0x1b, 5>;
        using ptp_rxcfg3 = phy_register<0x1c, 5>;
        using ptp_rxcfg4 = phy_register<0x1d, 5>;
        using ptp_trdl = phy_register<0x1e, 5>;
        using ptp_trdh = phy_register<0x1f, 5>;

        // ptp configuration registers - page 6
        using ptp_coc = phy_register<0x14, 6>;
        using psf_cfg1 = phy_register<0x15, 6>;
        using psf_cfg2 = phy_register<0x16, 6>;
        using psf_cfg3 = phy_register<0x17, 6>;
        using psf_cfg4 = phy_register<0x18, 6>;
        using ptp_sfdcfg = phy_register<0x19, 6>;
        using ptp_intctl = phy_register<0x1a, 6>;
        using ptp_clksrc = phy_register<0x1b, 6>;
        using ptp_etr = phy_register<0x1c, 6>;
        using ptp_off = phy_register<0x1d, 6>;
        using ptp_gpiomon = phy_register<0x1e, 6>;
        using ptp_rxhash = phy_register<0x1f, 6>;

    } // namespace registers

    namespace fields
    {

        namespace ptp_ctl
        {
            using trig_sel = phy_field<12, 10>;
            using trig_dis = phy_field<9>;
            using trig_en = phy_field<8>;
            using trig_read = phy_field<7>;
            using trig_load = phy_field<6>;
            using ptp_rd_clk = phy_field<5>;
            using ptp_load_clk = phy_field<4>;
            using ptp_step_clk = phy_field<3>;
            using ptp_enable = phy_field<2>;
            using ptp_disable = phy_field<1>;
            using ptp_reset = phy_field<0>;
        } // namespace ptp_ctl

        namespace ptp_tdr
        {
            using time_data = phy_field<15, 0>;
        }

        namespace ptp_sts
        {
            using txts_rdy = phy_field<11>;
            using rxts_rdy = phy_field<10>;
            using trid_done = phy_field<9>;
            using event_rdy = phy_field<8>;
            using txts_ie = phy_field<3>;
            using rxts_ie = phy_field<2>;
            using trig_ie = phy_field<1>;
            using event_ie = phy_field<0>;
        } // namespace ptp_sts

        namespace ptp_tsts
        {
            using trig7_error = phy_field<15>;
            using trig7_active = phy_field<14>;
            using trig6_error = phy_field<13>;
            using trig6_active = phy_field<12>;
            using trig5_error = phy_field<11>;
            using trig5_active = phy_field<10>;
            using trig4_error = phy_field<9>;
            using trig4_active = phy_field<8>;
            using trig3_error = phy_field<7>;
            using trig3_active = phy_field<6>;
            using trig2_error = phy_field<5>;
            using trig2_active = phy_field<4>;
            using trig1_error = phy_field<3>;
            using trig1_active = phy_field<2>;
            using trig0_error = phy_field<1>;
            using trig0_active = phy_field<0>;
        } // namespace ptp_tsts

        namespace ptp_ratel
        {
            using ptp_rate_lo = phy_field<15, 0>;
        }

        namespace ptp_rateh
        {
            using ptp_rage_dir = phy_field<15>;
            using ptp_tmp_rate = phy_field<14>;
            using ptp_rate_hi = phy_field<9, 0>;
        } // namespace ptp_rateh

        namespace ptp_rdcksum
        {
            using rd_cksum = phy_field<15, 0>;
        }

        namespace ptp_wrcksum
        {
            using wr_cksum = phy_field<15, 0>;
        }

        namespace ptp_txts
        {
            using ptp_tx_ts = phy_field<15, 0>;
        }

        namespace ptp_rxts
        {
            using ptp_rx_ts = phy_field<15, 0>;
        }

        namespace ptp_ests
        {
            using evnts_missed = phy_field<10, 8>;
            using evnt_ts_len = phy_field<7, 6>;
            using evnt_rf = phy_field<5>;
            using evnt_num = phy_field<4, 2>;
            using mult_evnt = phy_field<1>;
            using event_det = phy_field<0>;
        } // namespace ptp_ests

        namespace ptp_edata
        {
            using e7_rise = phy_field<15>;
            using e7_det = phy_field<14>;
            using e6_rise = phy_field<13>;
            using e6_det = phy_field<12>;
            using e5_rise = phy_field<11>;
            using e5_det = phy_field<10>;
            using e4_rise = phy_field<9>;
            using e4_det = phy_field<8>;
            using e3_rise = phy_field<7>;
            using e3_det = phy_field<6>;
            using e2_rise = phy_field<5>;
            using e2_det = phy_field<4>;
            using e1_rise = phy_field<3>;
            using e1_det = phy_field<2>;
            using e0_rise = phy_field<1>;
            using e0_det = phy_field<0>;
        } // namespace ptp_edata

        namespace ptp_trig
        {
            using trig_pulse = phy_field<15>;
            using trig_per = phy_field<14>;
            using trig_if_late = phy_field<13>;
            using trig_notify = phy_field<12>;
            using trig_gpip = phy_field<11, 8>;
            using trig_toggle = phy_field<7>;
            using trig_csel = phy_field<3, 1>;
            using trig_wr = phy_field<0>;
        } // namespace ptp_trig

        namespace ptp_evnt
        {
            using evnt_rise = phy_field<15>;
            using evnt_fall = phy_field<14>;
            using evnt_single = phy_field<13>;
            using evnt_gpio = phy_field<12>;
            using evnt_sel = phy_field<3, 1>;
            using evnt_wr = phy_field<0>;
        } // namespace ptp_evnt

        namespace ptp_coc
        {
            using ptp_clkout_en = phy_field<15>;
            using ptp_clkout_sel = phy_field<14>;
            using ptp_clkout_speedsel = phy_field<13>;
            using ptp_clkdiv = phy_field<7, 0>;
        } // namespace ptp_coc

        namespace ptp_rxcfg0
        {
            using domain_en = phy_field<15>;
            using alt_mast_dis = phy_field<14>;
            using user_ip_sel = phy_field<13>;
            using user_ip_en = phy_field<12>;
            using rx_slave = phy_field<11>;
            using ip1588_en = phy_field<10, 8>;
            using rx_l2_en = phy_field<7>;
            using rx_ipv6_en = phy_field<6>;
            using rx_ipv4_en = phy_field<5>;
            using rx_ptp_ver = phy_field<4, 1>;
            using rx_ts_en = phy_field<0>;
        } // namespace ptp_rxcfg0

        namespace ptp_rxcfg1
        {
            using byte0_mask = phy_field<15, 8>;
            using byte0_data = phy_field<7, 0>;
        } // namespace ptp_rxcfg1

        namespace ptp_rx_cfg2
        {
            using ip_addr_data = phy_field<15, 0>;
        }

        namespace ptp_rx_cfg3
        {
            using ts_min_ifg = phy_field<15, 12>;
            using acc_udp = phy_field<11>;
            using acc_crc = phy_field<10>;
            using ts_append = phy_field<9>;
            using ts_insert = phy_field<8>;
            using ptp_domain = phy_field<7, 0>;
        } // namespace ptp_rx_cfg3

        namespace ptp_rx_cfg4
        {
            using ipv4_udp_mod = phy_field<15>;
            using ts_sec_en = phy_field<14>;
            using ts_sec_len = phy_field<13, 12>;
            using rxts_ns_off = phy_field<11, 6>;
            using rxts_sec_off = phy_field<5, 0>;
        } // namespace ptp_rx_cfg4

        namespace ptp_tx_cfg0
        {
            using sync_1step = phy_field<15>;
            using dr_insert = phy_field<13>;
            using ntp_ts_en = phy_field<12>;
            using ignore_2step = phy_field<11>;
            using crc_1step = phy_field<10>;
            using chk_1step = phy_field<9>;
            using ip1588_en = phy_field<8>;
            using tx_l2_en = phy_field<7>;
            using tx_ipv6_en = phy_field<6>;
            using tx_ipv4_en = phy_field<5>;
            using tx_ptp_ver = phy_field<4, 1>;
            using tx_ts_en = phy_field<0>;
        } // namespace ptp_tx_cfg0

        namespace ptp_tx_cfg1
        {
            using byte0_mask = phy_field<15, 8>;
            using byte0_data = phy_field<7, 0>;
        } // namespace ptp_tx_cfg1
    }     // namespace fields

    void init();
    void shutdown();

    void enable_timestamping(bool enabled);

    void set_clock(uint64 time);
    void offset_clock();

    void set_output_clock(uint32 clock, eth::phy::callback_type completion_callback = {});

} // namespace drivers::dp83640

#endif /* SYSTEM_STMLIB_STMLIB_ETH_LWIP_ONETHREAD_DP83640_HPP_ */
