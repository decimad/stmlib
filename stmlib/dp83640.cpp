/*
 * dp83640.cpp
 *
 *  Created on: 04.11.2016
 *      Author: Michael
 */

#include "dp83640.hpp"
#include <stmlib/eth.hpp>

namespace drivers::dp83640
{

    void init()
    {
        eth::phy::write<registers::ptp_ctl>(1, fields::ptp_ctl::ptp_enable(1)); // f�r die 4 MHz
        //		eth::phy::write<registers::ptp_rxcfg0>(1, fields::ptp_rxcfg0::rx_ts_en(0));
        //		eth::phy::write<registers::ptp_rxcfg3>(1, fields::ptp_rx_cfg3::ts_insert(0));
        //		eth::phy::write<registers::ptp_rxcfg4>(1, fields::ptp_rx_cfg4::ts_sec_en(0)| fields::ptp_rx_cfg4::ts_sec_len(0));
        //		eth::phy::write<registers::ptp_txcfg0>(1, fields::ptp_tx_cfg0::tx_ts_en(0));

        // eth::phy::write<registers::ptp_ctl>(1, fields::ptp_ctl::ptp_enable(1));
        /*
        eth::phy::write<registers::ptp_rxcfg0>(1,
                fields::ptp_rxcfg0::ip1588_en(1)
              | fields::ptp_rxcfg0::rx_ipv4_en(1)
              | fields::ptp_rxcfg0::rx_ptp_ver(2)
              | fields::ptp_rxcfg0::rx_ts_en(1)
        );
        eth::phy::write<registers::ptp_rxcfg3>(1,
                fields::ptp_rx_cfg3::ts_insert(1)
        );
        eth::phy::write<registers::ptp_rxcfg4>(1,
                fields::ptp_rx_cfg4::ts_sec_en(1)
              | fields::ptp_rx_cfg4::ts_sec_len(3)
        );
        eth::phy::write<registers::ptp_txcfg0>(1,
                fields::ptp_tx_cfg0::sync_1step(1)
              | fields::ptp_tx_cfg0::dr_insert(1)
              | fields::ptp_tx_cfg0::chk_1step(1)
              | fields::ptp_tx_cfg0::tx_ipv4_en(1)
              | fields::ptp_tx_cfg0::tx_ptp_ver(2)
              | fields::ptp_tx_cfg0::tx_ts_en(1)
        );
        */
    }

    void set_clock(uint64 val)
    {
        eth::phy::write<registers::ptp_tdr>(1, (val >> 48) & 0xFFFF);
        eth::phy::write<registers::ptp_tdr>(1, (val >> 32) & 0xFFFF);
        eth::phy::write<registers::ptp_tdr>(1, (val >> 16) & 0xFFFF);
        eth::phy::write<registers::ptp_tdr>(1, (val >> 0) & 0xFFFF);
        eth::phy::write<registers::ptp_ctl>(1, fields::ptp_ctl::ptp_load_clk(1));
    }

    void offset_clock(int64 offset)
    {
        uint64 val = static_cast<uint64>(offset);

        eth::phy::write<registers::ptp_tdr>(1, (val >> 48) & 0xFFFF);
        eth::phy::write<registers::ptp_tdr>(1, (val >> 32) & 0xFFFF);
        eth::phy::write<registers::ptp_tdr>(1, (val >> 16) & 0xFFFF);
        eth::phy::write<registers::ptp_tdr>(1, (val >> 0) & 0xFFFF);
        eth::phy::write<registers::ptp_ctl>(1, fields::ptp_ctl::ptp_step_clk(1));
    }

    void rate_adjust(int adjust)
    {
        unsigned char sign = (adjust < 0) ? 1 : 0;
        unsigned int val = (adjust < 0) ? -adjust : adjust;
        eth::phy::write<registers::ptp_rateh>(1, fields::ptp_rateh::ptp_rage_dir(sign) | fields::ptp_rateh::ptp_rate_hi(adjust >> 16));
        eth::phy::write<registers::ptp_ratel>(1, fields::ptp_rateh::ptp_rate_hi(adjust));
    }

    void set_output_clock(uint32 clock, eth::phy::callback_type completion_callback)
    {
        uint8 divider = 250000000ul / clock;
        eth::phy::write<registers::ptp_coc>(1, fields::ptp_coc::ptp_clkout_en(1) | fields::ptp_coc::ptp_clkdiv(divider),
                                            std::move(completion_callback));
    }

} // namespace drivers::dp83640
