// !Generated by regml2cpp!
#ifndef __ETH_INCLUDED
#define __ETH_INCLUDED

#include <stmlib/bits.hpp>

namespace eth
{
    namespace fields
    {
        // Ethernet MAC configuration register [offset: 0x0000, reset: 0x0000 8000]
        namespace maccr
        {
            // CRC stripping for Type frames (rm page 1153)
            using cstf = bit::field<25>;
            // Watchdog disable (rm page 1153)
            using wd = bit::field<23>;
            // Jabber disable (rm page 1153)
            using jd = bit::field<22>;
            // Interframe gap (rm page 1174)
            using ifg = bit::field<19, 17>;
            // Carrier sense disable (rm page 1174)
            using csd = bit::field<16>;
            // Fast Ethernet speed (rm page 1174)
            using fes = bit::field<14>;
            // Receive own disable (rm page 1174)
            using rod = bit::field<13>;
            // Loopback mode (rm page 1174)
            using lm = bit::field<12>;
            // Duplex mode (rm page 1174)
            using dm = bit::field<11>;
            // IPv4 checksum offload (rm page 1174)
            using ipco = bit::field<10>;
            // Retry disable (rm page 1174)
            using rd = bit::field<9>;
            // Automatic pad/CRC stripping (rm page 1176)
            using apcs = bit::field<7>;
            // Back-off limit (rm page 1176)
            using bl = bit::field<6, 5>;
            // Deferral check (rm page 1176)
            using dc = bit::field<4>;
            // Transmitter enable (rm page 1176)
            using te = bit::field<3>;
            // Receiver enable (rm page 1176)
            using re = bit::field<2>;
        } // namespace maccr

        // Ethernet MAC frame filter register [offset: 0x0004, reset: 0x0000 0000]
        namespace macffr
        {
            // Receive all (rm page 1177)
            using ra = bit::field<31>;
            // Hash or perfect filter (rm page 1177)
            using hpf = bit::field<10>;
            // Source address filter (rm page 1177)
            using saf = bit::field<9>;
            // Source address inverse filtering (rm page 1177)
            using saif = bit::field<8>;
            // Pass control frames (rm page 1178)
            using pcf = bit::field<7, 6>;
            // Broadcast frames disable (rm page 1178)
            using bfd = bit::field<5>;
            // Pass all multicast (rm page 1178)
            using pam = bit::field<4>;
            // Destination address inverse filtering (rm page 1178)
            using daif = bit::field<3>;
            // Hash multicast (rm page 1178)
            using hm = bit::field<2>;
            // Hash unicast (rm page 1178)
            using hu = bit::field<1>;
            // Promiscuous mode (rm page 1178)
            using pm = bit::field<0>;
        } // namespace macffr

        // Ethernet MAC hash table high register [offset: 0x0008, reset: 0x0000 0000]
        namespace machthr
        {
            // Hash table high (rm page 1179)
            using hth = bit::field<31, 0>;
        } // namespace machthr

        // Ethernet MAC hash table low register [offset: 0x000C, reset: 0x0000 0000]
        namespace machtlr
        {
            // Hash table low (rm page 1179)
            using htl = bit::field<31, 0>;
        } // namespace machtlr

        // Ethernet MAC MII address register [offset: 0x0010, reset: 0x0000 0000]
        namespace macmiiar
        {
            // PHY address (rm page 1180)
            using pa = bit::field<15, 11>;
            // MII register (rm page 1180)
            using mr = bit::field<10, 6>;
            // Clock range (rm page 1180)
            using cr = bit::field<4, 2>;
            // MII write (rm page 1180)
            using mw = bit::field<1>;
            // MII busy (rm page 1180)
            using mb = bit::field<0>;
        } // namespace macmiiar

        // Ethernet MAC MII data register [offset: 0x0014, reset: 0x0000 0000]
        namespace macmiidr
        {
            // MII data (rm page 1180)
            using md = bit::field<15, 0>;
        } // namespace macmiidr

        // Ethernet MAC flow control register [offset: 0x0018, reset: 0x0000 0000]
        namespace macfcr
        {
            // Pause time (rm page 1181)
            using pt = bit::field<31, 16>;
            // Zero-quanta pause disable (rm page 1181)
            using zqpd = bit::field<7>;
            // Pause low threshold (rm page 1181)
            using plt = bit::field<5, 4>;
            // Unicast pause frame detect (rm page 1181)
            using upfd = bit::field<3>;
            // Receive flow control enable (rm page 1182)
            using rfce = bit::field<2>;
            // Transmit flow control enable (rm page 1182)
            using tfce = bit::field<1>;
            // Flow control busy/back pressure activate (rm page 1182)
            using fcb_bpa = bit::field<0>;
        } // namespace macfcr

        // Ethernet MAC VLAN tag register [offset: 0x001C, reset: 0x0000 0000]
        namespace macvlantr
        {
            // 12-bit VLAN tag comparison (rm page 1183)
            using vlantc = bit::field<16>;
            // VLAN tag identifier (for receive frames) (rm page 1183)
            using vlanti = bit::field<15, 0>;
        } // namespace macvlantr

        // Ethernet MAC PMT control and status register [offset: 0x002C, reset: 0x0000 0000]
        namespace macpmtcsr
        {
            // Wakeup frame filter register pointer reset (rm page 1184)
            using wffrpr = bit::field<31>;
            // Global unicast (rm page 1184)
            using gu = bit::field<9>;
            // Wakeup frame received (rm page 1184)
            using wfr = bit::field<6>;
            // Magic packet received (rm page 1184)
            using mpr = bit::field<5>;
            // Wakeup frame enable (rm page 1184)
            using wfe = bit::field<2>;
            // Magic Packet enable (rm page 1184)
            using mpe = bit::field<1>;
            // Power down (rm page 1184)
            using pd = bit::field<0>;
        } // namespace macpmtcsr

        // Ethernet MAC debug register [offset: 0x0034, reset: 0x0000 0000]
        namespace macdbgr
        {
            // Tx FIFO full (rm page 1185)
            using tff = bit::field<25>;
            // Tx FIFO not empty (rm page 1185)
            using tfne = bit::field<24>;
            // Tx FIFO write active (rm page 1185)
            using tfwa = bit::field<22>;
            // Tx FIFO read status (rm page 1185)
            using tfrs = bit::field<21, 20>;
            // MAC transmitter in pause (rm page 1185)
            using mtp = bit::field<19>;
            // MAC transmit frame controller status (rm page 1185)
            using mtfcs = bit::field<18, 17>;
            // MAC MII transmit engine active (rm page 1185)
            using mmtea = bit::field<16>;
            // Rx FIFO fill level (rm page 1186)
            using rffl = bit::field<9, 8>;
            // Rx FIFO read controller status (rm page 1186)
            using rfrcs = bit::field<6, 5>;
            // Rx FIFO write controller active (rm page 1186)
            using rfwra = bit::field<4>;
            // MAC small FIFO read / write controllers status (rm page 1186)
            using msfrwcs = bit::field<2, 1>;
            // MAC MII receive protocol engine active (rm page 1186)
            using mmrpea = bit::field<0>;
        } // namespace macdbgr

        // Ethernet MAC interrupt status register [offset: 0x0038, reset: 0x0000 0000]
        namespace macsr
        {
            // Time stamp trigger status (rm page 1187)
            using tsts = bit::field<9>;
            // MMC transmit status (rm page 1187)
            using mmcts = bit::field<6>;
            // MMC receive status (rm page 1187)
            using mmcrs = bit::field<5>;
            // MMC status (rm page 1187)
            using mmcs = bit::field<4>;
            // PMT status (rm page 1187)
            using pmts = bit::field<3>;
        } // namespace macsr

        // Ethernet MAC interrupt mask register [offset: 0x003C, reset: 0x0000 0000]
        namespace macimr
        {
            // Time stamp trigger interrupt mask (rm page 1188)
            using tstim = bit::field<9>;
            // PMT interrupt mask (rm page 1188)
            using pmtim = bit::field<3>;
        } // namespace macimr

        // Ethernet MAC address 0 high register [offset: 0x0040, reset: 0x8000 FFFF]
        namespace maca0hr
        {
            // Always 1. (rm page 1188)
            using mo = bit::field<31>;
            // MAC address0 high [47:32] (rm page 1188)
            using maca0h = bit::field<15, 0>;
        } // namespace maca0hr

        // Ethernet MAC address 0 low register [offset: 0x0044, reset: 0xFFFF FFFF]
        namespace maca0lr
        {
            // MAC address0 low [31:0] (rm page 1189)
            using maca0l = bit::field<31, 0>;
        } // namespace maca0lr

        // Ethernet MAC address 1 high register [offset: 0x0048, reset: 0x0000 FFFF]
        namespace maca1hr
        {
            // Address enable (rm page 1189)
            using ae = bit::field<31>;
            // Source address (rm page 1189)
            using sa = bit::field<30>;
            // Mask byte control (rm page 1190)
            using mbc = bit::field<29, 24>;
            // MAC address1 high [47:32] (rm page 1190)
            using maca1h = bit::field<15, 0>;
        } // namespace maca1hr

        // Ethernet MAC address1 low register [offset: 0x004C, reset: 0xFFFF FFFF]
        namespace maca1lr
        {
            // MAC address1 low [31:0] (rm page 1190)
            using maca1l = bit::field<31, 0>;
        } // namespace maca1lr

        // Ethernet MAC address 2 high register [offset: 0x0050, reset: 0x0000 FFFF]
        namespace maca2hr
        {
            // Address enable (rm page 1191)
            using ae = bit::field<31>;
            // Source address (rm page 1191)
            using sa = bit::field<30>;
            // Mask byte control (rm page 1191)
            using mbc = bit::field<29, 24>;
            // MAC address2 high [47:32] (rm page 1191)
            using maca2h = bit::field<15, 0>;
        } // namespace maca2hr

        // Ethernet MAC address 2 low register [offset: 0x0054, reset: 0xFFFF FFFF]
        namespace maca2lr
        {
            // MAC address2 low [31:0] (rm page 1191)
            using maca2l = bit::field<31, 0>;
        } // namespace maca2lr

        // Ethernet MAC address 3 high register [offset: 0x0058, reset: 0x0000 FFFF]
        namespace maca3hr
        {
            // Address enable (rm page 1192)
            using ae = bit::field<31>;
            // Source address (rm page 1192)
            using sa = bit::field<30>;
            // Mask byte control (rm page 1192)
            using mbc = bit::field<29, 24>;
            // MAC address3 high [47:32] (rm page 1192)
            using maca3h = bit::field<15, 0>;
        } // namespace maca3hr

        // Ethernet MAC address 3 low register [offset: 0x005C, reset: 0xFFFF FFFF]
        namespace maca3lr
        {
            // MAC address3 low [31:0] (rm page 1192)
            using maca3l = bit::field<31, 0>;
        } // namespace maca3lr

        // Ethernet MMC control register [offset: 0x0100, reset: 0x0000 0000]
        namespace mmccr
        {
            // MMC counter Full-Half preset (rm page 1193)
            using mcfhp = bit::field<5>;
            // MMC counter freeze (rm page 1193)
            using mcf = bit::field<3>;
            // Reset on read (rm page 1193)
            using ror = bit::field<2>;
            // Counter stop rollover (rm page 1193)
            using csr = bit::field<1>;
            // Counter reset (rm page 1193)
            using cr = bit::field<0>;
        } // namespace mmccr

        // Ethernet MMC receive interrupt register [offset: 0x0104, reset: 0x0000 0000]
        namespace mmcrir
        {
            // Received Good Unicast Frames Status (rm page 1194)
            using rgufs = bit::field<17>;
            // Received frames alignment error status (rm page 1194)
            using rfaes = bit::field<6>;
            // Received frames CRC error status (rm page 1194)
            using rfces = bit::field<5>;
        } // namespace mmcrir

        // Ethernet MMC transmit interrupt register [offset: 0x0108, reset: 0x0000 0000]
        namespace mmctir
        {
            // Transmitted good frames status (rm page 1194)
            using tgfs = bit::field<21>;
            // Transmitted good frames more single collision status (rm page 1195)
            using tgfmscs = bit::field<15>;
            // Transmitted good frames single collision status (rm page 1195)
            using tgfscs = bit::field<14>;
        } // namespace mmctir

        // Ethernet MMC receive interrupt mask register [offset: 0x010C, reset: 0x0000 0000]
        namespace mmcrimr
        {
            // Received good unicast frames mask (rm page 1195)
            using rgufm = bit::field<17>;
            // Received frames alignment error mask (rm page 1195)
            using rfaem = bit::field<6>;
            // Received frame CRC error mask (rm page 1195)
            using rfcem = bit::field<5>;
        } // namespace mmcrimr

        // Ethernet MMC transmit interrupt mask register [offset: 0x0110, reset: 0x0000 0000]
        namespace mmctimr
        {
            // Transmitted good frames mask (rm page 1196)
            using tgfm = bit::field<21>;
            // Transmitted good frames more single collision mask (rm page 1196)
            using tgfmscm = bit::field<15>;
            // Transmitted good frames single collision mask (rm page 1196)
            using tgfscm = bit::field<14>;
        } // namespace mmctimr

        // Ethernet MMC transmitted good frames after a single collision counter register [offset: 0x014C, reset: 0x0000 0000]
        namespace mmctgfsccr
        {
            // Transmitted good frames single collision counter (rm page 1196)
            using tgfscc = bit::field<31, 0>;
        } // namespace mmctgfsccr

        // Ethernet MMC transmitted good frames after more than a single collision counter register [offset: 0x0150, reset: 0x0000 0000]
        namespace mmctgfmsccr
        {
            // Transmitted good frames more single collision counter (rm page 1197)
            using tgfmscc = bit::field<31, 0>;
        } // namespace mmctgfmsccr

        // Ethernet MMC transmitted good frames counter register [offset: 0x0168, reset: 0x0000 0000]
        namespace mmctgfcr
        {
            // Transmitted good frames counter (rm page 1197)
            using tgfc = bit::field<31, 0>;
        } // namespace mmctgfcr

        // Ethernet MMC received frames with CRC error counter register [offset: 0x0194, reset: 0x0000 0000]
        namespace mmcrfcecr
        {
            // Received frames CRC error counter (rm page 1197)
            using rfcec = bit::field<31, 0>;
        } // namespace mmcrfcecr

        // Ethernet MMC received frames with alignment error counter register [offset: 0x0198, reset: 0x0000 0000]
        namespace mmcrfaecr
        {
            // Received frames alignment error counter (rm page 1197)
            using rfaec = bit::field<31, 0>;
        } // namespace mmcrfaecr

        // MMC received good unicast frames counter register [offset: 0x01C4, reset: 0x0000 0000]
        namespace mmcrgufcr
        {
            // Received good unicast frames counter (rm page 1198)
            using rgufc = bit::field<31, 0>;
        } // namespace mmcrgufcr

        // Ethernet PTP time stamp control register [offset: 0x0700, reset: 0x0000 00002000]
        namespace ptptscr
        {
            // Time stamp PTP frame filtering MAC address enable			 (rm page 1198)
            using tspffmae = bit::field<18>;
            // Time stamp clock node type (rm page 1198)
            using tscnt = bit::field<17, 16>;
            // Time stamp snapshot for message relevant to master enable			 (rm page 1198)
            using tssmrme = bit::field<15>;
            // Time stamp snapshot for event message enable (rm page 1198)
            using tsseme = bit::field<14>;
            // Time stamp snapshot for IPv4 frames enable (rm page 1199)
            using tssipv4fe = bit::field<13>;
            // Time stamp snapshot for IPv6 frames enable (rm page 1199)
            using tssipv6fe = bit::field<12>;
            // Time stamp snapshot for PTP over ethernet frames enable (rm page 1199)
            using tssptpoefe = bit::field<11>;
            // Time stamp PTP packet snooping for version2 format enable (rm page 1199)
            using tsptppsv2e = bit::field<10>;
            // Time stamp subsecond rollover: digital or binary rollover control (rm page 1199)
            using tsssr = bit::field<9>;
            // Time stamp snapshot for all received frames enable (rm page 1199)
            using tssarfe = bit::field<8>;
            // Time stamp addend register update (rm page 1199)
            using tsaru = bit::field<5>;
            // Time stamp interrupt trigger enable (rm page 1199)
            using tsite = bit::field<4>;
            // Time stamp system time update (rm page 1199)
            using tsstu = bit::field<3>;
            // Time stamp system time initialize (rm page 1200)
            using tssti = bit::field<2>;
            // Time stamp fine or coarse update (rm page 1200)
            using tsfcu = bit::field<1>;
            // Time stamp enable (rm page 1200)
            using tse = bit::field<0>;
        } // namespace ptptscr

        // Ethernet PTP subsecond increment register [offset: 0x0704, reset: 0x0000 0000]
        namespace ptpssir
        {
            // System time subsecond increment (rm page 1201)
            using stssi = bit::field<7, 0>;
        } // namespace ptpssir

        // Ethernet PTP time stamp high register [offset: 0x0708, reset: 0x0000 0000]
        namespace ptptshr
        {
            // System time second (rm page 1201)
            using sts = bit::field<31, 0>;
        } // namespace ptptshr

        // Ethernet PTP time stamp low register [offset: 0x070C, reset: 0x0000 0000]
        namespace ptptslr
        {
            // System time positive or negative sign (rm page 1201)
            using stpns = bit::field<31>;
            // System time subseconds (rm page 1201)
            using stss = bit::field<30, 0>;
        } // namespace ptptslr

        // Ethernet PTP time stamp high update register [offset: 0x0710, reset: 0x0000 0000]
        namespace ptptshur
        {
            // Time stamp update second (rm page 1202)
            using tsus = bit::field<31, 0>;
        } // namespace ptptshur

        // TSUPNS             Ethernet PTP time stamp low update register [offset: 0x0714, reset: 0x0000 0000]
        namespace ptptslur
        {
            // Time stamp update positive or negative sign (rm page 1203)
            using tsupns = bit::field<31>;
            // Time stamp update subseconds (rm page 1203)
            using tsuss = bit::field<30, 0>;
        } // namespace ptptslur

        // Ethernet PTP time stamp addend register [offset: 0x0718, reset: 0x0000 0000]
        namespace ptptsar
        {
            // Time stamp addend (rm page 1203)
            using tsa = bit::field<31, 0>;
        } // namespace ptptsar

        // Ethernet PTP target time high register [offset: 0x071C, reset: 0x0000 0000]
        namespace ptptthr
        {
            // Target time stamp high (rm page 1204)
            using ttsh = bit::field<31, 0>;
        } // namespace ptptthr

        // Ethernet PTP target time low register [offset: 0x0720, reset: 0x0000 0000]
        namespace ptpttlr
        {
            // Target time stamp low (rm page 1204)
            using ttsl = bit::field<31, 0>;
        } // namespace ptpttlr

        // Ethernet PTP time stamp status register [offset: 0x0728, reset: 0x0000 0000]
        namespace ptptssr
        {
            // Time stamp target time reached (rm page 1205)
            using tsttr = bit::field<1>;
            // Time stamp second overflow (rm page 1205)
            using tsso = bit::field<0>;
        } // namespace ptptssr

        // Ethernet PTP PPS control register [offset: 0x072C, reset: 0x0000 0000]
        namespace ptpppscr
        {
            // PPS frequency selection (rm page 1205)
            using ppsfreq = bit::field<3, 0>;
        } // namespace ptpppscr

        // Ethernet DMA bus mode register [offset: 0x1000, reset: 0x0002 0101]
        namespace dmabmr
        {
            // Mixed burst (rm page 1206)
            using mb = bit::field<26>;
            // Address-aligned beats (rm page 1206)
            using aab = bit::field<25>;
            // 4xPBL mode (rm page 1206)
            using fpm = bit::field<24>;
            // Use separate PBL (rm page 1206)
            using usp = bit::field<23>;
            // Rx DMA PBL (rm page 1206)
            using rdp = bit::field<22, 17>;
            // Fixed burst (rm page 1206)
            using fb = bit::field<16>;
            // Rx Tx priority ratio (rm page 1206)
            using pm = bit::field<15, 14>;
            // Programmable burst length (rm page 1207)
            using pbl = bit::field<13, 8>;
            // Enhanced descriptor format enable (rm page 1207)
            using edfe = bit::field<7>;
            // Descriptor skip length (rm page 1207)
            using dsl = bit::field<6, 2>;
            // DMA Arbitration (rm page 1207)
            using da = bit::field<1>;
            // Software reset (rm page 1207)
            using sr = bit::field<0>;
        } // namespace dmabmr

        // Ethernet DMA transmit poll demand register [offset: 0x1004, reset: 0x0000 0000]
        namespace dmatpdr
        {
            // Transmit poll demand (rm page 1208)
            using tpd = bit::field<31, 0>;
        } // namespace dmatpdr

        // EHERNET DMA receive poll demand register [offset: 0x1008, reset: 0x0000 0000]
        namespace dmarpdr
        {
            // Receive poll demand (rm page 1208)
            using rpd = bit::field<31, 0>;
        } // namespace dmarpdr

        // Ethernet DMA receive descriptor list address register [offset: 0x100C, reset: 0x0000 0000]
        namespace dmardlar
        {
            // Start of receive list (rm page 1208)
            using srl = bit::field<31, 0>;
        } // namespace dmardlar

        // Ethernet DMA transmit descriptor list address register [offset: 0x1010, reset: 0x0000 0000]
        namespace dmatdlar
        {
            // Start of transmit list (rm page 1209)
            using stl = bit::field<31, 0>;
        } // namespace dmatdlar

        // Ethernet DMA status register [offset: 0x1014, reset: 0x0000 0000]
        namespace dmasr
        {
            // Time stamp trigger status (rm page 1209)
            using tsts = bit::field<29>;
            // PMT status (rm page 1209)
            using pmts = bit::field<28>;
            // MMC status (rm page 1210)
            using mmcs = bit::field<27>;
            // Error bits status (rm page 1210)
            using ebs = bit::field<25, 23>;
            // Transmit process state (rm page 1210)
            using tps = bit::field<22, 20>;
            // Receive process state (rm page 1210)
            using rps = bit::field<19, 17>;
            // Normal interrupt summary (rm page 1210)
            using nis = bit::field<16>;
            // Abnormal interrupt summary (rm page 1211)
            using ais = bit::field<15>;
            // Early receive status (rm page 1211)
            using ers = bit::field<14>;
            // Fatal bus error status (rm page 1211)
            using fbes = bit::field<13>;
            // Early transmit status (rm page 1211)
            using ets = bit::field<10>;
            // Receive watchdog timeout status (rm page 1211)
            using rwts = bit::field<9>;
            // Receive process stopped status (rm page 1211)
            using rpss = bit::field<8>;
            // Receive buffer unavailable status (rm page 1211)
            using rbus = bit::field<7>;
            // Receive status (rm page 1211)
            using rs = bit::field<6>;
            // Transmit underflow status (rm page 1211)
            using tus = bit::field<5>;
            // Receive overflow status (rm page 1211)
            using ros = bit::field<4>;
            // Transmit jabber timeout status (rm page 1212)
            using tjts = bit::field<3>;
            // Transmit buffer unavailable status (rm page 1212)
            using tbus = bit::field<2>;
            // Transmit process stopped status (rm page 1212)
            using tpss = bit::field<1>;
            // Transmit status (rm page 1212)
            using ts = bit::field<0>;
        } // namespace dmasr

        // Ethernet DMA operation mode register [offset: 0x1018, reset: 0x0000 0000]
        namespace dmaomr
        {
            // Dropping of TCP/IP checksum error frames disable (rm page 1212)
            using dtcefd = bit::field<26>;
            // Receive store and forward (rm page 1212)
            using rsf = bit::field<25>;
            // Disable flushing of received frames (rm page 1212)
            using dfrf = bit::field<24>;
            // Transmit store and forward (rm page 1213)
            using tsf = bit::field<21>;
            // Flush transmit FIFO (rm page 1213)
            using ftf = bit::field<20>;
            // Transmit threshold control (rm page 1213)
            using ttc = bit::field<16, 14>;
            // Start/stop transmission (rm page 1213)
            using st = bit::field<13>;
            // Forward error frames (rm page 1213)
            using fef = bit::field<7>;
            // Forward undersized good frames (rm page 1214)
            using fugf = bit::field<6>;
            // Receive threshold control (rm page 1214)
            using rtc = bit::field<4, 3>;
            // Operate on second frame (rm page 1214)
            using osf = bit::field<2>;
            // Start/stop receive (rm page 1214)
            using sr = bit::field<1>;
        } // namespace dmaomr

        // Ethernet DMA interrupt enable register [offset: 0x1020, reset: 0x0000 0000]
        namespace dmaier
        {
            // Normal interrupt summary enable (rm page 1215)
            using nise = bit::field<16>;
            // Abnormal interrupt summary enable (rm page 1215)
            using aise = bit::field<15>;
            // Early receive interrupt enable (rm page 1215)
            using erie = bit::field<14>;
            // Fatal bus error interrupt enable (rm page 1215)
            using fbeie = bit::field<13>;
            // Early transmit interrupt enable (rm page 1215)
            using etie = bit::field<10>;
            // receive watchdog timeout interrupt enable (rm page 1216)
            using rwtie = bit::field<9>;
            // Receive process stopped interrupt enable (rm page 1216)
            using rpsie = bit::field<8>;
            // Receive buffer unavailable interrupt enable (rm page 1216)
            using rbuie = bit::field<7>;
            // Receive interrupt enable (rm page 1216)
            using rie = bit::field<6>;
            // Underflow interrupt enable (rm page 1216)
            using tuie = bit::field<5>;
            // Overflow interrupt enable (rm page 1216)
            using roie = bit::field<4>;
            // Transmit jabber timeout interrupt enable (rm page 1216)
            using tjtie = bit::field<3>;
            // Transmit buffer unavailable interrupt enable (rm page 1216)
            using tbuie = bit::field<2>;
            // Transmit process stopped interrupt enable (rm page 1216)
            using tpsie = bit::field<1>;
            // Transmit interrupt enable (rm page 1216)
            using tie = bit::field<0>;
            // Overflow bit for FIFO overflow counter (rm page 1217)
            using ofoc = bit::field<28>;
            // Missed frames by the application (rm page 1217)
            using mfa = bit::field<27, 17>;
            // Overflow bit for missed frame counter (rm page 1217)
            using omfc = bit::field<16>;
            // Missed frames by the controller (rm page 1217)
            using mfc = bit::field<15, 0>;
        } // namespace dmaier

        // Ethernet DMA receive status watchdog timer register [offset: 0x1024, reset: 0x0000 0000]
        namespace dmarswtr
        {
            // Receive status (RS) watchdog timer count (rm page 1217)
            using rswtc = bit::field<7, 0>;
        } // namespace dmarswtr

        // Ethernet DMA current host transmit descriptor register [offset: 0x1048, reset: 0x0000 0000]
        namespace dmachtdr
        {
            // Host transmit descriptor address pointer (rm page 1218)
            using htdap = bit::field<31, 0>;
        } // namespace dmachtdr

        // Ethernet DMA current host receive descriptor register [offset: 0x104C, reset: 0x0000 0000]
        namespace dmachrdr
        {
            // Host receive descriptor address pointer (rm page 1218)
            using hrdap = bit::field<31, 0>;
        } // namespace dmachrdr

        // Ethernet DMA current host transmit buffer address register [offset: 0x1050, reset: 0x0000 0000]
        namespace dmachtbar
        {
            // Host transmit buffer address pointer (rm page 1218)
            using htbap = bit::field<31, 0>;
        } // namespace dmachtbar

        // Ethernet DMA current host receive buffer address register [offset: 0x1054, reset: 0x0000 0000]
        namespace dmachrbar
        {
            // Host receive buffer address pointer (rm page 1219)
            using hrbap = bit::field<31, 0>;
        } // namespace dmachrbar

    } // namespace fields

    struct register_map
    {
        // 0 MACCR: Ethernet MAC configuration register (rm page 1153)
        bit::register_base maccr;
        // 4 MACFFR: Ethernet MAC frame filter register (rm page 1177)
        bit::register_base macffr;
        // 8 MACHTHR: Ethernet MAC hash table high register (rm page 1178)
        bit::register_base machthr;
        // 12 MACHTLR: Ethernet MAC hash table low register (rm page 1179)
        bit::register_base machtlr;
        // 16 MACMIIAR: Ethernet MAC MII address register (rm page 1179)
        bit::register_base macmiiar;
        // 20 MACMIIDR: Ethernet MAC MII data register (rm page 1180)
        bit::register_base macmiidr;
        // 24 MACFCR: Ethernet MAC flow control register (rm page 1181)
        bit::register_base macfcr;
        // 28 MACVLANTR: Ethernet MAC VLAN tag register (rm page 1182)
        bit::register_base macvlantr;
        uint8 zzzoffset0[12];
        // 44 MACPMTCSR: Ethernet MAC PMT control and status register (rm page 1184)
        bit::register_base macpmtcsr;
        uint32 zzzoffset1;
        // 52 MACDBGR: Ethernet MAC debug register (rm page 1185)
        bit::register_base macdbgr;
        // 56 MACSR: Ethernet MAC interrupt status register (rm page 1187)
        bit::register_base macsr;
        // 60 MACIMR: Ethernet MAC interrupt mask register (rm page 1188)
        bit::register_base macimr;
        // 64 MACA0HR: Ethernet MAC address 0 high register (rm page 1188)
        bit::register_base maca0hr;
        // 68 MACA0LR: Ethernet MAC address 0 low register (rm page 1189)
        bit::register_base maca0lr;
        // 72 MACA1HR: Ethernet MAC address 1 high register (rm page 1189)
        bit::register_base maca1hr;
        // 76 MACA1LR: Ethernet MAC address1 low register (rm page 1190)
        bit::register_base maca1lr;
        // 80 MACA2HR: Ethernet MAC address 2 high register (rm page 1190)
        bit::register_base maca2hr;
        // 84 MACA2LR: Ethernet MAC address 2 low register (rm page 1191)
        bit::register_base maca2lr;
        // 88 MACA3HR: Ethernet MAC address 3 high register (rm page 1191)
        bit::register_base maca3hr;
        // 92 MACA3LR: Ethernet MAC address 3 low register (rm page 1192)
        bit::register_base maca3lr;
        uint8 zzzoffset2[160];
        // 256 MMCCR: Ethernet MMC control register (rm page 1193)
        bit::register_base mmccr;
        // 260 MMCRIR: Ethernet MMC receive interrupt register (rm page 1193)
        bit::register_base mmcrir;
        // 264 MMCTIR: Ethernet MMC transmit interrupt register (rm page 1194)
        bit::register_base mmctir;
        // 268 MMCRIMR: Ethernet MMC receive interrupt mask register (rm page 1195)
        bit::register_base mmcrimr;
        // 272 MMCTIMR: Ethernet MMC transmit interrupt mask register (rm page 1195)
        bit::register_base mmctimr;
        uint8 zzzoffset3[56];
        // 332 MMCTGFSCCR: Ethernet MMC transmitted good frames after a single collision counter register (rm page 1196)
        bit::register_base mmctgfsccr;
        // 336 MMCTGFMSCCR: Ethernet MMC transmitted good frames after more than a single collision counter register (rm page 1196)
        bit::register_base mmctgfmsccr;
        uint8 zzzoffset4[20];
        // 360 MMCTGFCR: Ethernet MMC transmitted good frames counter register (rm page 1197)
        bit::register_base mmctgfcr;
        uint8 zzzoffset5[40];
        // 404 MMCRFCECR: Ethernet MMC received frames with CRC error counter register (rm page 1197)
        bit::register_base mmcrfcecr;
        // 408 MMCRFAECR: Ethernet MMC received frames with alignment error counter register (rm page 1197)
        bit::register_base mmcrfaecr;
        uint8 zzzoffset6[40];
        // 452 MMCRGUFCR: MMC received good unicast frames counter register (rm page 1198)
        bit::register_base mmcrgufcr;
        uint8 zzzoffset7[1336];
        // 1792 PTPTSCR: Ethernet PTP time stamp control register (rm page 1198)
        bit::register_base ptptscr;
        // 1796 PTPSSIR: Ethernet PTP subsecond increment register (rm page 1200)
        bit::register_base ptpssir;
        // 1800 PTPTSHR: Ethernet PTP time stamp high register (rm page 1201)
        bit::register_base ptptshr;
        // 1804 PTPTSLR: Ethernet PTP time stamp low register (rm page 1201)
        bit::register_base ptptslr;
        // 1808 PTPTSHUR: Ethernet PTP time stamp high update register (rm page 1201)
        bit::register_base ptptshur;
        // 1812 PTPTSLUR: TSUPNS             Ethernet PTP time stamp low update register (rm page 1203)
        bit::register_base ptptslur;
        // 1816 PTPTSAR: Ethernet PTP time stamp addend register (rm page 1203)
        bit::register_base ptptsar;
        // 1820 PTPTTHR: Ethernet PTP target time high register (rm page 1204)
        bit::register_base ptptthr;
        // 1824 PTPTTLR: Ethernet PTP target time low register (rm page 1204)
        bit::register_base ptpttlr;
        uint32 zzzoffset8;
        // 1832 PTPTSSR: Ethernet PTP time stamp status register (rm page 1204)
        bit::register_base ptptssr;
        // 1836 PTPPPSCR: Ethernet PTP PPS control register (rm page 1205)
        bit::register_base ptpppscr;
        uint8 zzzoffset9[2256];
        // 4096 DMABMR: Ethernet DMA bus mode register (rm page 1205)
        bit::register_base dmabmr;
        // 4100 DMATPDR: Ethernet DMA transmit poll demand register (rm page 1207)
        bit::register_base dmatpdr;
        // 4104 DMARPDR: EHERNET DMA receive poll demand register (rm page 1208)
        bit::register_base dmarpdr;
        // 4108 DMARDLAR: Ethernet DMA receive descriptor list address register (rm page 1208)
        bit::register_base dmardlar;
        // 4112 DMATDLAR: Ethernet DMA transmit descriptor list address register (rm page 1209)
        bit::register_base dmatdlar;
        // 4116 DMASR: Ethernet DMA status register (rm page 1209)
        bit::register_base dmasr;
        // 4120 DMAOMR: Ethernet DMA operation mode register (rm page 1212)
        bit::register_base dmaomr;
        // 4124 DMAIER: Ethernet DMA interrupt enable register (rm page 1215)
        bit::register_base dmaier;
        // 4128 DMARSWTR: Ethernet DMA receive status watchdog timer register (rm page 1217)
        bit::register_base dmarswtr;
        uint8 zzzoffset11[32];
        // 4136 DMACHTDR: Ethernet DMA current host transmit descriptor register (rm page 1218)
        bit::register_base dmachtdr;
        // DMACHRDR: Ethernet DMA current host receive descriptor register (rm page 1218)
        bit::register_base dmachrdr;
        // DMACHTBAR: Ethernet DMA current host transmit buffer address register (rm page 1218)
        bit::register_base dmachtbar;
        // DMACHRBAR: Ethernet DMA current host receive buffer address register (rm page 1219)
        bit::register_base dmachrbar;
    };

    extern "C" register_map __eth__device_0x40028000;
    static auto &device = __eth__device_0x40028000;

} // namespace eth

#endif
