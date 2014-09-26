#ifndef _RLW_H_
#define _RLW_H_

#include <net80211/ieee80211.h>

#define RLW_TIMEOUT 1000

#define	RLW_IDR0 0x0
#define	RLW_IDR1 0x1
#define	RLW_IDR2 0x2
#define	RLW_IDR3 0x3
#define	RLW_IDR4 0x4
#define	RLW_IDR5 0x5

#define	RLW_CR     0x0037
#define RLW_CR_RST (1 << 4)
#define RLW_CR_RE  (1 << 3)
#define RLW_CR_TE  (1 << 2)

#define RLW_IMR 0x003c

#define RLW_TCR             0x0040
#define RLW_TCR_CWMIN           (1U << 31)
#define RLW_TCR_HWVERID_RTL8180 (x << 25)
#define RLW_TCR_SAT             (1 << 24)
#define RLW_TCR_MXDMA16         (0 << 8)
#define RLW_TCR_MXDMA32         (1 << 8)
#define RLW_TCR_MXDMA64         (2 << 8)
#define RLW_TCR_MXDMA128        (3 << 8)
#define RLW_TCR_MXDMA256        (4 << 8)
#define RLW_TCR_MXDMA512        (5 << 8)
#define RLW_TCR_MXDMA1024       (6 << 8)
#define RLW_TCR_MXDMA2048       (7 << 8)
#define RLW_TCR_DISCW           (1 << 20)
#define RLW_TCR_ICV             (1 << 19)
#define RLW_TCR_LBK_NORM        (0 << 17)
#define RLW_TCR_LBK_MAC         (1 << 17)
#define RLW_TCR_LBK_BB          (2 << 17)
#define RLW_TCR_LBK_CONTTX      (3 << 17)
#define RLW_TCR_CRC             (1 << 16)
#define RLW_TCR_SRL ebaligokvo
#define RLW_TCR_LRL ebaligokvo

#define RLW_RCR             0x0044
#define RLW_RCR_ONLYERLPKT  (1U << 31)
#define RLW_RCR_ENCS2       (1 << 30)
#define RLW_RCR_ENCS1       (1 << 29)
#define RLW_RCR_ENMARP      (1 << 28)
#define RLW_RCR_CBSSID      (1 << 23)
#define RLW_RCR_APWRMGT     (1 << 22)
#define RLW_RCR_ADD3        (1 << 21)
#define RLW_RCR_AMF         (1 << 20)
#define RLW_RCR_ACF         (1 << 19)
#define RLW_RCR_ADF         (1 << 18)
#define RLW_RCR_RXFTH_MASK  ((1 << 13) | (1 << 14) | (1 << 15))
#define RLW_RCR_RXFTH_64    (2 << 13)
#define RLW_RCR_RXFTH_128   (3 << 13)
#define RLW_RCR_RXFTH_256   (4 << 13)
#define RLW_RCR_RXFTH_512   (5 << 13)
#define RLW_RCR_RXFTH_1024  (6 << 13)
#define RLW_RCR_RXFTH_NONE  (7 << 13)
#define RLW_RCR_AICV        (1 << 12)
#define RLW_RCR_MXDMA_MASK  ((1 << 8) | (1 << 9) | (1 << 10))
#define RLW_RCR_MXDMA_16    (0 << 8)
#define RLW_RCR_MXDMA_32    (1 << 8)
#define RLW_RCR_MXDMA_64    (2 << 8)
#define RLW_RCR_MXDMA_128   (3 << 8)
#define RLW_RCR_MXDMA_256   (4 << 8)
#define RLW_RCR_MXDMA_512   (5 << 8)
#define RLW_RCR_MXDMA_1024  (6 << 8)
#define RLW_RCR_MXDMA_UNLIM (7 << 8)
#define RLW_RCR_9356SEL     (1 << 6)
#define RLW_RCR_ACRC32      (1 << 5)
#define RLW_RCR_AB          (1 << 3)
#define RLW_RCR_AM          (1 << 2)
#define RLW_RCR_APM         (1 << 1)
#define RLW_RCR_AAP         (1 << 0)

#define	RLW_9346CR              0x0050
#define RLW_9346CR_EEDO         (1 << 0)
#define RLW_9346CR_EEDI         (1 << 1)
#define RLW_9346CR_EESK         (1 << 2)
#define RLW_9346CR_EECS         (1 << 3)
#define RLW_9346CR_EEM          (0x40|0x80)
#define RLW_9346CR_EEM_OFF      0x00
#define RLW_9346CR_EEM_AUTOLOAD 0x40
#define RLW_9346CR_EEM_PROGRAM  0x80
#define RLW_9346CR_EEM_WRITECFG (0x80|0x40)
#define RLW_93C46_WIDTH 6
#define RLW_93C56_WIDTH 8
#define RLW_93C46_READ      0x6
#define RLW_93C46_EWEN      0x4
#define RLW_93C46_EWEN_ADDR 0x30
#define RLW_93C46_ERASE     0x7
#define RLW_93C46_WRITE     0x5
#define RLW_93C46_ERAL      0x1
#define RLW_93C46_ERAL_ADDR 0x80
#define RLW_93C46_WRAL      0x2
#define RLW_93C46_WRAL_ADDR 0x40
#define RLW_93C46_EWDS      0x4
#define RLW_93C46_EWDS_ADDR 0x00

#define RLW_RF_OUT 0x80
#define RLW_RF_ENB 0x82
#define RLW_RF_SEL 0x84
#define RLW_RF_INP 0x86

#define RLW_BB_HOST_BANG_CLK (1 << 1)
#define RLW_BB_HOST_BANG_EN  (1 << 2)
#define RLW_BB_HOST_BANG_RW  (1 << 3)

#define RLW_TX_AGC_CTL 0x009c
#define RLW_TX_AGC_CTL_FEEDBACK_ANT     (1 << 2)
#define RLW_TX_AGC_CTL_PERPACKET_ANTSEL (1 << 1)
#define RLW_TX_AGC_CTL_PERPACKET_GAIN   (1 << 0)

#define RLW_CW_CONF                 0x00bc
#define RLW_CW_CONF_PERPACKET_RETRY (1 << 1)
#define RLW_CW_CONF_PERPACKET_CW    (1 << 0)

#define RLW_TX_MAXSIZE 0x9c4

struct rlw_data {
  struct rlw_softc *sc;
  uint8_t *buf;
  uint16_t buflen;
  struct mbuf *m;
  struct ieee80211_node *ni;
  STAILQ_ENTRY(rlw_data) next;
};

typedef STAILQ_HEAD(, rlw_data) rlw_datahead;

struct rlw_softc {
  struct ifnet      *rlw_ifp;
  bus_space_handle_t rlw_bhandle;
  bus_space_tag_t    rlw_btag;
  device_t           rlw_dev;
  struct resource   *rlw_res;
  int                rlw_res_id;
  int                rlw_res_type;
#define RLW_MSI_MESSAGES 1
  struct resource   *rlw_irq[RLW_MSI_MESSAGES];
  int                rlw_eecmd_read;
  int                rlw_eewidth;
  uint8_t            rlw_bssid[IEEE80211_ADDR_LEN];
#define RLW_MAX_CHANNELS 15
  uint8_t            rlw_txpwr_cck[RLW_MAX_CHANNELS];
  //uint8_t            rlw_txpwr_ofdm[RLW_MAX_CHANNELS];
  int                rlw_flags;
#define RLW_INIT_ONCE (1 << 1)
#define RLW_DETACHED  (1 << 2)
  int                rlw_if_flags;
  int                rlw_txtimer;
  struct callout     rlw_stat_callout;
  struct mtx         rlw_mtx;
#define RLW_LOCK(_sc)        mtx_lock(&(_sc)->rlw_mtx)
#define RLW_UNLOCK(_sc)      mtx_unlock(&(_sc)->rlw_mtx)
#define	RLW_LOCK_ASSERT(_sc) mtx_assert(&(_sc)->rlw_mtx, MA_OWNED)
#define RLW_RX_DATA_LIST_COUNT 4
#define RLW_TX_DATA_LIST_COUNT 16
  struct rlw_data   rlw_tx[RLW_TX_DATA_LIST_COUNT];
  rlw_datahead      rlw_tx_active;
  rlw_datahead      rlw_tx_inactive;
  rlw_datahead      rlw_tx_pending;
  struct rlw_data   rlw_rx[RLW_RX_DATA_LIST_COUNT];
  rlw_datahead      rlw_rx_active;
  rlw_datahead      rlw_rx_inactive;
  void              *rlw_tx_dma_buf;
};

#define CSR_READ_4(sc, reg) \
  bus_space_read_4(sc->rlw_btag, sc->rlw_bhandle, reg)
#define CSR_READ_2(sc, reg) \
  bus_space_read_1(sc->rlw_btag, sc->rlw_bhandle, reg)
#define CSR_READ_1(sc, reg) \
  bus_space_read_1(sc->rlw_btag, sc->rlw_bhandle, reg)

#define CSR_WRITE_1(sc, reg, val) \
  bus_space_write_1(sc->rlw_btag, sc->rlw_bhandle, reg, val)

#define CSR_WRITE_2(sc, reg, val) \
  bus_space_write_2(sc->rlw_btag, sc->rlw_bhandle, reg, val)

#define CSR_WRITE_4(sc, reg, val) \
  bus_space_write_4(sc->rlw_btag, sc->rlw_bhandle, reg, val)

#define CSR_SETBIT_1(sc, offset, val) \
  CSR_WRITE_1(sc, offset, CSR_READ_1(sc, offset) | (val))

#define CSR_CLRBIT_1(sc, offset, val) \
  CSR_WRITE_1(sc, offset, CSR_READ_1(sc, offset) & ~(val))

#define EE_SET(x) \
  CSR_WRITE_1(sc, RLW_9346CR, CSR_READ_1(sc, RLW_9346CR) | x)

#define EE_CLR(x) \
  CSR_WRITE_1(sc, RLW_9346CR, CSR_READ_1(sc, RLW_9346CR) & ~x)

#endif /* _RLW_H_ */
