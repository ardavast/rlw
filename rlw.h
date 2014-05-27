#ifndef _RLW_H_
#define _RLW_H_

#define RLW_TIMEOUT 1000

#define	RLW_IDR0 0x0
#define	RLW_IDR1 0x1
#define	RLW_IDR2 0x2
#define	RLW_IDR3 0x3
#define	RLW_IDR4 0x4
#define	RLW_IDR5 0x5

#define	RLW_CR     0x0037
#define RLW_CR_RST (1 << 4)

#define RLW_RCR         0x0044
#define RLW_RCR_9356SEL (1 << 6)

#define RLW_RF_OUT 0x80
#define RLW_RF_ENB 0x82
#define RLW_RF_SEL 0x84
#define RLW_RF_INP 0x86

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

#define RLW_BB_HOST_BANG_CLK (1 << 1)
#define RLW_BB_HOST_BANG_EN  (1 << 2)
#define RLW_BB_HOST_BANG_RW  (1 << 3)

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
  uint8_t            rlw_txpwr_ofdm[RLW_MAX_CHANNELS];
  struct callout     rlw_stat_callout;
  struct mtx         rlw_mtx;
#define RLW_LOCK(_sc)        mtx_lock(&(_sc)->rlw_mtx)
#define RLW_UNLOCK(_sc)      mtx_unlock(&(_sc)->rlw_mtx)
#define	RLW_LOCK_ASSERT(_sc) mtx_assert(&(_sc)->rlw_mtx, MA_OWNED)
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

#define CSR_SETBIT_1(sc, offset, val) \
  CSR_WRITE_1(sc, offset, CSR_READ_1(sc, offset) | (val))

#define CSR_CLRBIT_1(sc, offset, val) \
  CSR_WRITE_1(sc, offset, CSR_READ_1(sc, offset) & ~(val))

#define EE_SET(x) \
  CSR_WRITE_1(sc, RLW_9346CR, CSR_READ_1(sc, RLW_9346CR) | x)

#define EE_CLR(x) \
  CSR_WRITE_1(sc, RLW_9346CR, CSR_READ_1(sc, RLW_9346CR) & ~x)

#endif /* _RLW_H_ */
