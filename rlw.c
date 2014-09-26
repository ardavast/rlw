#include <sys/param.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/malloc.h>
#include <sys/bus.h> 
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/queue.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <net/if.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/ethernet.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_radiotap.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include "rlw.h"
#include "rtl8225.h"

static int  rlw_attach(device_t);
static int  rlw_detach(device_t);
static void rlw_reset(struct rlw_softc *);

static void rlw_read_eeprom(struct rlw_softc *, caddr_t, int, int);

static void rlw_tx_enable(struct rlw_softc *sc);
static void rlw_rx_enable(struct rlw_softc *sc);

static int  rlw_alloc_data_list(struct rlw_softc *sc, struct rlw_data data[] ,int ndata, int maxsz, void *dma_buf);
static void rlw_free_data_list(struct rlw_softc *sc, struct rlw_data data[], int ndata, int fillmbuf);
static int  rlw_alloc_tx_data_list(struct rlw_softc *sc);
static int  rlw_alloc_rx_data_list(struct rlw_softc *sc);

static void rlw_init(void *arg);
static void rlw_init_locked(void *arg);
static void rlw_stop(struct ifnet *);
static void rlw_stop_locked(struct ifnet *);

static void rlw_set_multi(void *arg);

static int  rlw_ioctl(struct ifnet *, u_long, caddr_t);
static void rlw_start(struct ifnet *);

static int
rlw_probe(device_t dev)
{
  if (pci_get_vendor(dev) == 0x10ec && pci_get_device(dev) == 0x8185) {
    device_set_desc(dev, "rlw device");
    return (BUS_PROBE_DEFAULT);
  }

  return (ENXIO);
}

static int
rlw_attach(device_t dev)
{
  struct rlw_softc *sc;
  struct ifnet *ifp;
  struct ieee80211com *ic;
  int error = 0;
  int rid = 0;
  uint8_t bands = 0;
  uint16_t val;
  int i;

  sc = device_get_softc(dev);
  sc->rlw_dev = dev;

  mtx_init(&sc->rlw_mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK, MTX_DEF);
  callout_init_mtx(&sc->rlw_stat_callout, &sc->rlw_mtx, 0);

  pci_enable_busmaster(dev);

  sc->rlw_res_id = PCIR_BAR(1);
  sc->rlw_res_type = SYS_RES_MEMORY;
  sc->rlw_res = bus_alloc_resource_any(dev, sc->rlw_res_type, &sc->rlw_res_id, RF_ACTIVE);

  if (sc->rlw_res == NULL) {
    device_printf(dev, "couldn't map memory\n");
    error = ENXIO;
    goto fail;
  }

  sc->rlw_btag = rman_get_bustag(sc->rlw_res);
  sc->rlw_bhandle = rman_get_bushandle(sc->rlw_res);

  sc->rlw_irq[0] = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_SHAREABLE | RF_ACTIVE);

  if (sc->rlw_irq[0] == NULL) {
    device_printf(dev, "couldn't map interrupt\n");
    error = ENXIO;
    goto fail;
  }

  RLW_LOCK(sc);
  rlw_reset(sc);
  RLW_UNLOCK(sc);

  if (CSR_READ_4(sc, RLW_RCR) & RLW_RCR_9356SEL)
   sc->rlw_eewidth = RLW_93C56_WIDTH;
  else
   sc->rlw_eewidth = RLW_93C46_WIDTH;

  /* for (i = 0; i < 128; i++) {
    rlw_read_eeprom(sc, (caddr_t)&val, i, 1);
    device_printf(dev, "%x: %x\n", i, val);
  } */ 

  rlw_read_eeprom(sc, (caddr_t)&val, 0x06, 1);
  val &= 0xff;
  if (val != 9 || rtl8225_detect(sc) != 0) {
    device_printf(dev, "RF frontend not supported\n");
    error = ENXIO;
    goto fail;
  }
  
  for (i = 0; i < IEEE80211_ADDR_LEN; i++) {
    sc->rlw_bssid[i] = CSR_READ_1(sc, RLW_IDR0 + i);
    device_printf(dev, "i = %x\n", sc->rlw_bssid[i]);
  }

  for (i = 0; i < 14; i += 2) {
    uint16_t txpwr;
    rlw_read_eeprom(sc, (caddr_t)&txpwr, 0x10 + (i >> 1), 1);
    sc->rlw_txpwr_cck[i] = txpwr & 0xFF;
    sc->rlw_txpwr_cck[i + 1] = txpwr >> 8;
    device_printf(dev, "cck chan %d = %x\n", i, sc->rlw_txpwr_cck[i]);
    device_printf(dev, "cck chan %d = %x\n", i + 1, sc->rlw_txpwr_cck[i + 1]);
  }

/*  for (i = 0; i < 14; i += 2) {
    uint16_t txpwr;
    rlw_read_eeprom(sc, (caddr_t)&txpwr, 0x20 + (i >> 1), 1);
    sc->rlw_txpwr_ofdm[i] = txpwr & 0xFF;
    sc->rlw_txpwr_ofdm[i + 1] = txpwr >> 8;
    device_printf(dev, "ofdm chan %d = %x\n", i, sc->rlw_txpwr_ofdm[i]);
    device_printf(dev, "ofdm chan %d = %x\n", i + 1, sc->rlw_txpwr_ofdm[i + 1]);
  } */

  ifp = sc->rlw_ifp = if_alloc(IFT_IEEE80211);
  if (ifp == NULL) {
    device_printf(sc->rlw_dev, "can not allocate ifnet\n");
    error = ENOMEM;
    goto fail1;
  }

  ifp->if_softc = sc;
  if_initname(ifp, "rlw", device_get_unit(sc->rlw_dev));
  ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
  ifp->if_init = rlw_init;
  ifp->if_ioctl = rlw_ioctl;
  ifp->if_start = rlw_start;
  IFQ_SET_MAXLEN(&ifp->if_snd, ifqmaxlen);
  ifp->if_snd.ifq_drv_maxlen = ifqmaxlen;
  IFQ_SET_READY(&ifp->if_snd);

  ic = ifp->if_l2com;
  ic->ic_ifp = ifp;
  ic->ic_phytype = IEEE80211_T_OFDM;
  ic->ic_opmode = IEEE80211_M_STA;
  ic->ic_caps = IEEE80211_C_STA;

  setbit(&bands, IEEE80211_MODE_11B);
  setbit(&bands, IEEE80211_MODE_11G);
  ieee80211_init_channels(ic, NULL, &bands);

  ieee80211_ifattach(ic, sc->rlw_bssid);

fail1: /* unlock */
fail:
  if (error)
    rlw_detach(dev);
  
  return (error);
}

static int
rlw_detach(device_t dev)
{
  struct rlw_softc *sc;

  sc = device_get_softc(dev);
	bus_generic_detach(dev);

  if (sc->rlw_irq[0])
    bus_release_resource(dev, SYS_RES_IRQ, 0, sc->rlw_irq[0]);
  if (sc->rlw_res)
    bus_release_resource(dev, sc->rlw_res_type, sc->rlw_res_id, sc->rlw_res);

  mtx_destroy(&sc->rlw_mtx);

  return (0);
}

static void
rlw_reset(struct rlw_softc *sc)
{
  int i;

  RLW_LOCK_ASSERT(sc);

  CSR_WRITE_1(sc, RLW_CR, RLW_CR_RST);

  for (i = 0; i < RLW_TIMEOUT; i++) {
    DELAY(10);
    if (!(CSR_READ_1(sc, RLW_CR) & RLW_CR_RST))
      break;
  }
  if (i == RLW_TIMEOUT)
    device_printf(sc->rlw_dev, "reset never completed!\n");
}

static void
rlw_read_eeprom(struct rlw_softc *sc, caddr_t dest, int off, int cnt)
{
  int d;
  int i, j;
  u_int16_t word = 0, *ptr;

  CSR_SETBIT_1(sc, RLW_9346CR, RLW_9346CR_EEM_PROGRAM);
  CSR_SETBIT_1(sc, RLW_9346CR, RLW_9346CR_EECS);
  DELAY(100);

  for (i = 0; i < cnt; i++) {
    /* bitbang the address */
    d = (off + i) | (RLW_93C46_READ << sc->rlw_eewidth);

    for (j = 1 << (sc->rlw_eewidth + 3); j; j >>= 1) {
      if (d & j) {
        EE_SET(RLW_9346CR_EEDI);
      } else {
        EE_CLR(RLW_9346CR_EEDI);
      }
      DELAY(100);
      EE_SET(RLW_9346CR_EESK);
      DELAY(150);
      EE_CLR(RLW_9346CR_EESK);
      DELAY(100);
    }

    /* read the output */
    for (j = 0x8000; j; j >>= 1) {
      EE_SET(RLW_9346CR_EESK);
      DELAY(100);
      if (CSR_READ_1(sc, RLW_9346CR) & RLW_9346CR_EEDO)
        word |= j;
      EE_CLR(RLW_9346CR_EESK);
      DELAY(100);
    }

    *dest = word;

    ptr = (u_int16_t *)(dest + (i * 2));
    *ptr = word;
  }

  CSR_CLRBIT_1(sc, RLW_9346CR, RLW_9346CR_EECS);
  CSR_CLRBIT_1(sc, RLW_9346CR, RLW_9346CR_EEM_PROGRAM);
  DELAY(100);
}

static void
rlw_init(void *arg)
{
  struct rlw_softc *sc = arg;

  RLW_LOCK(sc);
  rlw_init_locked(sc);
  RLW_UNLOCK(sc);
}

static void
rlw_init_locked(void *arg)
{
  struct rlw_softc *sc = arg;
  struct ifnet *ifp = sc->rlw_ifp;
  int ret;

  if (ifp->if_drv_flags & IFF_DRV_RUNNING)
    rlw_stop_locked(ifp);

  rlw_adapter_start(sc);
  if (error != 0)
    goto fail;

  sc->rlw_txtimer = 0;

  if (!(sc->rlw_flags & RLW_INIT_ONCE)) {
          ret = rlw_alloc_rx_data_list(sc);
          if (ret != 0)
                  goto fail;
          ret = rlw_alloc_tx_data_list(sc);
          if (ret != 0)
                  goto fail;
          sc->rlw_flags |= RLW_INIT_ONCE;
  }

  rlw_rx_enable(sc);
  rlw_tx_enable(sc);

  ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
  ifp->if_drv_flags |= IFF_DRV_RUNNING;

fail:
  return;
}


static int
rlw_alloc_data_list(struct rlw_softc *sc, struct rlw_data data[] ,int ndata, int maxsz, void *dma_buf)
{
  int i, error;

  for (i = 0; i < ndata; i++) {
    struct rlw_data *dp = &data[i];
    dp->sc = sc;

    if (dma_buf == NULL) {
      dp->m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
      if (dp->m == NULL) {
        device_printf(sc->rlw_dev, "could not allocate rx mbuf\n");
        error = ENOMEM;
        goto fail;
      }
      dp->buf = mtod(dp->m, uint8_t *);
    } else {
      dp->m = NULL;
      dp->buf = ((uint8_t *)dma_buf) + (i * maxsz);
    }
    dp->ni = NULL;
  }

  return (0);

fail:
  rlw_free_data_list(sc, data, ndata, 1);
  return (error);
}

static void
rlw_free_data_list(struct rlw_softc *sc, struct rlw_data data[], int ndata, int fillmbuf)
{
  int i;

  for (i = 0; i < ndata; i++) {
    struct rlw_data *dp = &data[i];

    if (fillmbuf == 1) {
      if (dp->m != NULL) {
        m_freem(dp->m);
        dp->m = NULL;
        dp->buf = NULL;
      }
    } else {
      dp->buf = NULL;
    }
    if (dp->ni != NULL) {
      ieee80211_free_node(dp->ni);
      dp->ni = NULL;
    }
  }
}

static int
rlw_alloc_tx_data_list(struct rlw_softc *sc)
{
  int error, i;

  error = rlw_alloc_data_list(sc, sc->rlw_tx, RLW_TX_DATA_LIST_COUNT, RLW_TX_MAXSIZE, sc->rlw_tx_dma_buf);
  if (error != 0)
    return (error);

  STAILQ_INIT(&sc->rlw_tx_active);
  STAILQ_INIT(&sc->rlw_tx_inactive);
  STAILQ_INIT(&sc->rlw_tx_pending);

  for (i = 0; i < RLW_TX_DATA_LIST_COUNT; i++)
    STAILQ_INSERT_HEAD(&sc->rlw_tx_inactive, &sc->rlw_tx[i], next);

  return (0);
}

static int
rlw_alloc_rx_data_list(struct rlw_softc *sc)
{
  int error, i;

  error = rlw_alloc_data_list(sc, sc->rlw_rx, RLW_RX_DATA_LIST_COUNT, MCLBYTES, NULL);
  if (error != 0)
    return (error);

  STAILQ_INIT(&sc->rlw_rx_active);
  STAILQ_INIT(&sc->rlw_rx_inactive);

  for (i = 0; i < RLW_RX_DATA_LIST_COUNT; i++)
    STAILQ_INSERT_HEAD(&sc->rlw_rx_inactive, &sc->rlw_rx[i], next);

  return (0);
}

static int
rlw_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
  struct rlw_softc *sc = ifp->if_softc;
  struct ieee80211com *ic = ifp->if_l2com;
  struct ifreq *ifr = (struct ifreq *)data;
  int error = 0;
  int startall = 0;

  RLW_LOCK(sc);
  error = (sc->sc_flags & RLW_DETACHED) ? ENXIO : 0;
  RLW_UNLOCK(sc);
  if (error)
    return (error);

  switch (cmd) {
  case SIOCSIFFLAGS:
    if (ifp->if_flags & IFF_UP) {
      if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
        if ((ifp->if_flags ^ sc->rlw_if_flags) & (IFF_ALLMULTI | IFF_PROMISC))
           rlw_set_multi(sc);
      } else {
        rlw_init(ifp->if_softc);
        startall = 1;
        (void)1;
      }
    } else {
      if (ifp->if_drv_flags & IFF_DRV_RUNNING)
        rlw_stop(ifp);
        (void)1;
    }

    sc->rlw_if_flags = ifp->if_flags;

    if (startall)
      ieee80211_start_all(ic);
    break;
  case SIOCGIFMEDIA:
    error = ifmedia_ioctl(ifp, ifr, &ic->ic_media, cmd);
    break;
  case SIOCGIFADDR:
    error = ether_ioctl(ifp, cmd, data);
    break;
  default:
    error = EINVAL;
    break;
  }

  return (error);
}

static void
rlw_tx_enable(struct rlw_softc *sc)
{
  uint8_t reg;

  reg = CSR_READ_1(sc, RLW_CW_CONF);
  reg &= ~RLW_CW_CONF_PERPACKET_CW;
  reg |= RLW_CW_CONF_PERPACKET_RETRY;
  CSR_WRITE_1(sc, RLW_CW_CONF, reg);

  reg = CSR_READ_1(sc, RLW_TX_AGC_CTL);
  reg &= ~RLW_TX_AGC_CTL_PERPACKET_GAIN;
  reg &= ~RLW_TX_AGC_CTL_PERPACKET_ANTSEL;
  reg |= ~RLW_TX_AGC_CTL_FEEDBACK_ANT;
  CSR_WRITE_1(sc, RLW_TX_AGC_CTL, reg);

  reg = CSR_READ_1(sc, RLW_CR);
  reg |= RLW_CR_TE;
  CSR_WRITE_1(sc, RLW_CR, reg);
}

static void
rlw_rx_enable(struct rlw_softc *sc)
{
  uint32_t reg;

  reg = CSR_READ_4(sc, RLW_RCR);
  reg &= ~RLW_RCR_RXFTH_MASK;
  reg &= ~RLW_RCR_MXDMA_MASK;
  reg |= (RLW_RCR_ONLYERLPKT
        | RLW_RCR_ENMARP
        | RLW_RCR_CBSSID
        | RLW_RCR_AMF
        | RLW_RCR_ADF
        | RLW_RCR_RXFTH_NONE
        | RLW_RCR_MXDMA_UNLIM
        | RLW_RCR_AB
        | RLW_RCR_APM);


  CSR_WRITE_4(sc, RLW_RCR, reg);
}

static void
rlw_start(struct ifnet *ifp)
{
}


static void
rlw_stop(struct ifnet *ifp)
{
  struct rlw_softc *sc = ifp->rlw_softc;
  RLW_LOCK(sc);
  rlw_stop_locked(ifp);
  RLW_UNLOCK(sc);
}

static void
rlw_stop_locked(struct ifnet *ifp)
{
  struct rlw_softc *sc = ifp->if_softc;
  uint8_t reg;

  ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);
  CSR_WRITE_2(sc, RLW_IMR, 0xffff);

  reg = CSR_READ_1(sc, RLW_CMD, 0xffff);
  reg &= ~(URTW_CMD_RX_ENABLE | URTW_CMD_TX_ENABLE);
  urtw_write8_m(sc, URTW_CMD, data8);

  error = sc->sc_rf_stop(sc);
  if (error != 0)
    goto fail;
  error = urtw_set_mode(sc, URTW_EPROM_CMD_CONFIG);
  if (error)
    goto fail;
  urtw_read8_m(sc, URTW_CONFIG4, &data8);
  urtw_write8_m(sc, URTW_CONFIG4, data8 | URTW_CONFIG4_VCOOFF);
  error = urtw_set_mode(sc, URTW_EPROM_CMD_NORMAL);
  if (error)
    goto fail;
fail:
  if (error)
    device_printf(sc->sc_dev, "failed to stop (%s)\n", usbd_errstr(error));

  callout_stop(&sc->sc_watchdog_ch);
  urtw_abort_xfers(sc);
}

static void
rlw_set_multi(void *arg)
{
  struct rlw_softc *sc = arg;
  struct ifnet *ifp = sc->sc_ifp;

  if (!(ifp->if_flags & IFF_UP))
    return;

  ifp->if_flags |= IFF_ALLMULTI;
}

static int
rlw_shutdown(device_t dev)
{
	return (0);
}

static int
rlw_suspend(device_t dev)
{
	return (0);
}

static int
rlw_resume(device_t dev)
{
	return (0);
}

static device_method_t rlw_methods[] = {
	DEVMETHOD(device_probe,		rlw_probe),
	DEVMETHOD(device_attach,	rlw_attach),
	DEVMETHOD(device_detach,	rlw_detach),
	DEVMETHOD(device_shutdown,	rlw_shutdown),
	DEVMETHOD(device_suspend,	rlw_suspend),
	DEVMETHOD(device_resume,	rlw_resume),
	DEVMETHOD_END
};

static devclass_t rlw_devclass;

DEFINE_CLASS_0(rlw, rlw_driver, rlw_methods, sizeof(struct rlw_softc));
DRIVER_MODULE(rlw, pci, rlw_driver, rlw_devclass, 0, 0);
