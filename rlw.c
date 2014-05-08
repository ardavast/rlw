#include <sys/param.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/malloc.h>
#include <sys/bus.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#define RLW_TIMEOUT 1000

#define RLW_RCR         0x0044
#define RLW_RCR_9356SEL (1 << 6)

#define	RLW_CR     0x0037
#define RLW_CR_RST (1 << 4)

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

struct rlw_softc {
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
  struct callout     rlw_stat_callout;
  struct mtx         rlw_mtx;
#define RLW_LOCK(_sc)        mtx_lock(&(_sc)->rlw_mtx)
#define RLW_UNLOCK(_sc)      mtx_unlock(&(_sc)->rlw_mtx)
#define	RLW_LOCK_ASSERT(_sc) mtx_assert(&(_sc)->rlw_mtx, MA_OWNED)
};

#define CSR_READ_4(sc, reg) \
  bus_space_read_4(sc->rlw_btag, sc->rlw_bhandle, reg)
#define CSR_READ_1(sc, reg) \
  bus_space_read_1(sc->rlw_btag, sc->rlw_bhandle, reg)

#define CSR_WRITE_1(sc, reg, val) \
  bus_space_write_1(sc->rlw_btag, sc->rlw_bhandle, reg, val)

#define CSR_SETBIT_1(sc, offset, val) \
  CSR_WRITE_1(sc, offset, CSR_READ_1(sc, offset) | (val))

#define CSR_CLRBIT_1(sc, offset, val) \
  CSR_WRITE_1(sc, offset, CSR_READ_1(sc, offset) & ~(val))

#define EE_SET(x) \
  CSR_WRITE_1(sc, RLW_9346CR, CSR_READ_1(sc, RLW_9346CR) | x)

#define EE_CLR(x) \
  CSR_WRITE_1(sc, RLW_9346CR, CSR_READ_1(sc, RLW_9346CR) & ~x)

static int  rlw_attach(device_t);
static int  rlw_detach(device_t);
static void rlw_reset(struct rlw_softc *);
static void rlw_eeprom_putbyte(struct rlw_softc *, int);
static void rlw_eeprom_getword(struct rlw_softc *, int, u_int16_t *);
static void rlw_read_eeprom(struct rlw_softc *, caddr_t, int, int);

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
  int error = 0;
  int rid = 0;
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

  for (i = 0; i < 128; i++) {
    rlw_read_eeprom(sc, (caddr_t)&val, i, 1);
    device_printf(dev, "%x: %x\n", i, val);
  }

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
rlw_eeprom_getword(struct rlw_softc *sc, int addr, u_int16_t *dest)
{
  int i;
  u_int16_t word = 0;

  rlw_eeprom_putbyte(sc, addr);

  for (i = 0x8000; i; i >>= 1) {
    EE_SET(RLW_9346CR_EESK);
    DELAY(100);
    if (CSR_READ_1(sc, RLW_9346CR) & RLW_9346CR_EEDO)
      word |= i;
    EE_CLR(RLW_9346CR_EESK);
    DELAY(100);
    }

  *dest = word;
}

static void
rlw_eeprom_putbyte(struct rlw_softc *sc, int addr)
{
  int d, i;

  d = addr | (RLW_93C46_READ << sc->rlw_eewidth);

  for (i = 1 << (sc->rlw_eewidth + 3); i; i >>= 1) {
    if (d & i) {
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
}

static void
rlw_read_eeprom(struct rlw_softc *sc, caddr_t dest, int off, int cnt)
{
  int i;
  u_int16_t word = 0, *ptr;

  CSR_SETBIT_1(sc, RLW_9346CR, RLW_9346CR_EEM_PROGRAM);
  DELAY(100);

  for (i = 0; i < cnt; i++) {
    CSR_SETBIT_1(sc, RLW_9346CR, RLW_9346CR_EECS);
    rlw_eeprom_getword(sc, off + i, &word);
    CSR_CLRBIT_1(sc, RLW_9346CR, RLW_9346CR_EECS);
    ptr = (u_int16_t *)(dest + (i * 2));
    *ptr = word;
  }

  CSR_CLRBIT_1(sc, RLW_9346CR, RLW_9346CR_EEM_PROGRAM);
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
