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

#include "rlw.h"
#include "rtl8225.h"

static uint16_t rtl8225_read(struct rlw_softc *, uint8_t);
static void rtl8225_write(struct rlw_softc *, uint8_t, uint16_t);

int
rtl8225_detect(struct rlw_softc *sc)
{
  uint16_t reg8, reg9;

  CSR_WRITE_2(sc, RLW_RF_OUT, 0x0480);
  CSR_WRITE_2(sc, RLW_RF_SEL, 0x0488);
  CSR_WRITE_2(sc, RLW_RF_ENB, 0x1fff);
  CSR_READ_1(sc, RLW_9346CR);
  pause("RTL8225 reset", 100);

  rtl8225_write(sc, 0, 0x1b7);
  reg8 = rtl8225_read(sc, 8);
  reg9 = rtl8225_read(sc, 9);
  rtl8225_write(sc, 0, 0x0b7);

  if (reg8 == 0x588 && reg9 == 0x700)
    return (0);
  else
    return (1);
}

static uint16_t
rtl8225_read(struct rlw_softc *sc, uint8_t addr)
{
  uint16_t out, reg80, reg82, reg84;
  int i;

  reg80 = CSR_READ_2(sc, RLW_RF_OUT) & ~0x000f;
  reg82 = CSR_READ_2(sc, RLW_RF_ENB);
  reg84 = CSR_READ_2(sc, RLW_RF_SEL) | 0x400;


  CSR_WRITE_2(sc, RLW_RF_ENB, reg82 | 0x000f);
  CSR_WRITE_2(sc, RLW_RF_SEL, reg84 | 0x000f);

  CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_EN);
  CSR_READ_1(sc, RLW_9346CR);
  DELAY(4);
  CSR_WRITE_2(sc, RLW_RF_OUT, reg80);
  CSR_READ_1(sc, RLW_9346CR);
  DELAY(5);

  for (i = 4; i >= 0; i--) {
    uint16_t reg = reg80 | ((addr >> i) & 1);
    if (!(i & 1)) {
      CSR_WRITE_2(sc, RLW_RF_OUT, reg);
      CSR_READ_1(sc, RLW_9346CR);
      DELAY(1);
    }

    CSR_WRITE_2(sc, RLW_RF_OUT, reg | RLW_BB_HOST_BANG_CLK);
    CSR_READ_1(sc, RLW_9346CR);
    DELAY(2);
    CSR_WRITE_2(sc, RLW_RF_OUT, reg | RLW_BB_HOST_BANG_CLK);
    CSR_READ_1(sc, RLW_9346CR);
    DELAY(2);

    if (i & 1) {
      CSR_WRITE_2(sc, RLW_RF_OUT, reg);
      CSR_READ_1(sc, RLW_9346CR);
      DELAY(1);
    }
  }

  CSR_WRITE_2(sc, RLW_RF_ENB, 0x000e);
  CSR_WRITE_2(sc, RLW_RF_SEL, 0x040e);
  CSR_READ_1(sc, RLW_9346CR);
  CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_RW | RLW_BB_HOST_BANG_CLK);
  CSR_READ_1(sc, RLW_9346CR);
  DELAY(2);
  CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_RW);
  CSR_READ_1(sc, RLW_9346CR);
  DELAY(2);
  CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_RW);
  CSR_READ_1(sc, RLW_9346CR);
  DELAY(2);

  out = 0;
  for (i = 11; i >= 0; i--) {
    CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_RW);
    CSR_READ_1(sc, RLW_9346CR);
    DELAY(1);
    CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_RW | RLW_BB_HOST_BANG_CLK);
    CSR_READ_1(sc, RLW_9346CR);
    DELAY(2);
    CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_RW | RLW_BB_HOST_BANG_CLK);
    CSR_READ_1(sc, RLW_9346CR);
    DELAY(2);
    CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_RW | RLW_BB_HOST_BANG_CLK);
    CSR_READ_1(sc, RLW_9346CR);
    DELAY(2);

    if (CSR_READ_2(sc, RLW_RF_INP) & (1 << 1))
      out |= 1 << i;

    CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_RW);
    CSR_READ_1(sc, RLW_9346CR);
    DELAY(2);
  }

  CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_RW | RLW_BB_HOST_BANG_EN);
  CSR_READ_1(sc, RLW_9346CR);
  DELAY(2);

  CSR_WRITE_2(sc, RLW_RF_ENB, reg82);
  CSR_WRITE_2(sc, RLW_RF_SEL, reg84);
  CSR_WRITE_2(sc, RLW_RF_OUT, 0x03a0);

  return out;
}

static void
rtl8225_write(struct rlw_softc *sc, uint8_t addr, uint16_t data0)
{
  uint16_t reg80, reg82, reg84;
  uint32_t data;
  int i;

  data = ((uint32_t)data0 << 4) | (addr & 0xf);

  reg80 = CSR_READ_2(sc, RLW_RF_OUT) & 0xfff3;

  reg82 = CSR_READ_2(sc, RLW_RF_ENB);
  CSR_WRITE_2(sc, RLW_RF_ENB, reg82 | 0x7);
  reg84 = CSR_READ_2(sc, RLW_RF_SEL);
  CSR_WRITE_2(sc, RLW_RF_SEL, reg84 | 0x7 | 0x400);

  CSR_READ_1(sc, RLW_9346CR);
  DELAY(10);

  CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_EN);
  CSR_READ_1(sc, RLW_9346CR);
  DELAY(2);
  CSR_WRITE_2(sc, RLW_RF_OUT, reg80);
  CSR_READ_1(sc, RLW_9346CR);
  DELAY(10);

  for (i = 15; i >= 0; i--) {
    uint16_t reg = reg80;
    if ((1 << i) & data)
      reg |= 1;

    if (i & 1)
      CSR_WRITE_2(sc, RLW_RF_OUT, reg);

    CSR_WRITE_2(sc, RLW_RF_OUT, reg | RLW_BB_HOST_BANG_CLK);
    CSR_WRITE_2(sc, RLW_RF_OUT, reg | RLW_BB_HOST_BANG_CLK);

    if (!(i & 1))
      CSR_WRITE_2(sc, RLW_RF_OUT, reg);
  }

  CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_EN);
  CSR_READ_1(sc, RLW_9346CR);
  DELAY(10);
  CSR_WRITE_2(sc, RLW_RF_OUT, reg80 | RLW_BB_HOST_BANG_EN);
  CSR_WRITE_2(sc, RLW_RF_SEL, reg84 | 0x400);
  CSR_WRITE_2(sc, RLW_RF_ENB, 0x1fff);
} 
