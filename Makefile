KMOD=	rlw
SRCS=	rlw.c rtl8225.c
SRCS+=	device_if.h bus_if.h pci_if.h

.include <bsd.kmod.mk>
