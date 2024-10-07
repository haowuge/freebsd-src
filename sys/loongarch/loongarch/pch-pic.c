
/*
 *
 * LoongArch pch-pic driver 
 * Author xiaoqiang zhao <zxq_yx_007@163.com>
 * 2024-06-18 11:28:00
 *
 */


#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/types.h>
#include <machine/intr.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "pic_if.h"

#define EIO_INTC_MAX_IRQS	256

struct pch_pic_irqsrc {
	struct intr_irqsrc	isrc;
	u_int				irq;
	uint32_t			node;
	uint32_t			vec_count;
};

struct pch_pic_softc {
	device_t				dev;
	device_t				parent;
	struct resource			*intc_res;
	struct pch_pic_irqsrc	isrcs[EIO_INTC_MAX_IRQS];
	struct intr_map_data_fdt *parent_map_data;
};

static int
pch_pic_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev,"loongson,pch-pic-1.0"))
		return (ENXIO);

	device_set_desc(dev, "LoongArch pch-pic");

	return (BUS_PROBE_DEFAULT);
}

static int
pch_pic_attach(device_t dev)
{
	struct pch_pic_softc *sc;
	phandle_t node, xref, intr_parent;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	if ((intr_parent = ofw_bus_find_iparent(node)) == 0) {
		device_printf(dev,
				"Cannot find our parent interrupt controller\n");
		return (ENXIO);
	}

	/*
	if ((sc->parent = OF_device_from_xref(intr_parent)) == NULL) {
		device_printf(dev,
				"Cannot find parent interrupt controller device\n");
		return (ENXIO);	
	}
	*/

	/* Register ourself as a interrupt controller */
	xref = OF_xref_from_node(node);
	if (intr_pic_register(dev, xref) == NULL) {
		device_printf(dev, "Cannot register GICP\n");
		return (ENXIO);
	}

	/*
	sc->parent_map_data = (struct intr_map_data_fdt *)intr_alloc_map_data(
		INTR_MAP_DATA_FDT, sizeof(struct intr_map_data_fdt)
	*/

	/* Register ourself to device can find us */
	OF_device_register_xref(xref, dev);
	
	return (0);
}

static void
pch_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{

}

static void
pch_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{

}

static int
pch_pic_map_intr(device_t dev, struct intr_map_data *data,
	struct intr_irqsrc **isrcp)
{
	return (0);
}

static void
pch_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{

}

static void
pch_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{

}

static void
pch_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
}


static int
pch_pic_setup_intr(device_t dev, struct intr_irqsrc *isrc,
	struct resource *res, struct intr_map_data *data)
{
	return (0);
}

static int
pch_pic_bind_intr(device_t dev, struct intr_irqsrc *isrc)
{
	return (0);
}

static device_method_t pch_pic_methods[] = {
	DEVMETHOD(device_probe,		pch_pic_probe),
	DEVMETHOD(device_attach,	pch_pic_attach),

	DEVMETHOD(pic_disable_intr,	pch_pic_disable_intr),
	DEVMETHOD(pic_enable_intr,	pch_pic_enable_intr),
	DEVMETHOD(pic_map_intr,		pch_pic_map_intr),
	DEVMETHOD(pic_pre_ithread,	pch_pic_pre_ithread),
	DEVMETHOD(pic_post_ithread,	pch_pic_post_ithread),
	DEVMETHOD(pic_post_filter,	pch_pic_post_filter),
	DEVMETHOD(pic_setup_intr,	pch_pic_setup_intr),
	DEVMETHOD(pic_bind_intr,	pch_pic_bind_intr),

	DEVMETHOD_END
};

static driver_t	pch_pic_driver = {
	"pch-pic",
	pch_pic_methods,
    sizeof(struct pch_pic_softc),
};

EARLY_DRIVER_MODULE(pch_pic, simplebus, pch_pic_driver, 0, 0,
	BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);

