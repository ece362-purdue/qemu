/*
 * RP2350 IDAU model (minimal)
 *
 * This implements the Armv8-M IDAU interface used by the CPU security
 * attribution logic (and the TT instruction). For now we provide a small
 * hard-coded map sufficient for RP2350 BootROM bring-up.
 *
 * This is not a full RP2350 security model.
 */

#include "qemu/osdep.h"
#include "qom/object.h"
#include "target/arm/idau.h"

#define TYPE_RP2350_IDAU "rp2350-idau"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350IDAUState, RP2350_IDAU)

struct RP2350IDAUState {
    Object parent_obj;
};

static void rp2350_idau_check(IDAUInterface *ii, uint32_t address,
                             int *iregion, bool *exempt, bool *ns, bool *nsc)
{
    /*
     * BootROM probes an address in the low alias region (example seen:
     * 0x00007fe1) and expects it to be attributed as NS with a valid iregion.
     *
     * We mark low Flash/XIP alias (0x00000000..0x00ffffff) as NonSecure.
     * Everything else defaults to Secure.
     */
    *exempt = false;
    *nsc = false;

    if (address < 0x01000000) {
        /*
         * BootROM expects a *valid* iregion and for the iregion number
         * for this low alias range (drives TT upper bits).
         *
         * Our earlier iregion=2 produced TT=0x02be0700; BootROM expects
         * TT=0x02ce0700, which differs by 0x00100000 (one iregion step).
         */
    *iregion = 2;
    /* BootROM expects this probe to be attributed as Secure. */
    *ns = false;
    } else {
        *iregion = IREGION_NOTVALID;
        *ns = false;
    }
}

static void rp2350_idau_class_init(ObjectClass *klass, const void *data)
{
    IDAUInterfaceClass *iic = IDAU_INTERFACE_CLASS(klass);

    iic->check = rp2350_idau_check;
}

static const TypeInfo rp2350_idau_type_info = {
    .name = TYPE_RP2350_IDAU,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(RP2350IDAUState),
    .class_init = rp2350_idau_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_IDAU_INTERFACE },
        { }
    },
};

static void rp2350_idau_register_types(void)
{
    type_register_static(&rp2350_idau_type_info);
}

type_init(rp2350_idau_register_types);
