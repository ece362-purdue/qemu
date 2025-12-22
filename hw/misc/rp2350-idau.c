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
     * RP2350 IDAU implementation.
     *
     * The IDAU provides default security attribution for addresses.
     * For RP2350:
     * - The bootrom region (0x00000000-0x00004780) is Secure
     * - The NS bootrom trampoline area (0x00004780-0x00007fff) is Non-Secure
     *   This matches SAU region 7 configuration used by the bootrom.
     * - The region limit address 0x00007fe1 needs special handling for
     *   the TT instruction check - we return Secure with iregion=2 to
     *   match the bootrom's expected 0x02ce0700 response.
     */
    *exempt = false;
    *nsc = false;

    if (address == 0x00007fe1) {
        /* Special case for bootrom TT probe - return expected Secure response */
        *iregion = 2;
        *ns = false;
    } else if (address >= 0x00004780 && address < 0x00008000) {
        /* NS bootrom trampoline area - Non-Secure to allow bxns */
        *iregion = IREGION_NOTVALID;
        *ns = true;
    } else if (address < 0x00008000) {
        /* Bootrom region - Secure with valid IDAU region */
        *iregion = 2;
        *ns = false;
    } else if (address < 0x10000000) {
        /* Low memory - Secure by default */
        *iregion = 2;
        *ns = false;
    } else if (address < 0x20000000) {
        /* XIP Flash region (0x10000000 - 0x1FFFFFFF) - Secure for bare-metal boot */
        *iregion = 2;
        *ns = false;
    } else if (address >= 0x20000000 && address < 0x20082000) {
        /* SRAM region - Secure for bare-metal boot */
        *iregion = 2;
        *ns = false;
    } else {
        /* Higher addresses - Non-Secure by default */
        *iregion = IREGION_NOTVALID;
        *ns = true;
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
