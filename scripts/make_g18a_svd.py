#!/usr/bin/env python3
"""The ATSAMD21G18A's SVD out of this package's chip.svd and the USB and I2S blocks of Microchip's device pack.

chip.svd (the ATSAMD21G17L) is not the file Microchip ships: its register, field and enum names
are the ones chip_atsam_common's drivers are written against (EIC::CTRLA, DMAC's trigger names,
EVSYS's generator enum, ...), shared with the SAM C21. Microchip's own ATSAMD21G18A.svd spells
those differently, so the drivers do not build against it. The two parts are the same silicon
family and differ in which peripherals they have - so the G18A file is chip.svd with the
L-only peripherals taken out and the ones only the A parts have, USB and I2S, put in from the
device pack unchanged, plus PTC's interrupt line 26 (A_ONLY_INTERRUPTS) and PM's clock mask
fields of the A part (PM_L_ONLY_FIELDS, PM_A_ONLY_FIELDS: the gate table's rows name them), and GCLK's CLKCTRL.ID
values of the A part (ID_L_ONLY, ID_A_ONLY: the generated `CLKCTRL::IDVal` is `GCLK::Peripheral`). Those two blocks are checked in as svd/samd21a_pack_usb_i2s.svd (a
<peripherals> wrapper around them, byte for byte the pack's), so making the file needs no pack.

    scripts/make_g18a_svd.py --out <file>                write the G18A SVD (the SDK's kvasir_chip_variant()
                                                         runs this when a build tree configures)
    scripts/make_g18a_svd.py --refresh-fragment <pack>   take USB and I2S anew from
                                                         <Microchip.SAMD21_DFP>/samd21a/svd/ATSAMD21G18A.svd

The pack: https://packs.download.microchip.com/ , Microchip.SAMD21_DFP.<version>.atpack (a zip),
Apache-2.0 like chip.svd. svd/README.md records which version the fragment came from.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
FRAGMENT = ROOT / "svd" / "samd21a_pack_usb_i2s.svd"
FRAGMENT_HEAD = (
    '<?xml version="1.0" encoding="utf-8"?>\n'
    "<!-- The USB and I2S peripherals of Microchip's ATSAMD21G18A.svd (Microchip.SAMD21_DFP, Apache-2.0), copied\n"
    "     byte for byte by scripts/make_g18a_svd.py - edit nothing here; a new pack goes in with\n"
    "     the script's refresh-fragment mode. -->\n"
    "<device>\n  <peripherals>\n")
FRAGMENT_TAIL = "  </peripherals>\n</device>\n"

# SAM D21 datasheet DS40001882, "Configuration Summary": what the L parts have and the A parts
# do not, and the other way round.
L_ONLY = ["AC1", "TC6", "TC7", "TCC3"]
A_ONLY = ["I2S", "USB"]
# Interrupt lines only the A parts have, whose peripheral neither chip.svd nor the fragment describes: PTC (the
# Peripheral Touch Controller; Table 2-1 "PTC" column: the G18A has 120/10 channels, the G17L none) on line 26
# (DS40001882L Table 11-3, md l.1878), at 0x42004C00 (the memory map, md l.1470). An interrupt-only peripheral.
A_ONLY_INTERRUPTS = [
    ("PTC", 0x42004C00, 26, "DS40001882L Table 11-3 (md l.1878), base from the memory map (md l.1470)")]


def interrupt_only_peripheral(name: str, base: int, line: int, source: str) -> str:
    return (f"    <peripheral>\n"
            f"      <!-- Kvasir: A part only, added by scripts/make_g18a_svd.py: {source} -->\n"
            f"      <name>{name}</name>\n"
            f"      <baseAddress>0x{base:08X}</baseAddress>\n"
            f"      <interrupt>\n"
            f"        <name>{name}</name>\n"
            f"        <value>{line}</value>\n"
            f"      </interrupt>\n"
            f"    </peripheral>\n")


def peripheral_block(text: str, name: str) -> re.Match:
    """The <peripheral ...>...</peripheral> element whose first <name> is `name`, with the
    whitespace before it. Peripherals do not nest, so a non-greedy match is exact."""
    for m in re.finditer(r"[ \t]*<peripheral(?: [^>]*)?>.*?</peripheral>\n", text, re.S):
        first = re.search(r"<name>([^<]+)</name>", m.group(0))
        if first and first.group(1) == name:
            return m
    raise SystemExit(f"no peripheral {name}")


# PM's clock mask fields of the parts (DS40001882 16.8.7 AHBMASK, 16.8.9 APBBMASK, 16.8.10 APBCMASK): chip.svd's
# PM is the L part's, so the G18A loses the masks of the L-only blocks and gains USB's two and I2S's.
PM_L_ONLY_FIELDS = {"APBCMASK": ["TC6_", "TC7_", "AC1_", "TCC3_"]}
PM_A_ONLY_FIELDS = [  # (register, the field after which it goes, name, bit, description, source)
    ("AHBMASK", "DMAC_", "USB_", 6, "USB AHB Clock Mask",
     "DS40001882 16.8.7 AHBMASK bit 6"),
    ("APBBMASK", "DMAC_", "USB_", 5, "USB APB Clock Enable",
     "DS40001882 16.8.9 APBBMASK bit 5"),
    ("APBCMASK", "PTC_", "I2S_", 20, "I2S APB Clock Enable",
     "DS40001882 16.8.10 APBCMASK bit 20"),
]


# GCLK CLKCTRL.ID of the parts (DS40001882L 15.8.3, the ID table md l.5191-5240: 0x06 USB, 0x22 PTC, 0x23/0x24
# I2S_0/1, 0x25 TCC3): chip.svd's IDSelect is the L part's, so the G18A loses the channels of the L-only blocks
# (TC6/TC7, TCC3) and gains USB's, PTC's and the two I2S clock units'.
ID_L_ONLY = ["TC6_TC7", "TCC3"]
ID_A_ONLY = [("USB", 0x06), ("PTC", 0x22), ("I2S_0", 0x23), ("I2S_1", 0x24)]


def field_block(reg: str, name: str) -> re.Match:
    """The <field>...</field> element named `name` in a register's text, with the whitespace before it."""
    for m in re.finditer(r"[ \t]*<field>.*?</field>\n", reg, re.S):
        if re.search(rf"<name>{re.escape(name)}</name>", m.group(0)):
            return m
    raise SystemExit(f"no field {name}")


def pm_masks(svd: str) -> str:
    """chip.svd's PM with the G18A's clock mask fields (PM_L_ONLY_FIELDS out, PM_A_ONLY_FIELDS in)."""
    pm = peripheral_block(svd, "PM")
    block = pm.group(0)
    for reg_name in {r for r, *_ in PM_A_ONLY_FIELDS} | set(PM_L_ONLY_FIELDS):
        regs = [m for m in re.finditer(r"[ \t]*<register>.*?</register>\n", block, re.S)
                if re.search(rf"<name>{reg_name}</name>", m.group(0))]
        assert len(regs) == 1, reg_name
        reg = regs[0].group(0)
        for name in PM_L_ONLY_FIELDS.get(reg_name, []):
            m = field_block(reg, name)
            reg = reg[: m.start()] + reg[m.end():]
        for r, after, name, bit, desc, source in PM_A_ONLY_FIELDS:
            if r != reg_name:
                continue
            m = field_block(reg, after)
            indent = re.match(r"[ \t]*", m.group(0)).group(0)
            reg = (reg[: m.end()]
                   + f"{indent}<field>\n"
                   + f"{indent}  <!-- Kvasir: A part only, added by scripts/make_g18a_svd.py: {source} -->\n"
                   + f"{indent}  <name>{name}</name>\n"
                   + f"{indent}  <description>{desc}</description>\n"
                   + f"{indent}  <bitOffset>{bit}</bitOffset>\n"
                   + f"{indent}  <bitWidth>1</bitWidth>\n"
                   + f"{indent}</field>\n"
                   + reg[m.end():])
        block = block[: regs[0].start()] + reg + block[regs[0].end():]
    return svd[: pm.start()] + block + svd[pm.end():]


def gclk_ids(svd: str) -> str:
    """chip.svd's GCLK with the G18A's CLKCTRL.ID values (ID_L_ONLY out, ID_A_ONLY in, in value order)."""
    gclk = peripheral_block(svd, "GCLK")
    block = gclk.group(0)
    enum = re.search(
        r"<name>IDSelect</name>\n(.*?)([ \t]*)</enumeratedValues>", block, re.S)
    assert enum, "no IDSelect in GCLK"
    values = re.findall(
        r"[ \t]*<enumeratedValue>.*?</enumeratedValue>\n", enum.group(1), re.S)
    assert "".join(values) == enum.group(
        1), "IDSelect holds more than enumeratedValues"
    indent = re.match(r"[ \t]*", values[0]).group(0)

    def value_of(v: str) -> int:
        return int(re.search(r"<value>([^<]+)</value>", v).group(1), 0)

    def name_of(v: str) -> str:
        return re.search(r"<name>([^<]+)</name>", v).group(1)

    for name in ID_L_ONLY:
        assert any(name_of(v) == name for v in values), name
    kept = [v for v in values if name_of(v) not in ID_L_ONLY]
    for name, value in ID_A_ONLY:
        assert all(value_of(v) != value for v in kept), name
        kept.append(f"{indent}<enumeratedValue>\n"
                    f"{indent}  <!-- Kvasir: A part only, added by scripts/make_g18a_svd.py: DS40001882L 15.8.3 -->\n"
                    f"{indent}  <name>{name}</name>\n"
                    f"{indent}  <description>{name}</description>\n"
                    f"{indent}  <value>0x{value:02x}</value>\n"
                    f"{indent}</enumeratedValue>\n")
    kept.sort(key=value_of)
    block = block[: enum.start(1)] + "".join(kept) + block[enum.end(1):]
    return svd[: gclk.start()] + block + svd[gclk.end():]


def make() -> str:
    """The G18A SVD: chip.svd without the L-only peripherals, with the fragment's A-only ones."""
    pack = FRAGMENT.read_text(encoding="utf-8")
    svd = (ROOT / "chip.svd").read_text(encoding="utf-8")

    for name in L_ONLY:
        m = peripheral_block(svd, name)
        svd = svd[: m.start()] + svd[m.end():]

    svd = pm_masks(svd)
    svd = gclk_ids(svd)

    added = "".join(peripheral_block(pack, name).group(0) for name in A_ONLY)
    added += "".join(interrupt_only_peripheral(*irq)
                     for irq in A_ONLY_INTERRUPTS)
    end = svd.rindex("  </peripherals>")
    svd = svd[:end] + added + svd[end:]

    svd, n = re.subn(r"<name>ATSAMD21G17L</name>",
                     "<name>ATSAMD21G18A</name>", svd, count=1)
    assert n == 1
    svd, n = re.subn(
        r"<description>[^<]*</description>",
        "<description>Microchip ATSAMD21G18A: Cortex-M0+ microcontroller with 256KB flash, 32KB "
        "SRAM, 48-pin package. Built by scripts/make_g18a_svd.py from chip.svd (ATSAMD21G17L) "
        "and the USB and I2S peripherals of Microchip's SAMD21 device pack.</description>",
        svd,
        count=1,
    )
    assert n == 1
    return svd


def main() -> None:
    args = sys.argv[1:]
    if len(args) != 2 or args[0] not in ("--out", "--refresh-fragment"):
        raise SystemExit(__doc__)
    if args[0] == "--refresh-fragment":
        pack = pathlib.Path(args[1]).read_text(encoding="utf-8")
        blocks = "".join(peripheral_block(pack, name).group(0)
                         for name in A_ONLY)
        FRAGMENT.write_text(FRAGMENT_HEAD + blocks +
                            FRAGMENT_TAIL, encoding="utf-8")
        print(f"{FRAGMENT}: {', '.join(A_ONLY)} from {args[1]}")
        return
    out = pathlib.Path(args[1])
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(make(), encoding="utf-8")
    print(f"{out}: {len(L_ONLY)} peripherals out, {len(A_ONLY)} in, {len(A_ONLY_INTERRUPTS)} interrupt line(s)")


if __name__ == "__main__":
    main()
