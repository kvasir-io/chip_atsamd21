#!/usr/bin/env python3
"""svd/ATSAMD21G18A.svd out of this package's chip.svd and Microchip's device pack.

chip.svd (the ATSAMD21G17L) is not the file Microchip ships: its register, field and enum names
are the ones chip_atsam_common's drivers are written against (EIC::CTRLA, DMAC's trigger names,
EVSYS's generator enum, ...), shared with the SAM C21. Microchip's own ATSAMD21G18A.svd spells
those differently, so the drivers do not build against it. The two parts are the same silicon
family and differ in which peripherals they have - so the G18A file is chip.svd with the
L-only peripherals taken out and the ones only the A parts have, USB and I2S, put in from the
device pack unchanged.

    scripts/make_g18a_svd.py <Microchip.SAMD21_DFP>/samd21a/svd/ATSAMD21G18A.svd

The pack: https://packs.download.microchip.com/ , Microchip.SAMD21_DFP.<version>.atpack (a zip),
Apache-2.0 like chip.svd. svd/README.md records which version the checked-in file came from.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent

# SAM D21 datasheet DS40001882, "Configuration Summary": what the L parts have and the A parts
# do not, and the other way round.
L_ONLY = ["AC1", "TC6", "TC7", "TCC3"]
A_ONLY = ["I2S", "USB"]


def peripheral_block(text: str, name: str) -> re.Match:
    """The <peripheral ...>...</peripheral> element whose first <name> is `name`, with the
    whitespace before it. Peripherals do not nest, so a non-greedy match is exact."""
    for m in re.finditer(r"[ \t]*<peripheral(?: [^>]*)?>.*?</peripheral>\n", text, re.S):
        first = re.search(r"<name>([^<]+)</name>", m.group(0))
        if first and first.group(1) == name:
            return m
    raise SystemExit(f"no peripheral {name}")


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    pack = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
    svd = (ROOT / "chip.svd").read_text(encoding="utf-8")

    for name in L_ONLY:
        m = peripheral_block(svd, name)
        svd = svd[: m.start()] + svd[m.end():]

    added = "".join(peripheral_block(pack, name).group(0) for name in A_ONLY)
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

    out = ROOT / "svd" / "ATSAMD21G18A.svd"
    out.write_text(svd, encoding="utf-8")
    print(f"{out}: {len(L_ONLY)} peripherals out, {len(A_ONLY)} in")


if __name__ == "__main__":
    main()
