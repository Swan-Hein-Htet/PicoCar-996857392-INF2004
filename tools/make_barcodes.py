#!/usr/bin/env python3
"""Printable Code 39 course barcodes, *A* to *Z*, one symbol per A4 page.

    python3 tools/make_barcodes.py <out_dir> [narrow_mm ...]

Writes barcodes_A-Z_<narrow>mm.pdf for each narrow-bar width (default 3.0,
4.4 and 6.0 mm; wide = 3 x narrow, symbol length = 47 x narrow, so 3.0 mm
is the 141 mm size of "Barcode & Line Specification.pdf").  Print at 100%
("actual size"), not "fit to page".  Standard library only.
"""

import os
import sys

# Standard Code 39: nine elements, bar first, most significant bit first,
# 1 = wide.  Written out independently of app_program/subsystem3/barcode.c
# so that decoding these pages with the firmware proves the two agree.
PATTERNS = {
    "A": 0x109, "B": 0x049, "C": 0x148, "D": 0x019, "E": 0x118, "F": 0x058,
    "G": 0x00D, "H": 0x10C, "I": 0x04C, "J": 0x01C, "K": 0x103, "L": 0x043,
    "M": 0x142, "N": 0x013, "O": 0x112, "P": 0x052, "Q": 0x007, "R": 0x106,
    "S": 0x046, "T": 0x016, "U": 0x181, "V": 0x0C1, "W": 0x1C0, "X": 0x091,
    "Y": 0x190, "Z": 0x0D0, "*": 0x094,
}
PAGE_W_PT = 841.89      # A4 landscape
PAGE_H_PT = 595.28
MM = 72.0 / 25.4
BAR_HEIGHT_MM = 100.0


def element_widths(letter):
    """The 29 elements of *letter* in narrow units (bar first)."""
    out = []
    for pos, char in enumerate("*" + letter + "*"):
        if pos:
            out.append(1)       # narrow gap between characters
        out += [3 if PATTERNS[char] >> (8 - bit) & 1 else 1
                for bit in range(9)]
    return out


def page_stream(letter, narrow_mm):
    widths = element_widths(letter)
    total = sum(widths) * narrow_mm * MM
    x = (PAGE_W_PT - total) / 2.0
    y = (PAGE_H_PT - BAR_HEIGHT_MM * MM) / 2.0
    cmds = ["0 g"]
    for idx, width in enumerate(widths):
        wide = width * narrow_mm * MM
        if idx % 2 == 0:
            cmds.append("%.3f %.3f %.3f %.3f re f" %
                        (x, y, wide, BAR_HEIGHT_MM * MM))
        x += wide
    cmds.append("BT /F1 16 Tf 36 36 Td (%s   narrow %.1f mm, length %.0f mm"
                "   print at 100%%) Tj ET" %
                (letter, narrow_mm, sum(widths) * narrow_mm))
    return "\n".join(cmds).encode()


def write_pdf(path, narrow_mm):
    letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    objs = {1: b"<< /Type /Catalog /Pages 2 0 R >>",
            3: b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>"}
    kids = []
    for idx, letter in enumerate(letters):
        page, content = 4 + 2 * idx, 5 + 2 * idx
        kids.append("%d 0 R" % page)
        objs[page] = ("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.2f %.2f]"
                      " /Resources << /Font << /F1 3 0 R >> >>"
                      " /Contents %d 0 R >>" %
                      (PAGE_W_PT, PAGE_H_PT, content)).encode()
        stream = page_stream(letter, narrow_mm)
        objs[content] = (b"<< /Length %d >>\nstream\n" % len(stream) +
                         stream + b"\nendstream")
    objs[2] = ("<< /Type /Pages /Kids [%s] /Count %d >>" %
               (" ".join(kids), len(letters))).encode()

    out = bytearray(b"%PDF-1.4\n")
    offsets = {}
    for num in sorted(objs):
        offsets[num] = len(out)
        out += b"%d 0 obj\n" % num + objs[num] + b"\nendobj\n"
    xref = len(out)
    out += b"xref\n0 %d\n0000000000 65535 f \n" % (len(objs) + 1)
    for num in sorted(objs):
        out += b"%010d 00000 n \n" % offsets[num]
    out += (b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" %
            (len(objs) + 1, xref))
    with open(path, "wb") as handle:
        handle.write(out)


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    os.makedirs(argv[1], exist_ok=True)
    for narrow in [float(v) for v in argv[2:]] or [3.0, 4.4, 6.0]:
        path = os.path.join(argv[1], "barcodes_A-Z_%.1fmm.pdf" % narrow)
        write_pdf(path, narrow)
        print("wrote", path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
