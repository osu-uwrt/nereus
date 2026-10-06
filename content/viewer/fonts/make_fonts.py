#!/usr/bin/env python3
"""Builds the viewer's Barlow TF fonts: Barlow (SIL OFL 1.1, The Barlow Project Authors) with tabular figures as
the default digits, so columns of numbers line up (the viewer's text renderer cannot select the `tnum` feature).

    python3 make_fonts.py            # downloads Barlow from google/fonts and writes BarlowTF-*.ttf here

The family is renamed "Barlow TF" so the modified fonts are never mistaken for the originals."""

import io
import re
import pathlib
import urllib.request

from fontTools.ttLib import TTFont

HERE = pathlib.Path(__file__).resolve().parent

# Raw-file URL template into the google/fonts repository's OFL directory.
SOURCE = "https://github.com/google/fonts/raw/main/ofl/{}"
FACES = {  # output name: upstream path
    "BarlowTF-Regular.ttf": "barlow/Barlow-Regular.ttf",
    "BarlowTF-Medium.ttf": "barlow/Barlow-Medium.ttf",
    "BarlowTF-SemiBold.ttf": "barlow/Barlow-SemiBold.ttf",
    "BarlowTF-SemiCondensedBold.ttf": "barlowsemicondensed/BarlowSemiCondensed-Bold.ttf",
}


def tabular(font: TTFont) -> None:
    """Points the digit code points at the glyphs the font's `tnum` feature substitutes."""
    gsub = font["GSUB"].table

    # Collect the proportional -> tabular glyph mapping from every lookup of the `tnum` feature.
    swap = {}
    for record in gsub.FeatureList.FeatureRecord:
        if record.FeatureTag != "tnum":
            continue
        for index in record.Feature.LookupListIndex:
            for sub in gsub.LookupList.Lookup[index].SubTable:
                swap.update(getattr(sub, "mapping", {}) or {})

    # Remap only digit characters, in every Unicode cmap subtable.
    for table in font["cmap"].tables:
        if table.isUnicode():
            for code, glyph in list(table.cmap.items()):
                if glyph in swap and chr(code).isdigit():
                    table.cmap[code] = swap[glyph]


def rename(font: TTFont) -> None:
    """Barlow -> Barlow TF in every name (family, full and PostScript names keep their width and weight)."""
    # Name IDs: 1 family, 3 unique ID, 4 full name, 6 PostScript, 16/17 typographic family/subfamily,
    # 18 compatible full name. PostScript-style names get no space ("BarlowTF-...").
    for record in font["name"].names:
        if record.nameID in (1, 3, 4, 6, 16, 17, 18):
            text = record.toUnicode()
            text = re.sub(r"\bBarlow(?! TF)(?=\b| |SemiCondensed|-)", "Barlow TF", text)
            record.string = text.replace("Barlow TFSemiCondensed", "BarlowTFSemiCondensed").replace(
                "Barlow TF-", "BarlowTF-"
            )


def main() -> None:
    """Downloads each face, applies the tabular digits and the rename, and saves it with the licence."""
    for name, path in FACES.items():
        data = urllib.request.urlopen(SOURCE.format(path), timeout=60).read()
        font = TTFont(io.BytesIO(data))
        tabular(font)
        rename(font)
        font.save(HERE / name)
        print("wrote", name)

    # The OFL requires the licence to travel with the modified fonts.
    license = urllib.request.urlopen(SOURCE.format("barlow/OFL.txt"), timeout=60).read()
    (HERE / "OFL.txt").write_bytes(license)


if __name__ == "__main__":
    main()
