"""The Nereus logo (trident and sonar): the mark as SVG, PNG icons (small sizes use heavier strokes), and the bare
mark's two parts as white masks for the viewer's title bar.

Regenerate: uv run --with cairosvg python content/viewer/icons/make_icons.py docs/images content/viewer/icons
"""

import math
import sys
from pathlib import Path

import cairosvg

DOCS, ICONS = Path(sys.argv[1]), Path(sys.argv[2])
CYAN, DEEP, FOAM = "#52dbd1", "#061118", "#e6fbf8"


def arc(cx, cy, r, a0, a1):
    x0, y0 = cx + r * math.cos(math.radians(a0)), cy - r * math.sin(math.radians(a0))
    x1, y1 = cx + r * math.cos(math.radians(a1)), cy - r * math.sin(math.radians(a1))
    return f"M {x0:.1f} {y0:.1f} A {r} {r} 0 0 0 {x1:.1f} {y1:.1f}"


def mark(small=False):
    """The trident rising out of sonar pings on the deep-water tile. small: fewer, heavier lines for 16-48 px."""
    rings = ((92, 0.95), (150, 0.6)) if small else ((92, 0.95), (150, 0.6), (208, 0.3))
    ping_width, shaft = (22, 30) if small else (14, 22)
    pings = "\n".join(
        f'  <path d="{arc(256, 404, r, 28, 152)}" fill="none" stroke="{CYAN}" '
        f'stroke-width="{ping_width}" stroke-linecap="round" opacity="{o}"/>'
        for r, o in rings
    )
    border = (
        ""
        if small
        else '  <rect x="16" y="16" width="480" height="480" rx="112" fill="none" stroke="#2a5b66" stroke-width="3"/>\n'
    )
    return f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" width="512" height="512">
  <title>Nereus</title>
  <defs>
    <linearGradient id="bg" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0" stop-color="#11394a"/><stop offset="1" stop-color="{DEEP}"/>
    </linearGradient>
  </defs>
  <rect x="16" y="16" width="480" height="480" rx="112" fill="url(#bg)"/>
{border}{pings}
  <g fill="none" stroke="{FOAM}" stroke-width="{shaft}" stroke-linecap="round" stroke-linejoin="round">
    <path d="M256 404 L256 128"/>
    <path d="M178 138 L178 186 Q178 226 218 226 L294 226 Q334 226 334 186 L334 138"/>
  </g>
  <g fill="{FOAM}">
    <path d="M256 70 L232 132 L280 132 Z"/>
    <path d="M178 92 L160 146 L196 146 Z"/>
    <path d="M334 92 L316 146 L352 146 Z"/>
  </g>
</svg>
"""


def glyph(part):
    """The bare mark (no tile) in white, for the title bar to tint: part "trident" or "sonar" (the pings)."""
    if part == "sonar":
        body = "\n".join(
            f'  <path d="{arc(256, 404, r, 28, 152)}" fill="none" stroke="#fff" stroke-width="22" '
            f'stroke-linecap="round" opacity="{o}"/>'
            for r, o in ((92, 0.95), (150, 0.6), (208, 0.3))
        )
    else:
        body = """  <g fill="none" stroke="#fff" stroke-width="26" stroke-linecap="round" stroke-linejoin="round">
    <path d="M256 404 L256 128"/>
    <path d="M178 138 L178 186 Q178 226 218 226 L294 226 Q334 226 334 186 L334 138"/>
  </g>
  <g fill="#fff">
    <path d="M256 70 L232 132 L280 132 Z"/>
    <path d="M178 92 L160 146 L196 146 Z"/>
    <path d="M334 92 L316 146 L352 146 Z"/>
  </g>"""
    # the mark's own extent (x 40..472, y 60..420), so it fills the title bar's square
    return f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="40 52 432 432" width="432" height="432">
{body}
</svg>
"""


DOCS.mkdir(parents=True, exist_ok=True)
ICONS.mkdir(parents=True, exist_ok=True)
(DOCS / "logo.svg").write_text(mark())
(DOCS / "logo-small.svg").write_text(mark(small=True))
for size in (16, 24, 32, 48, 64, 128, 256, 512):
    svg = mark(small=size <= 48)
    (ICONS / f"nereus-{size}.png").write_bytes(
        cairosvg.svg2png(bytestring=svg.encode(), output_width=size, output_height=size)
    )
for part in ("trident", "sonar"):  # white masks the viewer tints (its title bar)
    (ICONS / f"nereus-{part}-128.png").write_bytes(
        cairosvg.svg2png(bytestring=glyph(part).encode(), output_width=128, output_height=128)
    )
print(sorted(p.name for p in ICONS.iterdir()))
