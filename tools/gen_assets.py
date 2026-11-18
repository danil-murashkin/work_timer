"""Generate daisy RGB565 bitmap and Georgia serif glyph atlases for the firmware."""
from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
SRC = Path(
    r"C:\Users\danil\.cursor\projects\c-Users-danil-Desktop-Worspace-work-timer"
    r"\assets\image-198d9b66-a5ee-4cae-ac4d-d2c653b23282.png"
)
OUT_H = ROOT / "main" / "daisy_img.h"
OUT_FONT = ROOT / "main" / "font_serif.h"
PREVIEW = ROOT / "main" / "daisy_preview.png"
FONT_PREVIEW = ROOT / "main" / "font_serif_preview.png"

DAISY_SIZE = 165  # +5% from 157
CENTER_SCALE = 1.55
LCD_SWAP = True
GLYPHS = " 0123456789:ABCDEFGHIJKLMNOPQRSTUVWXYZ"

# Charis SIL (Charter-based): strong serifs, designed for low-res reproduction —
# better on tiny LCD bitmaps than Times/Georgia hairlines.
FONT_DIR = ROOT / "tools" / "fonts"
FONT_REGULAR = FONT_DIR / "CharisSIL-Regular.ttf"
FONT_BOLD = FONT_DIR / "CharisSIL-Bold.ttf"
FONT_FALLBACKS = (
    Path(r"C:\Windows\Fonts") / "DejaVuSerif.ttf",
    Path(r"C:\Windows\Fonts") / "LiberationSerif-Regular.ttf",
    Path(r"C:\Windows\Fonts") / "georgia.ttf",
    Path(r"C:\Windows\Fonts") / "times.ttf",
)


def find_serif(bold: bool = False) -> Path:
    preferred = FONT_BOLD if bold else FONT_REGULAR
    if preferred.exists():
        return preferred
    if bold and FONT_REGULAR.exists():
        return FONT_REGULAR
    for p in FONT_FALLBACKS:
        if p.exists():
            return p
    raise FileNotFoundError("No suitable serif font found in tools/fonts")


def rgb565(r: int, g: int, b: int) -> int:
    c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    if LCD_SWAP:
        c = ((c >> 8) | ((c & 0xFF) << 8)) & 0xFFFF
    return c


def crop_content(im: Image.Image, pad: int = 4) -> Image.Image:
    px = im.load()
    w, h = im.size
    minx, miny, maxx, maxy = w, h, 0, 0
    for y in range(h):
        for x in range(w):
            r, g, b = px[x, y]
            if r + g + b > 30:
                minx = min(minx, x)
                miny = min(miny, y)
                maxx = max(maxx, x)
                maxy = max(maxy, y)
    minx = max(0, minx - pad)
    miny = max(0, miny - pad)
    maxx = min(w - 1, maxx + pad)
    maxy = min(h - 1, maxy + pad)
    return im.crop((minx, miny, maxx + 1, maxy + 1))


def make_square(im: Image.Image) -> Image.Image:
    cw, ch = im.size
    side = max(cw, ch)
    sq = Image.new("RGB", (side, side), (0, 0, 0))
    sq.paste(im, ((side - cw) // 2, (side - ch) // 2))
    return sq


def snap_flat(im: Image.Image) -> Image.Image:
    pix = im.load()
    w, h = im.size
    for y in range(h):
        for x in range(w):
            r, g, b = pix[x, y]
            if r + g + b < 60:
                pix[x, y] = (0, 0, 0)
            elif r > 200 and g > 200 and b > 200:
                pix[x, y] = (255, 255, 255)
            elif r > 170 and g > 140 and b < 100 and r >= g - 20:
                pix[x, y] = (255, 214, 0)
            elif r > 180 and g > 160 and b < 120:
                pix[x, y] = (255, 214, 0)
    return im


def measure_radii(im: Image.Image) -> tuple[int, int]:
    size = im.size[0]
    cx = cy = size // 2
    pix = im.load()
    yellow_r = 0
    for rad in range(1, size // 2):
        ok = 0
        n = 0
        for a in range(0, 360, 6):
            x = int(cx + rad * math.cos(math.radians(a)))
            y = int(cy + rad * math.sin(math.radians(a)))
            if 0 <= x < size and 0 <= y < size:
                n += 1
                r, g, b = pix[x, y]
                if r > 200 and g > 180 and b < 40:
                    ok += 1
        if n and ok / n > 0.85:
            yellow_r = rad
        elif yellow_r > 8:
            break

    outer_r = 0
    for rad in range(size // 2 - 1, 0, -1):
        hit = 0
        for a in range(0, 360, 4):
            x = int(cx + rad * math.cos(math.radians(a)))
            y = int(cy + rad * math.sin(math.radians(a)))
            if 0 <= x < size and 0 <= y < size:
                r, g, b = pix[x, y]
                if r + g + b > 30:
                    hit += 1
        if hit > 3:
            outer_r = rad
            break
    return yellow_r, outer_r


def gen_daisy() -> None:
    im = Image.open(SRC).convert("RGB")
    im = crop_content(im)
    im = make_square(im)
    im = im.resize((DAISY_SIZE, DAISY_SIZE), Image.Resampling.BOX)
    im = snap_flat(im)
    yellow_r, outer_r = measure_radii(im)
    yellow_r = max(1, int(round(yellow_r * CENTER_SCALE)))
    cx = cy = DAISY_SIZE // 2
    draw = ImageDraw.Draw(im)
    draw.ellipse(
        (cx - yellow_r, cy - yellow_r, cx + yellow_r, cy + yellow_r),
        fill=(255, 214, 0),
    )
    pix = im.load()

    vals: list[int] = []
    mask: list[int] = []
    for y in range(DAISY_SIZE):
        for x in range(DAISY_SIZE):
            r, g, b = pix[x, y]
            vals.append(rgb565(r, g, b))
            if r > 200 and g > 200 and b > 200:
                mask.append(1)
            elif r > 200 and g > 180 and b < 40:
                mask.append(2)
            else:
                mask.append(0)

    lines = [
        "#pragma once",
        "#include <stdint.h>",
        f"#define DAISY_W {DAISY_SIZE}",
        f"#define DAISY_H {DAISY_SIZE}",
        f"#define DAISY_CENTER_R {yellow_r}",
        f"#define DAISY_OUTER_R {outer_r}",
        "",
        f"static const uint16_t daisy_img[{DAISY_SIZE * DAISY_SIZE}] = {{",
    ]
    for i in range(0, len(vals), 12):
        chunk = ", ".join(f"0x{v:04X}" for v in vals[i : i + 12])
        lines.append(f"    {chunk},")
    lines.append("};")
    lines.append("")
    lines.append(f"static const uint8_t daisy_mask[{DAISY_SIZE * DAISY_SIZE}] = {{")
    for i in range(0, len(mask), 24):
        chunk = ", ".join(str(v) for v in mask[i : i + 24])
        lines.append(f"    {chunk},")
    lines.append("};")
    lines.append("")

    OUT_H.write_text("\n".join(lines) + "\n", encoding="utf-8")
    im.save(PREVIEW)
    print(f"daisy {DAISY_SIZE}: center_r={yellow_r} outer_r={outer_r}")
    print(f"wrote {OUT_H} ({OUT_H.stat().st_size} bytes)")


def render_atlas(face: ImageFont.FreeTypeFont, cell: int, face_size: int) -> tuple[Image.Image, list[int]]:
    face = ImageFont.truetype(face.path, size=face_size) if hasattr(face, "path") else face
    # Re-open from path for exact size.
    return _render_atlas_from_path(Path(face.path) if hasattr(face, "path") else find_serif(), cell, face_size)


def _render_atlas_from_path(font_path: Path, cell: int, face_size: int) -> tuple[Image.Image, list[int]]:
    face = ImageFont.truetype(str(font_path), size=face_size)
    # Render oversized then threshold for cleaner stems.
    over = 3
    big_cell = cell * over
    atlas = Image.new("L", (cell * len(GLYPHS), cell), 0)
    advances: list[int] = []

    for i, ch in enumerate(GLYPHS):
        if ch == " ":
            advances.append(max(4, cell // 3))
            continue
        glyph = Image.new("L", (big_cell, big_cell), 0)
        gdraw = ImageDraw.Draw(glyph)
        bbox = face.getbbox(ch)
        if bbox is None:
            advances.append(cell // 2)
            continue
        # Center the ink box inside the oversampled cell.
        ink_w = bbox[2] - bbox[0]
        ink_h = bbox[3] - bbox[1]
        ox = (big_cell - ink_w) // 2 - bbox[0]
        oy = (big_cell - ink_h) // 2 - bbox[1]
        gdraw.text((ox, oy), ch, font=face, fill=255)
        small = glyph.resize((cell, cell), Image.Resampling.LANCZOS)
        # Hard threshold after downsample keeps serifs from dissolving.
        small = small.point(lambda p: 255 if p > 140 else 0)
        atlas.paste(small, (i * cell, 0))
        # Advance from font metrics, clamped to cell.
        adv = face.getlength(ch)
        advances.append(max(3, min(cell, int(round(adv / over)) + 1)))

    return atlas, advances


def pack_bits(atlas: Image.Image, cell: int) -> list[int]:
    packed: list[int] = []
    pix = atlas.load()
    for i in range(len(GLYPHS)):
        bits: list[int] = []
        for y in range(cell):
            for x in range(cell):
                bits.append(1 if pix[i * cell + x, y] > 128 else 0)
        for b in range(0, len(bits), 8):
            byte = 0
            for k in range(8):
                if b + k < len(bits) and bits[b + k]:
                    byte |= 1 << (7 - k)
            packed.append(byte)
    return packed


def emit_size_block(prefix: str, cell: int, advances: list[int], packed: list[int]) -> list[str]:
    bytes_per = (cell * cell + 7) // 8
    lines = [
        f"#define {prefix}_CELL {cell}",
        f"#define {prefix}_BYTES_PER_GLYPH {bytes_per}",
        "",
        f"static const uint8_t {prefix.lower()}_advance[SERIF_GLYPH_COUNT] = {{",
    ]
    for i in range(0, len(advances), 16):
        chunk = ", ".join(str(v) for v in advances[i : i + 16])
        lines.append(f"    {chunk},")
    lines.append("};")
    lines.append("")
    lines.append(
        f"static const uint8_t {prefix.lower()}_bits[SERIF_GLYPH_COUNT * {prefix}_BYTES_PER_GLYPH] = {{"
    )
    for i in range(0, len(packed), 16):
        chunk = ", ".join(f"0x{v:02X}" for v in packed[i : i + 16])
        lines.append(f"    {chunk},")
    lines.append("};")
    lines.append("")
    return lines


def gen_font() -> None:
    # LG: CURRENT TIME. TOT: TOTAL. MD: PAUSE/DO IT bold+narrow. SM: labels.
    # tuple: prefix, cell, face_size, bold, narrow (1.0 = normal)
    sizes = (
        ("SERIF_LG", 34, 63, False, 1.0),
        ("SERIF_TOT", 28, 54, False, 1.0),
        ("SERIF_MD", 20, 36, True, 0.82),  # +5% from 19/34
        ("SERIF_SM", 17, 33, False, 1.0),
    )

    previews: list[Image.Image] = []
    blocks: list[str] = []
    cmap = [0xFF] * 128
    for i, ch in enumerate(GLYPHS):
        cmap[ord(ch)] = i

    faces_used: list[str] = []
    header = [
        "#pragma once",
        "#include <stdint.h>",
        f"#define SERIF_GLYPH_COUNT {len(GLYPHS)}",
        "",
        "static const char serif_charset[SERIF_GLYPH_COUNT + 1] =",
        f'    "{GLYPHS}";',
        "",
        "static const uint8_t serif_cmap[128] = {",
    ]
    for i in range(0, 128, 16):
        chunk = ", ".join(f"0x{v:02X}" for v in cmap[i : i + 16])
        header.append(f"    {chunk},")
    header.append("};")
    header.append("")

    for prefix, cell, face_size, bold, narrow in sizes:
        font_path = find_serif(bold=bold)
        faces_used.append(f"{prefix}={font_path.name}")
        print(f"  {prefix}: face={font_path.name} cell={cell} pt={face_size} narrow={narrow}")
        face = ImageFont.truetype(str(font_path), size=face_size)
        over = max(2, round(face_size / cell))
        big_cell = cell * over
        atlas = Image.new("L", (cell * len(GLYPHS), cell), 0)
        advances: list[int] = []
        thr = 120 if bold else 145
        for i, ch in enumerate(GLYPHS):
            if ch == " ":
                # Wider word gap for CURRENT TIME (SM); normal elsewhere.
                if prefix == "SERIF_SM":
                    advances.append(max(9, cell // 2 + 2))
                else:
                    advances.append(max(4, cell // 3))
                continue
            glyph = Image.new("L", (big_cell + 4, big_cell + 4), 0)
            gdraw = ImageDraw.Draw(glyph)
            bbox = face.getbbox(ch)
            if bbox is None:
                advances.append(max(2, int(round(cell * narrow / 2))))
                continue
            ink_w = bbox[2] - bbox[0]
            ink_h = bbox[3] - bbox[1]
            ox = (big_cell - ink_w) // 2 - bbox[0] + 2
            oy = (big_cell - ink_h) // 2 - bbox[1] + 2
            gdraw.text((ox, oy), ch, font=face, fill=255)
            # Optional horizontal squeeze for a condensed look.
            if narrow < 0.999:
                mid_w = max(2, int(round(big_cell * narrow)))
                squeezed = glyph.resize((mid_w, big_cell + 4), Image.Resampling.LANCZOS)
                glyph = Image.new("L", (big_cell + 4, big_cell + 4), 0)
                glyph.paste(squeezed, ((big_cell + 4 - mid_w) // 2, 0))
            small = glyph.resize((cell, cell), Image.Resampling.LANCZOS)
            small = small.point(lambda p, t=thr: 255 if p > t else 0)
            atlas.paste(small, (i * cell, 0))
            # Slightly tight tracking so letters don't look sparse on LCD.
            adv_px = face.getlength(ch) * (cell / face_size) * narrow * 0.88
            advances.append(max(2, min(cell, int(round(adv_px)))))
        packed = pack_bits(atlas, cell)
        blocks.extend(emit_size_block(prefix, cell, advances, packed))
        previews.append(atlas)
        sample = "DO IT"
        width = sum(advances[GLYPHS.index(ch)] for ch in sample)
        print(f"    DO IT width={width}")

    header.insert(3, f'#define SERIF_FACE "{", ".join(faces_used)}"')
    header.insert(4, "")

    # Side-by-side preview strip.
    h = sum(p.size[1] + 4 for p in previews)
    w = max(p.size[0] for p in previews)
    sheet = Image.new("L", (w, h), 0)
    y = 0
    for p in previews:
        sheet.paste(p, (0, y))
        y += p.size[1] + 4
    sheet.save(FONT_PREVIEW)

    OUT_FONT.write_text("\n".join(header + blocks) + "\n", encoding="utf-8")
    print(f"wrote {OUT_FONT} ({OUT_FONT.stat().st_size} bytes)")


def main() -> None:
    gen_daisy()
    gen_font()


if __name__ == "__main__":
    main()
