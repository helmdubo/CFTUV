"""Compose the R1.9 visual before/after gate from deterministic renders."""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent
BEFORE = ROOT / "decal_r19_before_extreme.png"
AFTER = ROOT / "decal_r19_after_extreme.png"
OUTPUT = ROOT / "decal_r19_before_after_annotated.png"
FONT_PATH = Path(r"C:\Windows\Fonts\arial.ttf")


def _font(size):
    return ImageFont.truetype(str(FONT_PATH), size=size)


def main():
    before = Image.open(BEFORE).convert("RGB").resize((1200, 733))
    after = Image.open(AFTER).convert("RGB").resize((1200, 733))
    canvas = Image.new("RGB", (2400, 850), (12, 16, 24))
    canvas.paste(before, (0, 117))
    canvas.paste(after, (1200, 117))
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((0, 0, 1199, 116), fill=(112, 18, 24))
    draw.rectangle((1200, 0, 2399, 116), fill=(17, 91, 45))
    draw.text(
        (36, 19),
        "BEFORE R1.9 - FINAL ERROR",
        font=_font(38),
        fill=(255, 235, 235),
    )
    draw.text(
        (36, 67),
        "TERMINAL_BRIDGE_CUT_INVALID v10/e8; no decal emitted",
        font=_font(24),
        fill=(255, 190, 190),
    )
    draw.text(
        (1236, 19),
        "AFTER R1.9 - SATURATED",
        font=_font(38),
        fill=(225, 255, 231),
    )
    draw.text(
        (1236, 67),
        "width=102.752491 (10x mesh diagonal); preview == final",
        font=_font(24),
        fill=(185, 255, 203),
    )
    arrow_start = (2200, 210)
    arrow_end = (1713, 423)
    draw.line((arrow_start, arrow_end), fill=(255, 224, 30), width=10)
    draw.polygon(
        (
            arrow_end,
            (arrow_end[0] + 35, arrow_end[1] - 4),
            (arrow_end[0] + 19, arrow_end[1] - 31),
        ),
        fill=(255, 224, 30),
    )
    draw.rounded_rectangle(
        (1840, 220, 2370, 310),
        radius=14,
        fill=(9, 18, 30),
        outline=(255, 224, 30),
        width=4,
    )
    draw.text(
        (1861, 238),
        "v10/e8: last valid station\nSAT:3  CLAMP:2",
        font=_font(25),
        fill=(255, 240, 120),
    )
    canvas.save(OUTPUT)
    print(OUTPUT)


if __name__ == "__main__":
    main()
