"""The balance sigil's textures (K2_BALANCE_SIGIL, kinetic_sigil.lua).

Writes scripts/data/ragdoll_manual/textures/kinetic_brass_sigil_*.png:
  frame        the black panel, its bronze double border with corner knots and
               the double-line triangle (280 design units square)
  eye, maze, boot   the three sense tiles, engraved: a bright metal line over
               a darker one (dark silver, gold, bronze)
  alert        a tile's border alone, white (tinted red over an impaired sense)
  moon_0..8    a foot: a ring lit from the right by eighths, white (tinted)
  sun, star    the centre of mass and the capture point, white (tinted)

Run from the checkout: python tools/body/sigil_textures.py
"""
import math
import os

from PIL import Image, ImageDraw

OUT = os.path.join(os.path.dirname(__file__), '..', '..', 'scripts', 'data', 'ragdoll_manual', 'textures')
SS = 4  # supersampling

BLACK = (10, 9, 8)
BRONZE, BRONZE_DARK, BRONZE_LIGHT = (182, 124, 66), (94, 58, 26), (221, 164, 104)
GOLD, GOLD_DARK, GOLD_LIGHT = (212, 175, 90), (122, 90, 30), (240, 213, 136)
SILVER, SILVER_DARK, SILVER_LIGHT = (124, 130, 136), (58, 63, 68), (163, 169, 175)
WHITE = (255, 255, 255)


class Canvas:
    """A design-unit canvas drawn supersampled and scaled down on save."""

    def __init__(self, units, pixels):
        self.k = pixels * SS / units
        self.pixels = pixels
        self.image = Image.new('RGBA', (pixels * SS, pixels * SS), (0, 0, 0, 0))
        self.draw = ImageDraw.Draw(self.image)

    def p(self, x, y):
        return (x * self.k, y * self.k)

    def line(self, points, colour, width, closed=False):
        pts = [self.p(x, y) for x, y in points]
        if closed:
            pts.append(pts[0])
        w = max(1, round(width * self.k))
        self.draw.line(pts, fill=colour + (255,), width=w, joint='curve')
        for x, y in pts:  # round caps and joins
            self.draw.ellipse([x - w / 2, y - w / 2, x + w / 2, y + w / 2], fill=colour + (255,))

    def rect(self, x, y, w, h, outline=None, width=1.0, fill=None, alpha=255):
        if fill is not None:
            self.draw.rectangle([self.p(x, y), self.p(x + w, y + h)], fill=fill + (alpha,))
        if outline is not None:
            self.line([(x, y), (x + w, y), (x + w, y + h), (x, y + h)], outline, width, closed=True)

    def circle(self, cx, cy, r, outline=None, width=1.0, fill=None):
        box = [self.p(cx - r, cy - r), self.p(cx + r, cy + r)]
        if fill is not None:
            self.draw.ellipse(box, fill=fill + (255,))
        if outline is not None:
            self.draw.ellipse(box, outline=outline + (255,), width=max(1, round(width * self.k)))

    def polygon(self, points, fill):
        self.draw.polygon([self.p(x, y) for x, y in points], fill=fill + (255,))

    def save(self, name):
        path = os.path.join(OUT, 'kinetic_brass_sigil_%s.png' % name)
        self.image.resize((self.pixels, self.pixels), Image.LANCZOS).save(path)
        return path


def quad(a, c, b, n=24):
    """A quadratic curve from a to b with control c."""
    return [((1 - t) ** 2 * a[0] + 2 * (1 - t) * t * c[0] + t * t * b[0],
             (1 - t) ** 2 * a[1] + 2 * (1 - t) * t * c[1] + t * t * b[1]) for t in (i / n for i in range(n + 1))]


def cubic(a, c1, c2, b, n=24):
    return [((1 - t) ** 3 * a[0] + 3 * (1 - t) ** 2 * t * c1[0] + 3 * (1 - t) * t * t * c2[0] + t ** 3 * b[0],
             (1 - t) ** 3 * a[1] + 3 * (1 - t) ** 2 * t * c1[1] + 3 * (1 - t) * t * t * c2[1] + t ** 3 * b[1])
            for t in (i / n for i in range(n + 1))]


def ring(cx, cy, r, n=48):
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def tile(c, colour, x=0.0, y=0.0):
    """A sense tile's frame, a small copy of the window's: 44 units square."""
    c.rect(x, y, 44, 44, fill=BLACK, alpha=255)
    c.rect(x, y, 44, 44, outline=colour, width=1.4)
    c.rect(x + 3.5, y + 3.5, 37, 37, outline=colour, width=0.5)
    for dx in (-2.5, 41.5):
        for dy in (-2.5, 41.5):
            c.rect(x + dx, y + dy, 5, 5, fill=colour)


def engraved(c, strokes, dark, light, ox=0.0, oy=0.0):
    for colour, width in ((dark, 3.2), (light, 1.2)):
        for points, closed in strokes:
            c.line([(x + ox, y + oy) for x, y in points], colour, width, closed)


EYE = [
    (quad((7, 23), (22, 9), (37, 23)) + quad((37, 23), (22, 37), (7, 23)), True),
    (ring(22, 23, 6), True), (ring(22, 23, 1.4, 12), True),
    ([(13, 14), (11, 10)], False), ([(22, 12), (22, 7.5)], False), ([(31, 14), (33, 10)], False),
]
MAZE = [([(10, 10), (34, 10), (34, 34), (10, 34), (10, 16), (28, 16), (28, 28), (16, 28), (16, 22), (22, 22)], False)]
BOOT = [
    ([(14, 8), (26, 8), (26, 25), (35, 29)] + quad((35, 29), (38, 30), (38, 34), 8) + [(38, 36), (14, 36)], True),
    ([(14, 14), (26, 14)], False), ([(14, 32), (38, 32)], False),
]


def sense(name, strokes, colour, dark, light):
    c = Canvas(52, 64)  # the tile with its corner studs, 4 units of margin
    tile(c, colour, 4, 4)
    engraved(c, strokes, dark, light, 4, 4)
    return c.save(name)


def alert():
    c = Canvas(52, 64)
    c.rect(4, 4, 44, 44, outline=WHITE, width=1.8)
    for dx in (-2.5, 41.5):
        for dy in (-2.5, 41.5):
            c.rect(4 + dx, 4 + dy, 5, 5, fill=WHITE)
    return c.save('alert')


def frame():
    c = Canvas(280, 256)
    c.rect(9, 9, 262, 262, fill=BLACK, alpha=165)
    c.rect(9, 9, 262, 262, outline=BRONZE, width=1.6)
    c.rect(13, 13, 254, 254, outline=BRONZE, width=0.6)
    for x in (3, 265):
        for y in (3, 265):
            c.rect(x, y, 12, 12, fill=BLACK)
            c.rect(x, y, 12, 12, outline=GOLD, width=1.2)
            c.rect(x + 3.5, y + 3.5, 5, 5, outline=GOLD, width=1.2)
    c.line([(140, 44), (236, 226), (44, 226)], BRONZE, 1.4, closed=True)
    c.line([(140, 58), (224, 219), (56, 219)], BRONZE, 0.5, closed=True)
    return c.save('frame')


def moon(k):
    """A ring lit from the right: k eighths (0 the empty ring, 8 full)."""
    c = Canvas(34, 64)
    r, cx, cy = 15.0, 17.0, 17.0
    if k > 0:
        # The terminator is a half ellipse: bulging right for a crescent,
        # left for a gibbous moon.
        rx = r * (1 - k / 4.0)
        outer = [(cx + r * math.sin(a), cy - r * math.cos(a)) for a in (math.pi * i / 40 for i in range(41))]
        inner = [(cx + rx * math.sin(a), cy + r * math.cos(a)) for a in (math.pi * i / 40 for i in range(41))]
        c.polygon(outer + inner, WHITE)
    c.circle(cx, cy, r, outline=WHITE, width=1.5)
    return c.save('moon_%d' % k)


def sun():
    c = Canvas(18, 32)
    c.circle(9, 9, 6.5, fill=BLACK)
    c.circle(9, 9, 6.5, outline=WHITE, width=1.8)
    c.circle(9, 9, 1.8, fill=WHITE)
    return c.save('sun')


def star():
    c = Canvas(24, 32)
    r, s = 10.0, 2.6
    pts = []
    for i in range(4):
        a = math.pi / 2 * i
        pts.append((12 + r * math.sin(a), 12 - r * math.cos(a)))
        b = a + math.pi / 4
        pts.append((12 + s * math.sqrt(2) * math.sin(b), 12 - s * math.sqrt(2) * math.cos(b)))
    c.polygon(pts, WHITE)
    return c.save('star')


if __name__ == '__main__':
    made = [frame(), sense('eye', EYE, SILVER, SILVER_DARK, SILVER_LIGHT), sense('maze', MAZE, GOLD, GOLD_DARK, GOLD_LIGHT),
            sense('boot', BOOT, BRONZE, BRONZE_DARK, BRONZE_LIGHT), alert(), sun(), star()]
    made += [moon(k) for k in range(9)]
    for path in made:
        print(os.path.normpath(path))
