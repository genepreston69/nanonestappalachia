"""Photographic finish for the Cycles render: gentle S-curve, warmth, vignette, grain, light sharpening.

Usage: python3 grade.py in.png out.jpg [quality]
"""
import sys
import numpy as np
from PIL import Image, ImageFilter

src, dst = sys.argv[1], sys.argv[2]
quality = int(sys.argv[3]) if len(sys.argv) > 3 else 84

im = Image.open(src)
a = np.asarray(im).astype(np.float64)
a = a[..., :3] / (65535.0 if a.max() > 255 else 255.0)

# gentle S-curve for a little more snap than the AgX look gives
s = a * a * (3 - 2 * a)
a = a * 0.86 + s * 0.14
# late-day warmth
a[..., 0] *= 1.018
a[..., 2] *= 0.975
# soft lens vignette
h, w = a.shape[:2]
yy, xx = np.mgrid[0:h, 0:w]
r = np.hypot((xx - w / 2) / (w / 2), (yy - h / 2) / (h / 2)) / np.sqrt(2)
a *= (1 - 0.14 * r ** 2.2)[..., None]
# fine luminance grain
rng = np.random.default_rng(3)
a += rng.normal(0, 0.0065, (h, w, 1))
a = np.clip(a, 0, 1)

out = Image.fromarray((a * 255 + 0.5).astype(np.uint8))
out = out.filter(ImageFilter.UnsharpMask(radius=1.1, percent=45, threshold=2))
out.save(dst, quality=quality, optimize=True, progressive=True)
print(dst, out.size)
