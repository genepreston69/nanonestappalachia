# Clinic exterior render

Builds the photoreal street view of the NanoNest L-shape clinic in the Medical section of `index.html`.

- `scene.py` models the clinic in Blender from the floor plan (38′ × 22′, three modules). It covers the rounded white shell, black-framed floor-to-ceiling glass with frosted lower panes, lit interiors, signage, the landing with an accessible 1:12 ramp, gravel drip edge and lawn. It then renders the clinic with Cycles into a real HDRI photograph.
- `fetch_assets.py` downloads the CC0 assets from Poly Haven: the Lilienstein HDRI, decking, gravel, concrete and floor textures, and grass models. They are git-ignored.
- `grade.py` adds the photographic finish (gentle contrast, warmth, vignette, grain, light sharpening) and writes the JPEG.

```sh
pip install bpy==5.1.2            # Blender as a Python module, Python 3.13
python fetch_assets.py assets     # ~420 MB, mostly the 16k HDRI
python scene.py --w 960 --samples 32 --hdri 2k --out preview.png   # quick look
python scene.py --w 1920 --samples 256 --out raw.png               # ~20 min on 4 CPU cores
python grade.py raw.png clinic-exterior.jpg
```

Useful flags: `--yaw` (camera heading on the HDRI), `--dist`, `--lens`, `--shift`, `--bphi` (rotate the building), `--nogreen` (skip vegetation for fast tests) and `--border x0,x1,y0,y1` (render one region).
