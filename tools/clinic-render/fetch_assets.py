"""Download the CC0 Poly Haven assets the clinic render uses. Usage: python fetch_assets.py assets"""
import json, os, sys, urllib.request
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), 'assets')
def get(url):
    req=urllib.request.Request(url, headers={'User-Agent':'curl/8.5.0'})
    with urllib.request.urlopen(req) as r: return r.read()
def save(url, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if os.path.exists(path): return
    data = get(url); open(path, 'wb').write(data); print('saved', path, len(data)//1024, 'KB', flush=True)
def files(a): return json.loads(get(f'https://api.polyhaven.com/files/{a}'))
# HDRI
f = files('lilienstein')
save(f['hdri']['16k']['exr']['url'], f'{OUT}/hdri/lilienstein_16k.exr')
save(f['hdri']['2k']['exr']['url'], f'{OUT}/hdri/lilienstein_2k.exr')  # fast previews: --hdri 2k
# Textures
for t, res in [('wood_floor_deck','2k'),('gravel_floor','2k'),('plank_flooring','1k'),('brushed_concrete_03','2k')]:
    f = files(t)
    for m in ['Diffuse','nor_gl','Rough','Displacement','AO']:
        if m in f and res in f[m]:
            d = f[m][res]; fmt = 'jpg' if 'jpg' in d else 'png'
            save(d[fmt]['url'], f'{OUT}/tex/{t}/{t}_{m}_{res}.{fmt}')
# Models (gltf)
for mdl in ['grass_medium_01','grass_medium_02']:
    f = files(mdl)
    g = f['gltf'].get('2k') or f['gltf'].get('1k')
    save(g['gltf']['url'], f'{OUT}/models/{mdl}/{mdl}.gltf')
    for rel, inc in g['gltf'].get('include', {}).items():
        save(inc['url'], f'{OUT}/models/{mdl}/{rel}')
print('done')
