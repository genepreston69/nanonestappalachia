"""Photoreal exterior of the NanoNest L-shape clinic (38' x 22'), rendered into a real HDRI.

Setup: pip install bpy==5.1.2 (Python 3.13), then python fetch_assets.py assets
Preview: python scene.py --w 960 --samples 32 --out test.png [--nogreen] [--hdri 2k]
Final:   python scene.py --w 1920 --samples 256 --out raw.png && python grade.py raw.png clinic.jpg
Building local frame: x runs west->east along the 38' street face, y runs front->back, z up.
The street (front) face is the plane y = 0 and faces -y.
"""
import argparse, math, os, random, sys
import bpy, bmesh
from mathutils import Vector, Matrix

HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, 'assets')
ap = argparse.ArgumentParser()
ap.add_argument('--w', type=int, default=960)
ap.add_argument('--samples', type=int, default=32)
ap.add_argument('--out', default=os.path.join(HERE, 'render.png'))
ap.add_argument('--hdri', default='16k')
ap.add_argument('--yaw', type=float, default=120.0)      # camera heading, degrees CCW from +X
ap.add_argument('--dist', type=float, default=17.0)      # camera to building centre, metres
ap.add_argument('--lens', type=float, default=35.0)
ap.add_argument('--shift', type=float, default=0.05)
ap.add_argument('--bphi', type=float, default=0.0)       # extra building rotation, degrees
ap.add_argument('--nogreen', action='store_true')        # skip plant models (fast tests)
ap.add_argument('--blend', default='')
ap.add_argument('--border', default='')            # xmin,xmax,ymin,ymax (0-1, y from bottom)
args = ap.parse_args()

FT = 0.3048
L, DF, WB, D = 38 * FT, 15 * FT, 19 * FT, 22 * FT       # 11.58, 4.57, 5.79, 6.71
Z0, Z1 = 0.30, 3.30                                      # shell underside / roof top
FLOOR, CEIL = 0.62, 3.10
T = 0.14                                                 # wall thickness
R = 0.32                                                 # shell edge radius

random.seed(7)
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
col_main = scene.collection

# ---------------------------------------------------------------- helpers
def link(obj, parent=None):
    col_main.objects.link(obj)
    if parent is not None:
        obj.parent = parent
    return obj

def mesh_obj(name, verts, faces, mats=(), face_mats=None, parent=None):
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in verts], [], faces)
    me.validate()
    me.update()
    ob = bpy.data.objects.new(name, me)
    for m in mats:
        me.materials.append(m)
    if face_mats:
        for p, mi in zip(me.polygons, face_mats):
            p.material_index = mi
    return link(ob, parent)

def box(name, x0, x1, y0, y1, z0, z1, mat, parent=None, side_mat=None):
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    if side_mat is not None:
        return mesh_obj(name, v, f, [mat, side_mat], face_mats=[1, 0, 1, 1, 1, 1], parent=parent)
    return mesh_obj(name, v, f, [mat] if mat else [], parent=parent)

def prism(name, poly, z0, z1, mats=(), parent=None, face_mats=None):
    """Extrude a CCW 2D polygon (x, y) from z0 to z1."""
    n = len(poly)
    v = [(x, y, z0) for x, y in poly] + [(x, y, z1) for x, y in poly]
    f = [tuple(reversed(range(n))), tuple(range(n, 2 * n))]
    for i in range(n):
        j = (i + 1) % n
        f.append((i, j, n + j, n + i))
    return mesh_obj(name, v, f, mats, face_mats=face_mats, parent=parent)

def rr(u0, u1, v0, v1, r, n=8):
    """Rounded rectangle, CCW in (u, v)."""
    r = min(r, (u1 - u0) / 2, (v1 - v0) / 2)
    pts = []
    for cu, cv, a0 in [(u1 - r, v0 + r, -90), (u1 - r, v1 - r, 0), (u0 + r, v1 - r, 90), (u0 + r, v0 + r, 180)]:
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((cu + r * math.cos(a), cv + r * math.sin(a)))
    return pts

# wall frames: (u along wall seen from outside, v up, w depth into wall from the outer face)
def W_front(u, v, w):
    return (u, w, v)
def W_east(u, v, w):
    return (L - w, u, v)

def wall_prism(name, poly, w0, w1, wall, mats, parent):
    n = len(poly)
    v = [wall(u, vv, w0) for u, vv in poly] + [wall(u, vv, w1) for u, vv in poly]
    f = [tuple(range(n)), tuple(reversed(range(n, 2 * n)))]
    for i in range(n):
        j = (i + 1) % n
        f.append((j, i, n + i, n + j))
    ob = mesh_obj(name, v, f, mats, parent=parent)
    fix_normals(ob)
    return ob

def wall_ring(name, outer, inner, w0, w1, wall, mats, parent):
    n = len(outer)
    assert n == len(inner)
    v = ([wall(u, vv, w0) for u, vv in outer] + [wall(u, vv, w0) for u, vv in inner] +
         [wall(u, vv, w1) for u, vv in outer] + [wall(u, vv, w1) for u, vv in inner])
    f = []
    for i in range(n):
        j = (i + 1) % n
        f.append((i, j, n + j, n + i))                  # front face (w0)
        f.append((2 * n + i, 3 * n + i, 3 * n + j, 2 * n + j))  # back face (w1)
        f.append((i, 2 * n + i, 2 * n + j, j))          # outer side
        f.append((n + i, n + j, 3 * n + j, 3 * n + i))  # inner side
    ob = mesh_obj(name, v, f, mats, parent=parent)
    fix_normals(ob)
    return ob

def wall_box(name, u0, u1, v0, v1, w0, w1, wall, mat, parent):
    return wall_prism(name, [(u0, v0), (u1, v0), (u1, v1), (u0, v1)], w0, w1, wall, [mat], parent)

def fix_normals(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(ob.data)
    bm.free()

def segment(name, p0, p1, s, mat, parent, round_=False):
    """Square (or 8-sided) bar of thickness s between two points."""
    p0, p1 = Vector(p0), Vector(p1)
    d = p1 - p0
    length = d.length
    if round_:
        bpy.ops.mesh.primitive_cylinder_add(vertices=12, radius=s / 2, depth=length)
    else:
        bpy.ops.mesh.primitive_cube_add(size=1)
    ob = bpy.context.active_object
    ob.name = name
    if not round_:
        ob.scale = (s, s, length)
    ob.location = (p0 + p1) / 2
    ob.rotation_mode = 'QUATERNION'
    ob.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(d.normalized())
    ob.data.materials.append(mat)
    if parent is not None:
        ob.parent = parent
    return ob

def apply_mods(ob):
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active = ob
    ob.select_set(True)
    for m in list(ob.modifiers):
        bpy.ops.object.modifier_apply(modifier=m.name)

# ---------------------------------------------------------------- materials
def new_mat(name):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    return m, m.node_tree.nodes, m.node_tree.links, m.node_tree.nodes['Principled BSDF']

def setp(bsdf, **kw):
    names = {'base': 'Base Color', 'metal': 'Metallic', 'rough': 'Roughness', 'ior': 'IOR',
             'trans': 'Transmission Weight', 'coat': 'Coat Weight', 'coat_rough': 'Coat Roughness',
             'emit': 'Emission Color', 'emit_s': 'Emission Strength', 'spec': 'Specular IOR Level',
             'sss': 'Subsurface Weight', 'alpha': 'Alpha'}
    for k, v in kw.items():
        inp = bsdf.inputs[names[k]]
        inp.default_value = (*v, 1.0) if isinstance(v, tuple) and len(v) == 3 else v

def srgb(h):
    h = h.lstrip('#')
    c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return tuple(x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c)

def simple(name, color, rough=0.5, metal=0.0, **kw):
    m, n, l, b = new_mat(name)
    setp(b, base=color, rough=rough, metal=metal, **kw)
    return m

def tex_mat(name, folder, res, scale, rough_mul=1.0, bump=0.3, tint=None, proj='BOX', sat=1.0, val=1.0):
    """Poly Haven texture set, box-projected in object space; scale = metres per tile."""
    m, n, l, b = new_mat(name)
    tc = n.new('ShaderNodeTexCoord')
    mp = n.new('ShaderNodeMapping')
    mp.inputs['Scale'].default_value = (1 / scale, 1 / scale, 1 / scale)
    l.new(tc.outputs['Object'], mp.inputs['Vector'])
    def img(kind, colorspace):
        p = os.path.join(A, 'tex', folder, f'{folder}_{kind}_{res}.jpg')
        t = n.new('ShaderNodeTexImage')
        t.image = bpy.data.images.load(p, check_existing=True)
        t.image.colorspace_settings.name = colorspace
        t.projection = proj
        t.projection_blend = 0.25
        l.new(mp.outputs['Vector'], t.inputs['Vector'])
        return t
    dif = img('Diffuse', 'sRGB')
    col = dif.outputs['Color']
    if sat != 1.0 or val != 1.0:
        hs = n.new('ShaderNodeHueSaturation'); hs.inputs['Saturation'].default_value = sat
        hs.inputs['Value'].default_value = val
        l.new(col, hs.inputs['Color']); col = hs.outputs['Color']
    if tint:
        mix = n.new('ShaderNodeMix'); mix.data_type = 'RGBA'; mix.blend_type = 'MULTIPLY'
        mix.inputs['Factor'].default_value = 1.0
        l.new(col, mix.inputs['A']); mix.inputs['B'].default_value = (*tint, 1)
        col = mix.outputs['Result']
    l.new(col, b.inputs['Base Color'])
    rgh = img('Rough', 'Non-Color')
    mul = n.new('ShaderNodeMath'); mul.operation = 'MULTIPLY'; mul.inputs[1].default_value = rough_mul
    l.new(rgh.outputs['Color'], mul.inputs[0]); l.new(mul.outputs[0], b.inputs['Roughness'])
    dsp = img('Displacement', 'Non-Color')
    bp = n.new('ShaderNodeBump'); bp.inputs['Strength'].default_value = bump
    bp.inputs['Distance'].default_value = 0.01
    l.new(dsp.outputs['Color'], bp.inputs['Height']); l.new(bp.outputs['Normal'], b.inputs['Normal'])
    return m

def glass_mat(name, rough=0.0, tint=(0.86, 0.92, 0.90)):
    m, n, l, b = new_mat(name)
    setp(b, base=tint, rough=rough, ior=1.52, trans=1.0)
    out = n['Material Output']
    lp = n.new('ShaderNodeLightPath')
    tr = n.new('ShaderNodeBsdfTransparent'); tr.inputs['Color'].default_value = (0.82, 0.86, 0.84, 1)
    mx = n.new('ShaderNodeMixShader')
    l.new(lp.outputs['Is Shadow Ray'], mx.inputs['Fac'])
    l.new(b.outputs['BSDF'], mx.inputs[1]); l.new(tr.outputs['BSDF'], mx.inputs[2])
    l.new(mx.outputs['Shader'], out.inputs['Surface'])
    return m

def ext_white():
    """Arctic White aluminium shell: glossy coat, faint roughness noise, module seam on the east end."""
    m, n, l, b = new_mat('ext_white')
    setp(b, base=srgb('#ecedea'), rough=0.24, coat=0.35, coat_rough=0.06, spec=0.5)
    tc = n.new('ShaderNodeTexCoord')
    sep = n.new('ShaderNodeSeparateXYZ'); l.new(tc.outputs['Object'], sep.inputs['Vector'])
    # module joint between the two front modules runs across the east end at y = DF/2
    sub = n.new('ShaderNodeMath'); sub.operation = 'SUBTRACT'; sub.inputs[1].default_value = DF / 2
    l.new(sep.outputs['Y'], sub.inputs[0])
    ab = n.new('ShaderNodeMath'); ab.operation = 'ABSOLUTE'; l.new(sub.outputs[0], ab.inputs[0])
    sm = n.new('ShaderNodeMapRange'); sm.inputs['From Min'].default_value = 0.004
    sm.inputs['From Max'].default_value = 0.007; sm.inputs['To Min'].default_value = 1.0
    sm.inputs['To Max'].default_value = 0.0
    l.new(ab.outputs[0], sm.inputs['Value'])
    gate = n.new('ShaderNodeMath'); gate.operation = 'GREATER_THAN'; gate.inputs[1].default_value = WB + 0.4
    l.new(sep.outputs['X'], gate.inputs[0])
    seam = n.new('ShaderNodeMath'); seam.operation = 'MULTIPLY'
    l.new(sm.outputs['Result'], seam.inputs[0]); l.new(gate.outputs[0], seam.inputs[1])
    mix = n.new('ShaderNodeMix'); mix.data_type = 'RGBA'
    l.new(seam.outputs[0], mix.inputs['Factor'])
    mix.inputs['A'].default_value = (*srgb('#ecedea'), 1); mix.inputs['B'].default_value = (0.05, 0.05, 0.05, 1)
    l.new(mix.outputs['Result'], b.inputs['Base Color'])
    # gentle roughness variation so the panels don't read as CG-perfect
    nz = n.new('ShaderNodeTexNoise'); nz.inputs['Scale'].default_value = 1.5; nz.inputs['Detail'].default_value = 6
    l.new(tc.outputs['Object'], nz.inputs['Vector'])
    mr = n.new('ShaderNodeMapRange'); mr.inputs['To Min'].default_value = 0.18; mr.inputs['To Max'].default_value = 0.32
    l.new(nz.outputs['Fac'], mr.inputs['Value']); l.new(mr.outputs['Result'], b.inputs['Roughness'])
    bp = n.new('ShaderNodeBump'); bp.inputs['Strength'].default_value = 0.6; bp.inputs['Distance'].default_value = 0.003
    l.new(seam.outputs[0], bp.inputs['Height']); bp.invert = True
    l.new(bp.outputs['Normal'], b.inputs['Normal'])
    return m

M = {}
M['ext'] = ext_white()
M['reveal'] = simple('reveal', srgb('#e6e7e3'), 0.35)
M['alu'] = simple('black_alu', (0.012, 0.013, 0.014), 0.38, 0.6)
M['glass'] = glass_mat('glass')
M['frost'] = glass_mat('frosted', rough=0.42, tint=(0.93, 0.95, 0.95))
M['plinth'] = simple('plinth', (0.028, 0.030, 0.029), 0.75)
M['wall'] = simple('int_wall', srgb('#ebe8e1'), 0.85)
M['ceil'] = simple('int_ceiling', srgb('#f1f0ec'), 0.9)
M['floor'] = tex_mat('int_floor', 'plank_flooring', '1k', 1.6, 0.8, 0.15, tint=(1.05, 1.0, 0.92))
M['deck'] = tex_mat('deck', 'wood_floor_deck', '2k', 2.0, 1.1, 0.5, sat=0.42, val=0.95)
M['fascia'] = tex_mat('fascia', 'wood_floor_deck', '2k', 1.2, 1.1, 0.3, sat=0.3, val=0.55)
M['gravel'] = tex_mat('gravel', 'gravel_floor', '2k', 2.2, 1.0, 2.5, sat=0.3, val=0.48)
M['path'] = tex_mat('path', 'brushed_concrete_03', '2k', 2.5, 1.0, 0.4)
M['steel'] = simple('steel', (0.75, 0.75, 0.74), 0.22, 1.0)
M['rail'] = simple('rail_black', (0.015, 0.016, 0.017), 0.45, 0.3)
M['fabric'] = simple('fabric', srgb('#3f5a4a'), 0.9, sss=0.0)
M['wood_int'] = tex_mat('wood_int', 'plank_flooring', '1k', 1.2, 0.7, 0.05, tint=(1.0, 0.9, 0.8))
M['white_furn'] = simple('white_furn', srgb('#f2f1ee'), 0.4)
M['sign_green'] = simple('sign_green', srgb('#0b5a35'), 0.35, coat=0.6, coat_rough=0.05)
M['sign_white'] = simple('sign_white', (0.9, 0.9, 0.88), 0.3)
M['letters'] = simple('letters', srgb('#26302b'), 0.4, 0.4)
M['planter'] = simple('planter', srgb('#e9e8e4'), 0.45)
M['soil'] = simple('soil', (0.035, 0.025, 0.018), 0.95)
M['grille'] = simple('grille', (0.03, 0.03, 0.03), 0.6)
M['art'] = simple('art', srgb('#6f8f6a'), 0.6)
m, n, l, b = new_mat('downlight')
setp(b, base=(1, 1, 1), emit=(1.0, 0.86, 0.68), emit_s=25.0)
M['emit'] = m

# ---------------------------------------------------------------- building
B = bpy.data.objects.new('Building', None)
link(B)

def l_poly(i):
    return [(i, i), (L - i, i), (L - i, DF - i), (WB - i, DF - i), (WB - i, D - i), (i, D - i)]

outer = prism('shell', l_poly(0), Z0, Z1, [M['ext']], parent=B)
bv = outer.modifiers.new('bevel', 'BEVEL')
bv.width = R; bv.segments = 16; bv.limit_method = 'ANGLE'; bv.angle_limit = math.radians(60)
bv.profile = 0.5
apply_mods(outer)

cutters = bpy.data.collections.new('cutters')
scene.collection.children.link(cutters)
def to_cutters(ob):
    for c in ob.users_collection:
        c.objects.unlink(ob)
    cutters.objects.link(ob)
    ob.hide_render = True

inner = prism('inner', l_poly(T), FLOOR, CEIL, [M['wall'], M['floor'], M['ceil']], parent=B,
              face_mats=[1, 2] + [0] * 6)
to_cutters(inner)

# glazing schedule ------------------------------------------------------
GB, GT = 0.66, 2.90           # glass opening bottom / top (floor-to-ceiling)
TRANSOM = 1.88                # frosted below this line
openings = [
    # wall, u0, u1, v0, v1, mullion u's, bays [(kind)], door bay index
    dict(wall=W_front, u0=0.50, u1=5.45, v0=GB, v1=GT,
         mull=[1.30, 2.13, 2.96, 3.81, 4.72],
         bays=['frost', 'frost', 'clear', 'clear', 'door', 'clear']),
    dict(wall=W_front, u0=9.05, u1=11.10, v0=GB, v1=GT, mull=[10.075], bays=['frost', 'frost']),
    dict(wall=W_east, u0=2.30, u1=3.70, v0=GB, v1=GT, mull=[], bays=['frost']),          # Exam 3
    dict(wall=W_east, u0=0.55, u1=1.30, v0=1.45, v1=2.35, mull=[], bays=['clear']),      # office
]
for k, o in enumerate(openings):
    wall = o['wall']
    c = wall_prism(f'cut{k}', rr(o['u0'], o['u1'], o['v0'], o['v1'], 0.12), -0.4, T + 0.06, wall,
                   [M['reveal']], B)
    to_cutters(c)
    # raised lip around the opening, the NanoNest picture-frame detail
    lp = 0.075
    wall_ring(f'lip{k}', rr(o['u0'] - lp, o['u1'] + lp, o['v0'] - lp, o['v1'] + lp, 0.12 + lp),
              rr(o['u0'], o['u1'], o['v0'], o['v1'], 0.12), -0.022, 0.0, wall, [M['ext']], B)
    # black aluminium perimeter frame, set back in the reveal
    fw = 0.05
    wall_ring(f'frame{k}', rr(o['u0'] - 0.004, o['u1'] + 0.004, o['v0'] - 0.004, o['v1'] + 0.004, 0.124),
              rr(o['u0'] + fw, o['u1'] - fw, o['v0'] + fw, o['v1'] - fw, 0.07), 0.035, 0.105, wall,
              [M['alu']], B)
    edges = [o['u0']] + o['mull'] + [o['u1']]
    for j, mu in enumerate(o['mull']):
        wall_box(f'mull{k}_{j}', mu - 0.026, mu + 0.026, o['v0'] + 0.02, o['v1'] - 0.02, 0.035, 0.11, wall,
                 M['alu'], B)
    for j, kind in enumerate(o['bays']):
        a, z = edges[j], edges[j + 1]
        if kind == 'frost':
            wall_box(f'tr{k}_{j}', a, z, TRANSOM - 0.025, TRANSOM + 0.025, 0.035, 0.105, wall, M['alu'], B)
            wall_box(f'gl{k}_{j}', a, z, TRANSOM, o['v1'], 0.064, 0.074, wall, M['glass'], B)
            wall_box(f'fr{k}_{j}', a, z, o['v0'], TRANSOM, 0.064, 0.074, wall, M['frost'], B)
        else:
            wall_box(f'gl{k}_{j}', a, z, o['v0'], o['v1'], 0.064, 0.074, wall, M['glass'], B)
        if kind == 'door':
            wall_box(f'rail{k}_{j}', a, z, o['v0'], o['v0'] + 0.13, 0.03, 0.11, wall, M['alu'], B)
            wall_box(f'head{k}_{j}', a, z, 2.74, 2.79, 0.03, 0.11, wall, M['alu'], B)
            # stainless pull handle on the latch side
            hu = a + 0.12
            segment('pull', wall(hu, 1.05, -0.07), wall(hu, 2.05, -0.07), 0.03, M['steel'], B, round_=True)
            for hv in (1.15, 1.95):
                segment('stand', wall(hu, hv, -0.07), wall(hu, hv, 0.03), 0.016, M['steel'], B, round_=True)

shell_bool = outer.modifiers.new('cut_inner', 'BOOLEAN')
shell_bool.operation = 'DIFFERENCE'; shell_bool.solver = 'EXACT'
shell_bool.operand_type = 'COLLECTION'; shell_bool.collection = cutters
shell_bool.material_mode = 'TRANSFER'
apply_mods(outer)
bpy.data.collections.remove(cutters)

# dark recessed plinth under the shell
prism('plinth', l_poly(0.20), 0.0, Z0 + 0.12, [M['plinth']], parent=B)

# ---------------------------------------------------------------- signage
# green cross panel
panel = wall_prism('sign', rr(5.98, 6.56, 2.12, 2.70, 0.06), -0.035, 0.0, W_front, [M['sign_green']], B)
cw, cl = 0.052, 0.38
cu, cv = 6.27, 2.41
wall_box('cross_h', cu - cl / 2, cu + cl / 2, cv - cw, cv + cw, -0.042, -0.03, W_front, M['sign_white'], B)
wall_box('cross_v', cu - cw, cu + cw, cv - cl / 2, cv + cl / 2, -0.044, -0.03, W_front, M['sign_white'], B)

def text(body, size, u, v, extrude=0.008):
    cu_ = bpy.data.curves.new(body, 'FONT')
    cu_.body = body; cu_.size = size; cu_.extrude = extrude; cu_.align_y = 'BOTTOM'
    cu_.space_character = 1.06
    ob = bpy.data.objects.new(body, cu_)
    ob.data.materials.append(M['letters'])
    link(ob, B)
    ob.rotation_euler = (math.radians(90), 0, 0)
    ob.location = (u, -0.012, v)
    return ob
text('COMMUNITY', 0.13, 6.74, 2.52)
text('CLINIC', 0.25, 6.71, 2.13)

# louvred vent by the utility room
wall_box('vent', 8.20, 8.52, 2.20, 2.52, -0.02, 0.0, W_front, M['reveal'], B)
for i in range(6):
    vz = 2.23 + i * 0.05
    wall_box(f'louvre{i}', 8.22, 8.50, vz, vz + 0.012, -0.03, -0.02, W_front, M['grille'], B)

# ---------------------------------------------------------------- interior
def iwall(x0, x1, y0, y1, gaps=(), axis='x', name='iw'):
    """Partition from floor to ceiling; gaps along its run as (a, b, head)."""
    run0, run1 = (x0, x1) if axis == 'x' else (y0, y1)
    cuts = sorted(gaps)
    segs, cur = [], run0
    for a, b_, head in cuts:
        segs.append((cur, a, FLOOR, CEIL)); segs.append((a, b_, head, CEIL)); cur = b_
    segs.append((cur, run1, FLOOR, CEIL))
    for i, (a, b_, za, zb) in enumerate(segs):
        if b_ - a < 1e-3:
            continue
        if axis == 'x':
            box(f'{name}{i}', a, b_, y0, y1, za, zb, M['wall'], B)
        else:
            box(f'{name}{i}', x0, x1, a, b_, za, zb, M['wall'], B)

y8, y6, y12 = 8 * FT, 6 * FT, 12 * FT
iwall(T, 26 * FT, y8 - 0.05, y8 + 0.05, gaps=[(1 * FT, 4 * FT, 2.75), (15 * FT, 18 * FT, 2.75), (22 * FT, 25 * FT, 2.75)], name='p8_')
iwall(T, WB, y12 - 0.05, y12 + 0.05, gaps=[(6 * FT, 9 * FT, 2.75), (10 * FT, 13 * FT, 2.75)], name='p12_')
# reception / waiting partition with a check-in window
x7 = 7 * FT
box('p7a', x7 - 0.05, x7 + 0.05, T, 0.45, FLOOR, CEIL, M['wall'], B)
box('p7b', x7 - 0.05, x7 + 0.05, 2.05, y8, FLOOR, CEIL, M['wall'], B)
box('p7c', x7 - 0.05, x7 + 0.05, 0.45, 2.05, FLOOR, 1.67, M['wall'], B)
box('p7d', x7 - 0.05, x7 + 0.05, 0.45, 2.05, 2.75, CEIL, M['wall'], B)
box('ledge', x7 - 0.16, x7 + 0.16, 0.45, 2.05, 1.67, 1.71, M['white_furn'], B)
box('p19', WB - 0.05, WB + 0.05, T, y8, FLOOR, CEIL, M['wall'], B)
box('p26', 26 * FT - 0.05, 26 * FT + 0.05, T, y8, FLOOR, CEIL, M['wall'], B)
x29 = 29 * FT
iwall(x29 - 0.05, x29 + 0.05, T, DF - T, gaps=[(3 * FT, 6 * FT, 2.75), (8.5 * FT, 11.5 * FT, 2.75)], axis='y', name='p29_')
box('p6', x29, L - T, y6 - 0.05, y6 + 0.05, FLOOR, CEIL, M['wall'], B)
box('nurse_back', WB, x29, DF - T - 0.6, DF - T, FLOOR, 1.55, M['white_furn'], B)

def chair(x, y, facing, name):
    """Waiting-room chair; facing = direction the sitter looks, in degrees."""
    e = bpy.data.objects.new(name, None); link(e, B)
    e.location = (x, y, FLOOR); e.rotation_euler = (0, 0, math.radians(facing - 90))
    s = box(name + 's', -0.25, 0.25, -0.24, 0.24, 0.40, 0.47, M['fabric'], e)
    bk = box(name + 'b', -0.25, 0.25, -0.30, -0.24, 0.47, 0.88, M['fabric'], e)
    for lx in (-0.22, 0.22):
        for ly in (-0.26, 0.21):
            box(name + 'l', lx - 0.012, lx + 0.012, ly - 0.012, ly + 0.012, 0.0, 0.40, M['rail'], e)
    return e

for i, x in enumerate([2.62, 3.20, 3.78]):
    chair(x, y8 - 0.35, -90, f'cb{i}')
for i, y in enumerate([0.95, 1.52]):
    chair(WB - 0.37, y, 180, f'cr{i}')
for i, x in enumerate([2.95, 3.50]):
    chair(x, T + 0.38, 90, f'cf{i}')
box('table', 4.05, 4.45, y8 - 0.55, y8 - 0.15, FLOOR, FLOOR + 0.48, M['wood_int'], B)
box('art_frame', 2.55, 3.55, y8 - 0.07, y8 - 0.05, 1.45, 2.10, M['rail'], B)
box('art', 2.58, 3.52, y8 - 0.08, y8 - 0.07, 1.48, 2.07, M['art'], B)
# reception: upper cabinets and a clock above the frosted line
box('rcab', T, T + 0.35, 0.4, 2.0, 2.15, 2.85, M['white_furn'], B)
box('rdesk', 1.55, x7 - 0.06, 0.45, 2.05, FLOOR, FLOOR + 0.76, M['wood_int'], B)
# provider office shelving and desk
box('oshelf1', 9.2, 10.9, y6 - 0.33, y6 - 0.05, 2.02, 2.05, M['wood_int'], B)
box('oshelf2', 9.2, 10.9, y6 - 0.33, y6 - 0.05, 2.45, 2.48, M['wood_int'], B)
for i in range(14):
    bx = 9.25 + i * 0.11 + random.uniform(-0.01, 0.01)
    h = random.uniform(0.22, 0.32)
    c = random.choice(['#3d4f5c', '#7a5c3e', '#c9c2b4', '#2f4a3a', '#8a8f94', '#a14e3a'])
    box(f'book{i}', bx, bx + random.uniform(0.04, 0.08), y6 - 0.28, y6 - 0.08, 2.05, 2.05 + h,
        simple(f'bk{i}', srgb(c), 0.7), B)
box('odesk', 10.85, L - T - 0.02, 0.3, 1.55, FLOOR + 0.72, FLOOR + 0.76, M['wood_int'], B)
box('e3cab', 9.2, 11.2, DF - T - 0.36, DF - T, 2.10, 2.80, M['white_furn'], B)

# ceiling downlights: visible trim discs plus small area lamps
def downlight(x, y, power=11.0):
    bpy.ops.mesh.primitive_cylinder_add(vertices=24, radius=0.075, depth=0.01, location=(x, y, CEIL - 0.004))
    d = bpy.context.active_object; d.parent = B; d.data.materials.append(M['emit'])
    d.visible_shadow = False
    lt = bpy.data.lights.new('dl', 'AREA'); lt.shape = 'DISK'; lt.size = 0.14; lt.energy = power
    lt.color = (1.0, 0.82, 0.62)
    lo = bpy.data.objects.new('dl', lt); link(lo, B); lo.location = (x, y, CEIL - 0.02)
for x, y in [(2.85, 0.75), (4.95, 0.75), (2.85, 1.85), (4.95, 1.85), (1.05, 0.8), (1.05, 1.85),
             (9.6, 0.9), (10.8, 0.9), (10.25, 3.2), (1.6, 3.05), (4.4, 3.05), (7.2, 3.05), (10.25, 2.2)]:
    downlight(x, y)

# ---------------------------------------------------------------- site
door_c = (3.81 + 4.72) / 2
LAND = dict(x0=3.05, x1=5.55, y0=-1.75, y1=0.06)
DECK_TOP = FLOOR + 0.02
box('landing', LAND['x0'], LAND['x1'], LAND['y0'], LAND['y1'], 0.0, DECK_TOP, M['deck'], B, side_mat=M['fascia'])
# 1:12 ramp straight out to the walk
RW = 1.45
rx0, rx1 = door_c - RW / 2, door_c + RW / 2
rlen = DECK_TOP * 12
ry_top, ry_bot = LAND['y0'], LAND['y0'] - rlen
mesh_obj('ramp', [(rx0, ry_top, 0), (rx1, ry_top, 0), (rx1, ry_bot, 0), (rx0, ry_bot, 0),
                  (rx0, ry_top, DECK_TOP), (rx1, ry_top, DECK_TOP), (rx1, ry_bot, 0.012), (rx0, ry_bot, 0.012)],
         [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)], [M['deck'], M['fascia']],
         face_mats=[1, 0, 1, 1, 1, 1], parent=B)
# steps off the east side of the landing
for i, (sx, top) in enumerate([(LAND['x1'], DECK_TOP * 2 / 3), (LAND['x1'] + 0.30, DECK_TOP / 3)]):
    if True:
        box(f'step{i}', sx, sx + 0.30, -1.55, -0.05, 0.0, top, M['deck'], B, side_mat=M['fascia'])

# black steel guards and handrails
def rail_run(p0, p1, h=0.92, posts=True, name='rail'):
    p0, p1 = Vector(p0), Vector(p1)
    up = Vector((0, 0, h))
    segment(name + 'top', p0 + up, p1 + up, 0.042, M['rail'], B, round_=True)
    segment(name + 'mid', p0 + up * 0.55, p1 + up * 0.55, 0.03, M['rail'], B, round_=True)
    if posts:
        n_ = max(1, round((p1 - p0).length / 1.5))
        for i in range(n_ + 1):
            p = p0.lerp(p1, i / n_)
            segment(name + 'post', p, p + up, 0.045, M['rail'], B)
for side, x in (('w', rx0 + 0.04), ('e', rx1 - 0.04)):
    rail_run((x, ry_bot + 0.25, 0.012 + 0.25 / 12), (x, ry_top, DECK_TOP), name='ramp' + side)
rail_run((rx0 + 0.04, ry_top, DECK_TOP), (LAND['x0'] + 0.04, ry_top, DECK_TOP), name='landf')
rail_run((LAND['x0'] + 0.04, ry_top, DECK_TOP), (LAND['x0'] + 0.04, -0.12, DECK_TOP), name='landw')

# concrete walk from the bottom of the ramp toward the street
box('walk', rx0 - 0.1, rx1 + 0.1, -40.0, ry_bot + 0.02, -0.02, 0.022, M['path'], B)
# gravel drip strip around the building
prism('gravel', l_poly(-0.62), -0.02, 0.028, [M['gravel']], parent=B)

# mini-split condenser on the east end (ductless HVAC ships standard)
box('pad', L + 0.08, L + 0.62, 3.0, 4.05, 0.0, 0.09, M['path'], B)
cond = box('condenser', L + 0.14, L + 0.46, 3.08, 3.98, 0.09, 0.71, M['white_furn'], B)
bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=0.23, depth=0.01,
                                    location=(L + 0.465, 3.33, 0.41), rotation=(0, math.radians(90), 0))
g = bpy.context.active_object; g.parent = B; g.data.materials.append(M['grille'])
box('lineset', L + 0.0, L + 0.08, 3.55, 3.65, 0.71, 2.35, M['white_furn'], B)

# planters flanking the landing
def planter(x, y, r=0.27, h=0.66):
    bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=r, depth=h, location=(x, y, h / 2))
    p = bpy.context.active_object; p.parent = B; p.data.materials.append(M['planter'])
    bv_ = p.modifiers.new('b', 'BEVEL'); bv_.width = 0.012; bv_.segments = 3; bv_.limit_method = 'ANGLE'
    bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=r - 0.025, depth=0.02, location=(x, y, h - 0.04))
    s = bpy.context.active_object; s.parent = B; s.data.materials.append(M['soil'])
    return (x, y, h - 0.04)
pl1 = planter(LAND['x0'] - 0.55, -0.55)
pl2 = planter(LAND['x1'] + 1.30, -0.55)

# ---------------------------------------------------------------- vegetation
assets_col = bpy.data.collections.new('assets'); scene.collection.children.link(assets_col)
vl_assets = bpy.context.view_layer.layer_collection.children['assets']

def load_asset(name):
    path = os.path.join(A, 'models', name, f'{name}.gltf')
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=path)
    new = [o for o in bpy.data.objects if o not in before]
    col = bpy.data.collections.new(name)
    assets_col.children.link(col)
    for o in new:
        for c in o.users_collection:
            c.objects.unlink(o)
        col.objects.link(o)
    return col, new

def place(col, loc, rot=None, scale=1.0, parent=B):
    e = bpy.data.objects.new(col.name + '_i', None)
    e.instance_type = 'COLLECTION'; e.instance_collection = col
    link(e, parent)
    e.location = loc
    e.rotation_euler = (0, 0, rot if rot is not None else random.uniform(0, 6.283))
    e.scale = (scale, scale, scale)
    return e

if not args.nogreen:
    def split(name):
        """Import a multi-variant model and return one collection per root object."""
        col, objs = load_asset(name)
        roots = [o for o in objs if o.parent is None]
        out = []
        for r_ in roots:
            sub = bpy.data.collections.new(r_.name)
            col.children.link(sub)
            stack = [r_]
            while stack:
                o = stack.pop()
                for c in list(o.users_collection):
                    c.objects.unlink(o)
                sub.objects.link(o)
                stack.extend(o.children)
            r_.location = (0, 0, 0)
            out.append(sub)
        return out
    grass = split('grass_medium_01') + split('grass_medium_02')
    vl_assets.exclude = True

    # ornamental grass in the planters and in small drifts at the corners
    orn = [c for c in grass if c.name.startswith('grass_medium_02_d') or c.name.startswith('grass_medium_02_e')]
    place(orn[-1], pl1, scale=2.6); place(orn[-1], pl2, scale=2.6)
    for (x, y, s_) in [(L + 0.85, -0.85, 2.4), (L + 1.25, -0.35, 2.0), (-0.75, -0.8, 2.3), (-0.35, -1.15, 1.9)]:
        place(random.choice(orn), (x, y, 0), scale=s_)

# ---------------------------------------------------------------- lawn edge (hair grass)
def grass_mat():
    m, n_, l_, b_ = new_mat('lawn')
    hi = n_.new('ShaderNodeHairInfo')
    mr = n_.new('ShaderNodeMapRange'); l_.new(hi.outputs['Random'], mr.inputs['Value'])
    mix = n_.new('ShaderNodeMix'); mix.data_type = 'RGBA'
    l_.new(mr.outputs['Result'], mix.inputs['Factor'])
    mix.inputs['A'].default_value = (0.095, 0.118, 0.016, 1)
    mix.inputs['B'].default_value = (0.235, 0.245, 0.040, 1)
    root = n_.new('ShaderNodeMix'); root.data_type = 'RGBA'; root.blend_type = 'MULTIPLY'
    ramp = n_.new('ShaderNodeMapRange'); ramp.inputs['From Max'].default_value = 0.6
    ramp.inputs['To Min'].default_value = 0.55; ramp.inputs['To Max'].default_value = 1.0
    l_.new(hi.outputs['Intercept'], ramp.inputs['Value'])
    comb = n_.new('ShaderNodeCombineColor')
    for ch in ('Red', 'Green', 'Blue'):
        l_.new(ramp.outputs['Result'], comb.inputs[ch])
    root.inputs['Factor'].default_value = 1.0
    l_.new(mix.outputs['Result'], root.inputs['A']); l_.new(comb.outputs['Color'], root.inputs['B'])
    l_.new(root.outputs['Result'], b_.inputs['Base Color'])
    b_.inputs['Roughness'].default_value = 0.55
    tl = n_.new('ShaderNodeBsdfTranslucent'); l_.new(root.outputs['Result'], tl.inputs['Color'])
    ms = n_.new('ShaderNodeMixShader'); ms.inputs['Fac'].default_value = 0.3
    l_.new(b_.outputs['BSDF'], ms.inputs[1]); l_.new(tl.outputs['BSDF'], ms.inputs[2])
    l_.new(ms.outputs['Shader'], n_['Material Output'].inputs['Surface'])
    return m
M['lawn'] = grass_mat()

def ring_faces(inner, outer, z=0.0):
    n_ = len(inner)
    v = [(x, y, z) for x, y in inner] + [(x, y, z) for x, y in outer]
    return v, [(i, (i + 1) % n_, n_ + (i + 1) % n_, n_ + i) for i in range(n_)]

def band(p0, p1, w, z=0.0):
    """Quad strip of width w on the right-hand side of p0->p1."""
    p0, p1 = Vector(p0), Vector(p1)
    d = (p1 - p0).normalized(); nrm = Vector((d.y, -d.x)) * w
    return [(p0.x, p0.y, z), (p1.x, p1.y, z), (p1.x + nrm.x, p1.y + nrm.y, z), (p0.x + nrm.x, p0.y + nrm.y, z)]

if not args.nogreen:
    verts, faces = ring_faces(l_poly(-0.60), l_poly(-0.98), 0.0)
    for p0, p1, w in [((rx0, LAND['y0'] - 0.62), (rx0, ry_bot - 12.0), 0.32),
                      ((rx1, ry_bot - 12.0), (rx1, LAND['y0'] - 0.62), 0.32),
                      ((LAND['x0'], -0.62), (LAND['x0'], LAND['y0']), 0.3),
                      ((LAND['x0'], LAND['y0']), (rx0, LAND['y0']), 0.3),
                      ((rx1, LAND['y0']), (LAND['x1'] + 0.6, LAND['y0']), 0.3),
                      ((LAND['x1'] + 0.6, LAND['y0']), (LAND['x1'] + 0.6, -0.62), 0.3)]:
        k = len(verts); verts += band(p0, p1, w); faces.append((k, k + 1, k + 2, k + 3))
    lawn = mesh_obj('lawn_edge', verts, faces, [M['lawn']], parent=B)
    fix_normals(lawn)
    for poly in lawn.data.polygons:
        if poly.normal.z < 0:
            poly.flip()
    area = sum(pp.area for pp in lawn.data.polygons)
    lawn.modifiers.new('grass', 'PARTICLE_SYSTEM')
    ps = lawn.particle_systems[0].settings
    ps.type = 'HAIR'; ps.use_advanced_hair = True
    ps.count = int(area * 650)
    ps.factor_random = 0.06
    ps.child_type = 'SIMPLE'; ps.rendered_child_count = 14; ps.child_percent = 2
    ps.child_radius = 0.05; ps.child_length = 0.6; ps.child_length_threshold = 0.3
    ps.roughness_1 = 0.012; ps.roughness_endpoint = 0.035; ps.roughness_end_shape = 1.0
    ps.radius_scale = 0.0016; ps.root_radius = 1.0; ps.tip_radius = 0.0
    ps.material_slot = 'lawn'
    ps.display_step = 3; ps.render_step = 3
    ps.hair_length = 0.11

# ---------------------------------------------------------------- placement in the world
yaw = math.radians(args.yaw)
centre = Vector((args.dist * math.cos(yaw), args.dist * math.sin(yaw), 0))
B.rotation_euler = (0, 0, math.radians(args.bphi))
B.location = centre - Matrix.Rotation(math.radians(args.bphi), 3, 'Z') @ Vector((L / 2, DF / 2, 0))

# shadow catcher meadow
bpy.ops.mesh.primitive_plane_add(size=400, location=(0, 0, 0))
gp = bpy.context.active_object; gp.name = 'catcher'; gp.is_shadow_catcher = True
gm = simple('meadow_bounce', (0.10, 0.16, 0.05), 0.95)
gp.data.materials.append(gm)

# camera
cam_d = bpy.data.cameras.new('cam'); cam_d.lens = args.lens; cam_d.sensor_width = 36
cam_d.shift_y = args.shift
cam = bpy.data.objects.new('cam', cam_d); link(cam)
cam.location = (0, 0, 1.70)
cam.rotation_euler = (math.radians(90), 0, yaw - math.radians(90))
scene.camera = cam

# world
world = bpy.data.worlds.new('world'); scene.world = world; world.use_nodes = True
wn, wl = world.node_tree.nodes, world.node_tree.links
for x in list(wn):
    wn.remove(x)
env = wn.new('ShaderNodeTexEnvironment')
env.image = bpy.data.images.load(os.path.join(A, 'hdri', f'lilienstein_{args.hdri}.exr'))
bg = wn.new('ShaderNodeBackground'); out = wn.new('ShaderNodeOutputWorld')
wl.new(env.outputs['Color'], bg.inputs['Color']); wl.new(bg.outputs['Background'], out.inputs['Surface'])
try:
    world.cycles.sampling_method = 'MANUAL'
    world.cycles.sample_map_resolution = 4096
except Exception as e:
    print('world sampling', e)

# render settings
scene.render.engine = 'CYCLES'
cy = scene.cycles
cy.device = 'CPU'
cy.samples = args.samples
cy.use_adaptive_sampling = True
cy.adaptive_threshold = 0.015
cy.use_denoising = True
try:
    cy.denoiser = 'OPENIMAGEDENOISE'
except Exception:
    pass
cy.max_bounces = 8; cy.glossy_bounces = 4; cy.transmission_bounces = 8; cy.transparent_max_bounces = 16
cy.caustics_reflective = False; cy.caustics_refractive = False
cy.blur_glossy = 1.0
scene.cycles_curves.shape = 'RIBBONS'
cy.sample_clamp_indirect = 8.0
scene.render.film_transparent = False
scene.render.resolution_x = args.w
scene.render.resolution_y = round(args.w * 520 / 960)
scene.render.resolution_percentage = 100
vs = scene.view_settings
vs.view_transform = 'AgX'
for look in ('AgX - Medium High Contrast', 'Medium High Contrast', 'AgX - Base Contrast'):
    try:
        vs.look = look; break
    except Exception:
        continue
vs.exposure = 0.15
scene.render.image_settings.file_format = 'PNG'
scene.render.image_settings.color_depth = '16'
scene.render.filepath = args.out
if args.border:
    bx = [float(v) for v in args.border.split(',')]
    scene.render.use_border = True; scene.render.use_crop_to_border = True
    scene.render.border_min_x, scene.render.border_max_x, scene.render.border_min_y, scene.render.border_max_y = bx
if args.blend:
    bpy.ops.wm.save_as_mainfile(filepath=args.blend)
bpy.ops.render.render(write_still=True)
print('WROTE', args.out)
