"""STEP helpers for the AHAB model tools: named solids, bounding box/volume, meshing, and a minimal GLB writer."""
import json
import struct
import sys

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS


def solids(path):
    r = STEPControl_Reader()

    if r.ReadFile(path) != IFSelect_RetDone:
        sys.exit('cannot read ' + path)

    r.TransferRoots()
    shape = r.OneShape()
    tr = r.WS().TransferReader()
    out = []
    ex = TopExp_Explorer(shape, TopAbs_SOLID)

    while ex.More():
        s = TopoDS.Solid_s(ex.Current())
        ent = tr.EntityFromShapeResult(s, 1)
        name = ent.Name().ToCString() if ent is not None and ent.Name() is not None else '?'
        out.append((name, s))
        ex.Next()

    return out


def info(s):
    b = Bnd_Box()
    BRepBndLib.Add_s(s, b)
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(s, g)
    c = g.CentreOfMass()
    return b.Get(), g.Mass(), (c.X(), c.Y(), c.Z())


def mesh(s, deflection=0.2):
    BRepMesh_IncrementalMesh(s, deflection, False, 0.3, True)
    V, F = [], []
    ex = TopExp_Explorer(s, TopAbs_FACE)

    while ex.More():
        face = TopoDS.Face_s(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)

        if tri is not None:
            T = loc.Transformation()
            base = len(V)

            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(T)
                V.append((p.X(), p.Y(), p.Z()))

            rev = face.Orientation() == TopAbs_REVERSED

            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                F.append((base + a - 1, base + c - 1, base + b - 1) if rev else (base + a - 1, base + b - 1, base + c - 1))

        ex.Next()

    return np.array(V, dtype=np.float64), np.array(F, dtype=np.int64)


def normals(V, F):
    n = np.zeros_like(V)
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])

    for k in range(3):
        np.add.at(n, F[:, k], fn)

    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln == 0] = 1
    return n / ln


def write_glb(path, V, F, name, rgba):
    V = V.astype('<f4')
    N = normals(V.astype(np.float64), F).astype('<f4')
    I = F.astype('<u4').ravel()
    bv = [V.tobytes(), N.tobytes(), I.tobytes()]
    offs, blob = [], b''

    for b in bv:
        offs.append(len(blob))
        blob += b + b'\0' * ((4 - len(b) % 4) % 4)

    gltf = {
        'asset': {'version': '2.0', 'generator': 'ahab step_to_glb'},
        'scene': 0, 'scenes': [{'nodes': [0]}], 'nodes': [{'mesh': 0, 'name': name}],
        'materials': [{'name': name, 'pbrMetallicRoughness': {'baseColorFactor': rgba, 'metallicFactor': 0.0,
                                                               'roughnessFactor': 0.6}}],
        'meshes': [{'name': name, 'primitives': [{'attributes': {'POSITION': 0, 'NORMAL': 1}, 'indices': 2,
                                                  'material': 0}]}],
        'buffers': [{'byteLength': len(blob)}],
        'bufferViews': [{'buffer': 0, 'byteOffset': offs[0], 'byteLength': len(bv[0]), 'target': 34962},
                        {'buffer': 0, 'byteOffset': offs[1], 'byteLength': len(bv[1]), 'target': 34962},
                        {'buffer': 0, 'byteOffset': offs[2], 'byteLength': len(bv[2]), 'target': 34963}],
        'accessors': [{'bufferView': 0, 'componentType': 5126, 'count': len(V), 'type': 'VEC3',
                       'min': V.min(0).tolist(), 'max': V.max(0).tolist()},
                      {'bufferView': 1, 'componentType': 5126, 'count': len(N), 'type': 'VEC3'},
                      {'bufferView': 2, 'componentType': 5125, 'count': len(I), 'type': 'SCALAR'}],
    }
    js = json.dumps(gltf, separators=(',', ':')).encode()
    js += b' ' * ((4 - len(js) % 4) % 4)
    total = 12 + 8 + len(js) + 8 + len(blob)

    with open(path, 'wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, total))
        f.write(struct.pack('<II', len(js), 0x4E4F534A) + js)
        f.write(struct.pack('<II', len(blob), 0x004E4942) + blob)


if __name__ == '__main__':
    for name, s in solids(sys.argv[1]):
        (x0, y0, z0, x1, y1, z1), vol, c = info(s)
        print('%-14s bbox x[%8.1f %8.1f] y[%8.1f %8.1f] z[%8.1f %8.1f] mm  vol %9.0f cm3  centroid (%.1f %.1f %.1f)'
              % (name, x0, x1, y0, y1, z0, z1, vol / 1000, *c))
