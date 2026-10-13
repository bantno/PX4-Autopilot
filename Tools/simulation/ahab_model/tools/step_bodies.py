"""List the named solids in a STEP file with bounding box (mm) and volume, or mesh them to numpy arrays."""
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


if __name__ == '__main__':
    for name, s in solids(sys.argv[1]):
        (x0, y0, z0, x1, y1, z1), vol, c = info(s)
        print('%-14s bbox x[%8.1f %8.1f] y[%8.1f %8.1f] z[%8.1f %8.1f] mm  vol %9.0f cm3  centroid (%.1f %.1f %.1f)'
              % (name, x0, x1, y0, y1, z0, z1, vol / 1000, *c))
