"""Checks STEP files the way CAD programs read them, with OpenCascade.

    python check_step.py file.step [expected_volume ...]

For each file: reads it, then for each solid reports whether OpenCascade
finds it valid (BRepCheck_Analyzer), whether its shell is closed (no free
edges), and its volume. With expected volumes (one per solid, in order),
also the difference. Exits 1 if anything is wrong.

Needs OCP (pip install cadquery-ocp).
"""
import sys

from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS


def count(shape, kind):
    e = TopExp_Explorer(shape, kind)
    n = 0
    while e.More():
        n += 1
        e.Next()
    return n


def main():
    path, expected = sys.argv[1], [float(v) for v in sys.argv[2:]]
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != IFSelect_RetDone:
        print(f"{path}: can't read")
        return 1
    reader.TransferRoots()
    shape = reader.OneShape()
    ok = True
    e = TopExp_Explorer(shape, TopAbs_SOLID)
    i = 0
    while e.More():
        solid = TopoDS.Solid_s(e.Current())
        valid = BRepCheck_Analyzer(solid).IsValid()
        free = ShapeAnalysis_FreeBounds(solid)
        open_edges = count(free.GetClosedWires(), TopAbs_EDGE) + count(free.GetOpenWires(), TopAbs_EDGE)
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(solid, props)
        vol = props.Mass()
        line = f"solid {i}: {count(solid, TopAbs_FACE)} faces, valid {valid}, free edges {open_edges}, volume {vol:.4f}"
        if i < len(expected):
            line += f" (expected {expected[i]:.4f}, {100 * (vol - expected[i]) / expected[i]:+.4f}%)"
        print(line)
        ok = ok and valid and open_edges == 0 and vol > 0
        i += 1
        e.Next()
    if i == 0:
        print("no solids")
        ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
