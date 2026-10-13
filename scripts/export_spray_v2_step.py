#!/usr/bin/env python3
"""Rebuild rigid spray V2 meshes; requires cadquery-ocp 8.0, trimesh, fast-simplification.

Usage: python export_spray_v2_step.py '/path/to/EOAT ass_y_ROBOI_Sharing_v2 (2).stp'
The CAD is millimetres; output is metres in the robot mating-flange frame.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.collections import Sequence_TDF_Label
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool

EXCLUDE = {'RB20-1900ES_LINK6', 'spray coat', 'hose_1', 'hose_2', 'hose_3'}


def name(label):
    attribute = TDataStd_Name()
    return attribute.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attribute) else ''


def triangulate(shape):
    BRepMesh_IncrementalMesh(shape, 0.7, False, 0.25, True)
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    triangles = []
    while explorer.More():
        face = TopoDS.Face(explorer.Current())
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face, location)
        if mesh:
            vertices = np.array([mesh.Node(i).Transformed(location.Transformation()).Coord()
                                 for i in range(1, mesh.NbNodes()+1)])
            for i in range(1, mesh.NbTriangles()+1):
                triangle = vertices[np.array(mesh.Triangle(i).Get())-1]
                triangles.append(triangle[::-1] if face.Orientation() == TopAbs_REVERSED else triangle)
        explorer.Next()
    return np.asarray(triangles)


def export(source, output):
    document = TDocStd_Document(TCollection_ExtendedString('eoat'))
    reader = STEPCAFControl_Reader()
    reader.ReadFile(str(source))
    if not reader.Transfer(document):
        raise ValueError('STEP transfer failed')
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    roots = Sequence_TDF_Label()
    shapes.GetFreeShapes(roots)
    if roots.Length() != 1:
        raise ValueError('Expected a single EOAT assembly')
    triangles, included = [], []

    def walk(label, parent_location):
        reference = TDF_Label()
        if shapes.IsReference_s(label):
            shapes.GetReferredShape_s(label, reference)
            location = parent_location.Multiplied(shapes.GetLocation_s(label))
        else:
            reference, location = label, parent_location
        part_name = name(reference)
        if part_name in EXCLUDE:
            return
        children = Sequence_TDF_Label()
        if shapes.GetComponents_s(reference, children):
            for i in range(1, children.Length()+1):
                walk(children.Value(i), location)
        else:
            triangles.append(triangulate(shapes.GetShape_s(reference).Moved(location)))
            included.append(part_name)

    walk(roots.Value(1), TopLoc_Location())
    vertices = (np.concatenate(triangles)-[0, -67, 0])[:, :, [2, 0, 1]] / 1000
    mesh = trimesh.Trimesh(vertices=vertices.reshape(-1, 3),
                           faces=np.arange(vertices.size//3).reshape(-1, 3), process=True)
    output.mkdir(parents=True, exist_ok=True)
    metadata_path = output/'mount.json'
    metadata = json.loads(metadata_path.read_text())
    metadata.update(source_file=source.name, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                    included_parts=included, bounds_m=mesh.bounds.tolist(), triangles=len(mesh.faces))
    for kind, count in [('visual', 120000), ('collision', 25000)]:
        reduced = mesh.simplify_quadric_decimation(face_count=count, aggression=5)
        reduced.vertices = reduced.vertices.astype(np.float32).astype(float)
        points = reduced.triangles
        areas = np.linalg.norm(np.cross(points[:, 1]-points[:, 0], points[:, 2]-points[:, 0]), axis=1)
        reduced.update_faces(areas > 1e-16)
        reduced.export(output/f'spray_v2_{kind}.stl')
        metadata[kind+'_triangles'] = len(reduced.faces)
    metadata_path.write_text(json.dumps(metadata, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('step', type=Path)
    args = parser.parse_args()
    export(args.step, Path(__file__).resolve().parents[1]/'assets/construct/spray_v2')
