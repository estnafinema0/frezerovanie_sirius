"""Read-only OCCT inspection. Optional analysis utility, not the Windows CAM runtime.
Install cadquery-ocp==7.9.3.1.1 (pins VTK 9.6.2) in an isolated environment.
Usage: python analyze_step.py file.STEP [file2.STEP ...] > report.json
Never converts trimmed STEP faces into a Bezier heightfield implicitly.
"""
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TColStd import TColStd_SequenceOfAsciiString
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID, TopAbs_REVERSED, TopAbs_WIRE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepTools import BRepTools
from OCP.Bnd import Bnd_Box
from OCP.GeomAbs import GeomAbs_Plane

def count(shape, kind):
    e=TopExp_Explorer(shape,kind); n=0
    while e.More():
        n+=1; e.Next()
    return n

def bbox(shape):
    box=Bnd_Box(); BRepBndLib.AddOptimal_s(shape,box,False,False)
    return list(box.Get())

def inspect(filename):
    file=Path(filename)
    if file.stat().st_size > 50*1024*1024: raise ValueError('Input exceeds 50 MiB')
    reader=STEPControl_Reader()
    if reader.ReadFile(str(file)) != IFSelect_RetDone: raise ValueError('STEP read failed')
    length,angle,solidangle=(TColStd_SequenceOfAsciiString() for _ in range(3))
    reader.FileUnits(length,angle,solidangle)
    reader.SetSystemLengthUnit(1.0) # OCCT system unit: millimetres
    transferred=reader.TransferRoots()
    if transferred<=0: raise ValueError('No STEP roots transferred')
    shape=reader.OneShape(); faces=[]; e=TopExp_Explorer(shape,TopAbs_FACE)
    while e.More():
        face=TopoDS.Face_s(e.Current()); surf=BRepAdaptor_Surface(face,True)
        item={'index':len(faces)+1,'type':str(surf.GetType()).split('.')[-1],
              'orientation':str(face.Orientation()).split('.')[-1],
              'bounds_mm':bbox(face),'wires':count(face,TopAbs_WIRE)}
        outer=BRepTools.OuterWire_s(face)
        wires=TopExp_Explorer(face,TopAbs_WIRE);item['wire_bounds_mm']=[]
        item['wire_roles']=[]
        while wires.More():
            item['wire_bounds_mm'].append(bbox(wires.Current()))
            item['wire_roles'].append('outer' if wires.Current().IsSame(outer) else 'inner')
            wires.Next()
        if surf.GetType()==GeomAbs_Plane:
            direction=surf.Plane().Axis().Direction()
            sign=-1 if face.Orientation()==TopAbs_REVERSED else 1
            item['outward_normal']=[sign*direction.X(),sign*direction.Y(),sign*direction.Z()]
        # A UV midpoint may be in a trim hole; do NOT label it a surface contact.
        faces.append(item);e.Next()
    bounds=bbox(shape)
    return {'file':file.name,'sha256':hashlib.sha256(file.read_bytes()).hexdigest(),
            'reader':'OCCT via cadquery-ocp 7.9.3.1.1','source_length_units':[length.Value(i).ToCString() for i in range(1,length.Length()+1)],
            'output_units':'mm','transferred_roots':transferred,'valid_brep':BRepCheck_Analyzer(shape).IsValid(),
            'solids':count(shape,TopAbs_SOLID),'face_count':len(faces),'surface_types':dict(Counter(f['type'] for f in faces)),
            'bounds_mm':bounds,'size_mm':[bounds[i+3]-bounds[i] for i in range(3)],'faces':faces,
            'cam_eligible':False,'reason':'STEP/B-rep is not the supported .bpatch domain; trimming, adjacent faces, stock and tool assembly require separate validation.'}

if __name__=='__main__':
    if len(sys.argv)<2: raise SystemExit('Usage: analyze_step.py file.STEP [file2.STEP ...]')
    print(json.dumps([inspect(p) for p in sys.argv[1:]],indent=2,ensure_ascii=False))
