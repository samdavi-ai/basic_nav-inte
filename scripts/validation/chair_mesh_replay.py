"""Replay native Gazebo XY poses against the Fuel chair collision mesh projection."""
import argparse
import ctypes as ct
from ctypes.util import find_library
import gzip
import hashlib
import json
import io
import math
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from analyze_routing import HALF, SIGNS, WORLD


def polygon(points):
    ring = [*points, points[0]]
    return '((' + ','.join(f'{x:.14g} {y:.14g}' for x, y in ring) + '))'


def chair_projection(model, exporter):
    link = ET.parse(model).find('model/link')
    mesh = link.find('collision/geometry/mesh')
    obj = model.parent / 'meshes' / mesh.findtext('uri').split('/')[-1]
    scale = np.fromstring(mesh.findtext('scale'), sep=' ')
    pose = np.fromstring(link.findtext('pose'), sep=' ')
    if any(pose[:5]) or link.find('collision/pose') is not None:
        raise ValueError('Unsupported chair link or collision transform')
    output = subprocess.check_output([exporter, obj], text=True)
    vertices = (np.loadtxt(io.StringIO(output))*scale)[:, :2]
    faces = np.arange(len(vertices)).reshape(-1, 3)
    return vertices, faces, pose[5], obj


class Geometry:
    def __init__(self):
        self.lib = ct.CDLL(find_library('geos_c'))
        ptr, number = ct.c_void_p, ct.POINTER(ct.c_double)
        for name, result, args in [
            ('initGEOS', None, [ptr, ptr]), ('GEOSGeomFromWKT', ptr, [ct.c_char_p]),
            ('GEOSUnaryUnion', ptr, [ptr]), ('GEOSDistance', ct.c_int, [ptr, ptr, number]),
            ('GEOSArea', ct.c_int, [ptr, number]), ('GEOSIntersection', ptr, [ptr, ptr]),
            ('GEOSGeom_destroy', None, [ptr])]:
            fn = getattr(self.lib, name)
            fn.restype, fn.argtypes = result, args
        self.lib.initGEOS(None, None)

    def read(self, wkt):
        geom = self.lib.GEOSGeomFromWKT(wkt.encode())
        if not geom:
            raise ValueError('Invalid input geometry')
        return geom


def replay(samples, model, exporter):
    vertices, faces, link_yaw, obj = chair_projection(model, exporter)
    geos = Geometry()
    lib = geos.lib
    results = {}
    for inc in ET.parse(WORLD).findall('.//world/include'):
        if inc.findtext('uri') != 'model://chair':
            continue
        x, y, z, r, p, heading = map(float, inc.findtext('pose').split())
        if r or p or z:
            raise ValueError('Unsupported chair placement')
        c, s = math.cos(heading+link_yaw), math.sin(heading+link_yaw)
        placed = vertices @ [[c,s],[-s,c]] + [x,y]
        projections = []
        for face in faces:
            if len(face) != 3:
                raise ValueError('Expected triangular collision mesh')
            a,b,d = placed[face]
            if abs((b[0]-a[0])*(d[1]-a[1])-(b[1]-a[1])*(d[0]-a[0])) > 1e-15:
                projections.append('POLYGON ' + polygon([a,b,d]))
            else:
                endpoints = np.unique([a,b,d], axis=0)
                if len(endpoints) == 1:
                    projections.append('POINT (%.14g %.14g)' % tuple(endpoints[0]))
                else:
                    projections.append('LINESTRING (' + ','.join(
                        '%.14g %.14g' % tuple(point) for point in endpoints) + ')')
        collection = geos.read('GEOMETRYCOLLECTION ('+','.join(projections)+')')
        chair = lib.GEOSUnaryUnion(collection)
        if not chair:
            raise ValueError('Invalid projected chair')
        lib.GEOSGeom_destroy(collection)
        radius = np.linalg.norm(placed-[x,y],axis=1).max() + np.linalg.norm(HALF)
        lower = np.linalg.norm(samples[:,1:3]-[x,y],axis=1)-radius
        best, overlap = float('inf'), 0.
        overlap_samples = checked = 0
        witness = closest = None
        for i in np.argsort(lower):
            if lower[i] > best:
                break
            t, px, py, yaw = samples[i]
            c, s = math.cos(yaw), math.sin(yaw)
            body = geos.read('POLYGON '+polygon((SIGNS*HALF) @ [[c,s],[-s,c]] + [px,py]))
            distance = ct.c_double()
            if not lib.GEOSDistance(body,chair,ct.byref(distance)):
                raise ValueError('Mesh distance failed')
            checked += 1
            if distance.value < best:
                best, closest = distance.value, samples[i].tolist()
            if distance.value == 0:
                intersection = lib.GEOSIntersection(body,chair)
                area = ct.c_double()
                if not intersection or not lib.GEOSArea(intersection,ct.byref(area)):
                    raise ValueError('Intersection failed')
                lib.GEOSGeom_destroy(intersection)
                overlap_samples += int(area.value > 0)
                if area.value > overlap:
                    overlap, witness = area.value, samples[i].tolist()
            lib.GEOSGeom_destroy(body)
        lib.GEOSGeom_destroy(chair)
        results[inc.findtext('name')] = dict(min_distance_m=best, closest=closest,
            overlapping_samples=overlap_samples, max_overlap_m2=overlap,
            max_overlap_pose=witness, exact_queries=checked)
    return dict(native_samples=len(samples), mesh_sha256=hashlib.sha256(obj.read_bytes()).hexdigest(),
                footprint_m=(HALF*2).tolist(), chairs=results)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('truth',type=Path)
    parser.add_argument('model',type=Path)
    parser.add_argument('exporter',type=Path)
    args = parser.parse_args()
    with gzip.open(args.truth,'rt') as stream:
        print(json.dumps(replay(np.array([json.loads(line) for line in stream]),args.model,args.exporter),indent=2))
