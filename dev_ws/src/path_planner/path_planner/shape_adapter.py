from math import sin, cos, pi
from dataclasses import dataclass, field, astuple
import math
from typing import Union, Optional
from collections.abc import Sequence, Iterable
import numpy as np
from shapely.geometry import Point, LineString, Polygon, MultiPoint, MultiLineString, MultiPolygon


@dataclass
class ShapeAdapter():    
    items: MultiPolygon
    buffer: float = 0.25

    def intersects(self, shape: LineString|Point|tuple) -> bool:
        if isinstance(shape, LineString):
            multi_line = MultiLineString([
                shape.parallel_offset(self.buffer, side="left"), 
                shape, 
                shape.parallel_offset(-self.buffer, side="left")
            ])
            if self.items.intersects(multi_line):
                return True        
            return False
        elif isinstance(shape, Point|tuple):
            if isinstance(shape, tuple):
                shape = Point(shape)
            multi_point = MultiPoint([
                (shape.x + self.buffer, shape.y + self.buffer),
                (shape.x - self.buffer, shape.y + self.buffer),
                (shape.x - self.buffer, shape.y - self.buffer),
                (shape.x + self.buffer, shape.y - self.buffer),
            ])
            if self.items.intersects(multi_point):
                return True
            return False
        else:
            raise Exception("A wrong object type is checked for intersection!")
        
    def find_colliders(self, path: Sequence[Point|tuple[float,float]]):
        colliders = []
        for i in range(len(path) - 1):
            start, end = path[i], path[i+1]
            line = LineString([start, end])
            for item in self.items.geoms:
                if item.intersects(line) and item not in colliders:
                    colliders.append(item)
        return colliders


@dataclass
class Rectangle():
    x: float
    y: float
    rot_z: float
    length: float
    width: float
    grasp_dist: float = 0.55
    polygon: Polygon|None = None  
    corners: Sequence[Point] = field(default_factory=list)
    grasp_points: Sequence[Point] = field(default_factory=list)
    
    def __post_init__(self):
        half_width = self.width / 2
        half_length = self.length / 2
        
        pbl = self.rotate([self.x - half_width, self.y - half_length])   # bottom left
        pbr = self.rotate([self.x + half_width, self.y - half_length])   # bottom right
        ptr = self.rotate([self.x + half_width, self.y + half_length])   # top right
        ptl = self.rotate([self.x - half_width, self.y + half_length])   # top left

        self.corners = [Point(pbl), Point(pbr), Point(ptr), Point(ptl)]
        self.polygon = Polygon([pbl, pbr, ptr, ptl, pbl])
        
        n_pbl = np.asarray(pbl)
        n_pbr = np.asarray(pbr)
        n_ptr = np.asarray(ptr)
        n_ptl = np.asarray(ptl)
        
        hor_offset = n_pbr - n_pbl  # type: ignore
        hor_offset = hor_offset / np.linalg.norm(hor_offset) * self.grasp_dist
        ver_offset = n_ptl - n_pbl  # type: ignore
        ver_offset = ver_offset / np.linalg.norm(ver_offset) * self.grasp_dist

        n_pblxbr = (n_pbl + n_pbr) / 2 - ver_offset
        n_pbrxtr = (n_pbr + n_ptr) / 2 + hor_offset
        n_ptlxtr = (n_ptl + n_ptr) / 2 + ver_offset
        n_pblxtl = (n_pbl + n_ptl) / 2 - hor_offset
        
        self.grasp_points = [
            Point(*n_pblxbr.tolist()),
            Point(*n_pbrxtr.tolist()),
            Point(*n_ptlxtr.tolist()),
            Point(*n_pblxtl.tolist())
        ]
        print(type(self.grasp_points), self.grasp_points)
        
        print(f"\n\n\nTHE BOX: ({self.x}, {self.y}) WITH ROTATION: {self.rot_z}".center(50, "*"))
        for i, p in enumerate(self.corners):
            print(f"Corner {i}: ".ljust(20) + f"({p.x}, {p.y})")
            
        for i, p in enumerate(self.grasp_points):
            print(f"Grasping point {i}: ".ljust(20) + f"{p}")
    
    def rotate(self, point):
        px, py = point
        new_x = self.x + math.cos(self.rot_z) * (px - self.x) - math.sin(self.rot_z) * (py - self.y)
        new_y = self.y + math.sin(self.rot_z) * (px - self.x) + math.cos(self.rot_z) * (py - self.y)
        return new_x, new_y
        

@dataclass
class RectangleFromBottomLeft():
    x: float
    y: float
    rot_z: float
    length: float
    width: float
    offset_mag: float = 0.3
    polygon: Polygon = None  # type: ignore
    corners: Sequence[Point] = field(default_factory=list)
    grasp_points: Sequence[Point] = field(default_factory=list)
    
    
    def __post_init__(self):
        pbl = [self.x, self.y]
        pbr = [self.x + cos(self.rot_z)*self.length, self.y + sin(self.rot_z)*self.length]
        ptr = [self.x + cos(self.rot_z)*self.length - sin(self.rot_z)*self.width, self.y + sin(self.rot_z)*self.length + cos(self.rot_z)*self.width]
        ptl = [self.x - sin(self.rot_z)*self.width, self.y + cos(self.rot_z)*self.width]

        self.corners = [Point(pbl), Point(pbr), Point(ptr), Point(ptl)]
        self.polygon = Polygon([pbl, pbr, ptr, ptl, pbl])
        
        n_pbl = np.asarray(pbl)
        n_pbr = np.asarray(pbr)
        n_ptr = np.asarray(ptr)
        n_ptl = np.asarray(ptl)
        
        hor_offset = n_pbr - n_pbl  # type: ignore
        hor_offset = hor_offset / np.linalg.norm(hor_offset) * self.offset_mag
        ver_offset = n_ptl - n_pbl  # type: ignore
        ver_offset = ver_offset / np.linalg.norm(ver_offset) * self.offset_mag
        
        n_pblxbr = (n_pbl + n_pbr) / 2 - ver_offset
        n_pbrxtr = (n_pbr + n_ptr) / 2 + hor_offset
        n_ptlxtr = (n_ptl + n_ptr) / 2 + ver_offset
        n_pblxtl = (n_pbl + n_ptl) / 2 - hor_offset
        
        self.grasp_points = [
            Point(*n_pblxbr.tolist()),
            Point(*n_pbrxtr.tolist()),
            Point(*n_ptlxtr.tolist()),
            Point(*n_pblxtl.tolist())
        ]
        
        print(type(self.grasp_points), self.grasp_points)
        
        print(f"THE BOX: ({self.x}, {self.y}) WITH ROT: {self.rot_z}".center(50, "*"))
        for i, p in enumerate(self.corners):
            print(f"Corner {i}: ".ljust(20) + f"({p.x}, {p.y})")
            
        for i, p in enumerate(self.grasp_points):
            print(f"Grasping point {i}: ".ljust(20) + f"{p}")

    
def main():
    unit_box_clone = Rectangle(6.74006, 9.05942, 1, 0.3, 0.3)
    unit_box_clone_0 = Rectangle(5.26531, 9.33454, -0.2, 0.3, 0.3)
    unit_box_clone_1 = Rectangle(7.01425, 4.50425, 1, 0.3, 0.3)
    unit_box_clone_2 = Rectangle(8.82217, 3.39774, 0, 0.3, 0.3)
    unit_box_clone_3 = Rectangle(6.88396, 5.77537, 1, 0.3, 0.3)
    unit_box_clone_5 = Rectangle(13.26190, 8.09082, 0, 0.3, 0.3)
    unit_box_clone_7 = Rectangle(2.54692, 9.28152, 0, 0.3, 0.15)
   
if "__main__" == __name__:
    main()