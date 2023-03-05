from shapely.geometry import Polygon, MultiPolygon, LineString, Point, MultiPoint
from path_planner.shape_adapter import ShapeAdapter, Rectangle

from path_planner.rrt import RRT_star, dijkstra
import datetime


def main(args=None):
    s_polygon = Polygon(
        shell = [
            (-0.1, -0.1),
            (15.1, -0.1),
            (15.1, 10.1),
            (-0.1, 10.1),
            (-0.1, -0.1),
        ],
        holes = [
            [
                (0.1,0.1), 
                (5.9,0.1), 
                (5.9,9), 
                (6.1,9), 
                (6.1,0.1), 
                (14.9,0.1), 
                (14.9,9.9),
                (0.1,9.9),
                (0.1,0.1)
            ]
        ]
    )
    s_env = ShapeAdapter(MultiPolygon([s_polygon]), 0.25)
    
    then = datetime.datetime.now()    
    G = RRT_star((0.7, 0.7), (9.0, 2.0), s_polygon, 10000, 0.25, 1, (15, 10), (0, 0))
    print(f"Duration: {datetime.datetime.now() - then}")
    if G.success:
        path = dijkstra(G)
        for idx, p in enumerate(path):
            print(f"{idx}: {p}")
    else:
        print("FAILURE")
    
    print(type(path), path)     
    d_env = ShapeAdapter(MultiPolygon([Rectangle(6, 9.5, 0, 0.9, 0.9).polygon, s_polygon]))
    colliders = d_env.find_colliders([Point(p) for p in path])
    for c in colliders: print(c.centroid)
    

if __name__ == "__main__":
    main()

