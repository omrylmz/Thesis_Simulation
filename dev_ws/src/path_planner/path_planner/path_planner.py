from datetime import datetime
import json
import time
from typing import Sequence
from shapely.geometry import Point, LineString, Polygon, MultiPoint, MultiLineString, MultiPolygon
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from json_interfaces.srv import JsonService  # type: ignore

from path_planner.shape_adapter import ShapeAdapter, RectangleFromBottomLeft, Rectangle
from path_planner.rrt import RRT, RRT_star, dijkstra


class PathPlanner(Node):
    
    def __init__(self, n_iter: int = 2000, radius: float = 0.5, step_size: float = 0.2, buffer: float = 0.25):
        super().__init__('PathPlanner')  # type: ignore
        # RRT* parameters
        self.n_iter = n_iter
        self.radius = radius
        self.step_size = step_size
        self.top_right = None
        self.bottom_left = (0, 0)
        
        # Environment parameters
        self.static_graph = None
        self.dynamic_graph = None
        self.static_env = None
        self.dynamic_shapes = []
        self.dynamic_env = None
        self.buffer = buffer

        self.path_plan_srv = self.create_service(JsonService, "path_planner", self.find_motion_plan)
        self.path_plan_srv
        self.initialize_environment_srv = self.create_service(JsonService, "initialize_environment", self.initialize_environment)
        self.initialize_environment_srv
     
    def initialize_environment(self, req_obj, resp_obj):
        config = self.load_config(req_obj)
        try:
            static_items, dynamic_items = config["static_items"], config["dynamic_items"]
        except:
            raise Exception("Environments should be initialized properly.")
            
        s_polygons = [Polygon(shell=item["shell"], holes=item["holes"]) for item in static_items]
        self.dynamic_shapes = [Rectangle(*item) for item in dynamic_items]
        d_polygons = [shape.polygon for shape in self.dynamic_shapes]
        static_env, dynamic_env = MultiPolygon(s_polygons), MultiPolygon(s_polygons + d_polygons)
        self.static_env = ShapeAdapter(items=static_env, buffer=self.buffer)
        self.dynamic_env = ShapeAdapter(items=dynamic_env, buffer=self.buffer)
                        
        print(f"Environment is initialized.")
        resp_obj.resp = json.dumps(True)
        return resp_obj
    
    def find_motion_plan(self, req_obj, resp_obj):
        print(f"Motion planning attempt started.")
        config = self.load_config(req_obj)
        try:
            start, end = config["start"], config["end"]
            start_grasps, end_grasps = self.find_grasping_delegates(start, end)
        except:
            raise Exception("Start and end coordinates should be provided for motion plan.")
        
        if self.dynamic_env is None or self.static_env is None:
            raise Exception("Before motion planning, environments should be specified!")

        then = datetime.now()
        for i, start in enumerate(start_grasps[0: 2 if 2 < len(start_grasps) else len(start_grasps)]):
            for j, end in enumerate(end_grasps[0: 2 if 2 < len(end_grasps) else len(end_grasps)]):
                n_iter = int(self.n_iter / 10) if i != 0 or j != 0 else self.n_iter
                self.dynamic_graph = RRT(start, end, self.dynamic_graph, self.dynamic_env, n_iter, self.radius, self.step_size, self.top_right, self.bottom_left)
                # if True:
                if self.dynamic_graph.success:
                    path = dijkstra(self.dynamic_graph)
                    # path = [start, [-1, -1], [-2, -2], end]
                    if path[0] != start:
                        path.insert(0, start)
                    if path[-1] != end:
                        path.append(end)
                    print(path)
                    resp_obj.resp = json.dumps({
                        "path": path,
                        "colliders": []
                    }, indent=4)
                    print("Time spent: ", datetime.now() - then)
                    return resp_obj
            
        for i, start in enumerate(start_grasps[0: 2 if 2 < len(start_grasps) else len(start_grasps)]):
            for j, end in enumerate(end_grasps[0: 2 if 2 < len(end_grasps) else len(end_grasps)]):
                n_iter = int(self.n_iter / 10) if i != 0 or j != 0 else self.n_iter
                self.static_graph = RRT(start, end, self.static_graph, self.static_env, n_iter, self.radius, self.step_size, self.top_right, self.bottom_left)
                if self.static_graph.success:
                    path = dijkstra(self.static_graph)
                    colliders = self.dynamic_env.find_colliders(path)
                    collider_centers = [(coll.centroid.x, coll.centroid.y) for coll in colliders]

                    if path[0] != start:
                        path.insert(0, start)
                    if path[-1] != end:
                        path.append(end)
                    print(path)
                    resp_obj.resp = json.dumps({
                        "path": path,
                        "colliders": collider_centers
                    }, indent=4)
                    print("Time spent: ", datetime.now() - then)
                    return resp_obj
        
        resp_obj.resp = "null"
        return resp_obj

    def find_grasping_delegates(self, start, end):
        for ds in self.dynamic_shapes:
            if abs(ds.x - start[0]) < 0.01 and abs(ds.y - start[1]) < 0.01:
                dist_to_end = [(pnt.x-end[0])**2 + (pnt.y-end[1])**2 for pnt in ds.grasp_points]
                zipped = list(zip(ds.grasp_points, dist_to_end))
                start_list = [[z0.x, z0.y] for z0, z1 in sorted(zipped, key=lambda z: z[1])]
                break
        else:
            start_list = [start]
        for ds in self.dynamic_shapes:
            if abs(ds.x - end[0]) < 0.01 and abs(ds.y - end[1]) < 0.01:
                dist_to_start = [(pnt.x-start[0])**2 + (pnt.y-start[1])**2 for pnt in ds.grasp_points]
                zipped = list(zip(ds.grasp_points, dist_to_start))
                end_list = [[z0.x, z0.y] for z0, z1 in sorted(zipped, key=lambda z: z[1])]
                break
        else:
            end_list = [end]
        return start_list, end_list

    def load_config(self, req_obj):
        config = json.loads(str(req_obj.req))
        print(f"Received configuration: {config}\n")
        if "n_iter" in config:
            self.n_iter = config["n_iter"]
        if "radius" in config:
            self.radius = config["radius"]
        if "step_size" in config:
            self.step_size = config["step_size"]
        if "top_right" in config:
            self.top_right = config["top_right"]
        if "bottom_left" in config:
            self.bottom_left = config["bottom_left"]
        if "buffer" in config:
            self.buffer = config["buffer"]
        return config


def main(args=None):
    rclpy.init(args=args)

    path_planner = PathPlanner()
    
    rclpy.spin(path_planner)
    
    path_planner.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
    