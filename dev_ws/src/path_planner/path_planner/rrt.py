'''
MIT License
Copyright (c) 2019 Fanjin Zeng
This work is licensed under the terms of the MIT license, see <https://opensource.org/licenses/MIT>.  
'''

from collections import deque
from collections.abc import Sequence, Mapping
from dataclasses import dataclass, field
from operator import truediv
import numpy as np
from random import random
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from shapely.geometry import Point, LineString, Polygon, MultiPoint, MultiLineString, MultiPolygon

    
class GraphWithDistances:
    ''' Define graph '''
    def __init__(self, startpos, endpos):
        startpos, endpos = tuple(startpos), tuple(endpos)
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}
        
    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))
        
    def randomPosition(self, top_right, bottom_left):
        (self.min_x, self.min_y), (self.max_x, self.max_y) = bottom_left, top_right
        rx = self.min_x + random()*(self.max_x - self.min_x)
        ry = self.min_y + random()*(self.max_y - self.min_y)
        return rx, ry

    def nearest(self, vex, environment):
        Nvex = None
        Nidx = None
        minDist = float("inf")

        for idx, v in enumerate(self.vertices):
            line = LineString([v, vex])
            if environment.intersects(line):
                continue

            dist = self.distance(v, vex)
            if dist < minDist:
                minDist = dist
                Nidx = idx
                Nvex = v
        return Nvex, Nidx

    @staticmethod
    def newVertex(randvex, nearvex, stepSize):
        dirn = np.array(randvex) - np.array(nearvex)
        length = np.linalg.norm(dirn)
        dirn = (dirn / length) * min (stepSize, length)

        newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1])
        return newvex

    @staticmethod
    def distance(x, y):
        return np.linalg.norm(np.array(x) - np.array(y))


def RRT_star(startpos, endpos, environment, n_iter, radius, stepSize, top_right, bottom_left=(0,0)):
    ''' RRT star algorithm '''
    startpos, endpos = tuple(startpos), tuple(endpos)
    G = GraphWithDistances(startpos, endpos)

    print(startpos, endpos, environment, n_iter, radius, stepSize, top_right, bottom_left)
    for i in range(n_iter):
        randvex = G.randomPosition(top_right, bottom_left)
        if environment.intersects(Point(randvex)):
            continue

        nearvex, nearidx = G.nearest(randvex, environment)
        if nearvex is None:
            continue

        newvex = Graph.newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = Graph.distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)
        G.distances[newidx] = G.distances[nearidx] + dist

        # update nearby vertices distance (if shorter)
        for vex in G.vertices:
            if vex == newvex:
                continue

            dist = Graph.distance(vex, newvex)
            if dist > radius:
                continue

            line = LineString([vex, newvex])
            if environment.intersects(line):
                continue

            idx = G.vex2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

        dist = Graph.distance(newvex, G.endpos)
        end_line = LineString([newvex, G.endpos])
        if dist < 2 * radius and not environment.intersects(end_line):
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx]+dist)
            except:
                G.distances[endidx] = G.distances[newidx]+dist

            G.success = True
            print('success')
            break
    return G


class Graph:
    ''' Define graph '''
    def __init__(self):
        self.vertices = []
        self.edges = []
        self.vex2idx = {}
        self.neighbors = {}
            
    def set_endpoints(self, startpos, endpos):
        self.startpos, self.endpos = tuple(startpos), tuple(endpos)
        self.success = False
        self.start_connected = False
        self.end_connected = False
        
        if not self.vertices:
            self.start_connected = True
            self.vertices = [self.startpos]
            self.edges = []

            self.vex2idx = {self.startpos:0}
            self.neighbors = {0:[]}
            return
                
    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2):
        cost = Graph.distance(self.vertices[idx1], self.vertices[idx2])
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))

    @staticmethod
    def randomPosition(environment, top_right, bottom_left):
        (min_x, min_y), (max_x, max_y) = bottom_left, top_right
        rx = min_x + random()*(max_x - min_x)
        ry = min_y + random()*(max_y - min_y)
        if environment.intersects(Point(rx, ry)):
            return Graph.randomPosition(environment, top_right, bottom_left)
        return rx, ry

    def nearest(self, vex, environment):
        Nvex = None
        Nidx = None
        minDist = float("inf")

        for idx, v in enumerate(self.vertices):
            line = LineString([v, vex])
            if environment.intersects(line):
                continue

            dist = self.distance(v, vex)
            if dist < minDist:
                minDist = dist
                Nidx = idx
                Nvex = v
        return Nvex, Nidx
    
    def is_endpoints_connected(self):
        if self.start_connected and self.end_connected:
            self.success = True
            return True
        return False
    
    def connect_endpoints_to_graph(self, environment, radius):
        if environment.intersects(self.startpos) or environment.intersects(self.endpos):
            return False
        if self.startpos in self.vertices:
            self.start_connected = True
        if self.endpos in self.vertices:
            self.end_connected = True
        if not self.start_connected:
            Nvex, Nidx = self.nearest(self.startpos, environment)
            if Nvex and self.distance(self.startpos, Nvex) < 2 * radius:
                startidx = self.add_vex(self.startpos)
                self.add_edge(startidx, Nidx)
                self.start_connected = True
        if not self.end_connected:
            Nvex, Nidx = self.nearest(self.endpos, environment)
            if Nvex and self.distance(self.endpos, Nvex) < 2 * radius:
                endidx = self.add_vex(self.endpos)
                self.add_edge(Nidx, endidx)        
                self.end_connected = True
        return True
    
    def connect_newvex_to_endpoints(self, newvex, environment, radius):
        if not self.start_connected:
            dist = Graph.distance(newvex, self.startpos)
            start_line = LineString([newvex, self.startpos])
            if dist < 2 * radius and not environment.intersects(start_line):
                startidx = self.add_vex(self.startpos)
                self.add_edge(startidx, self.vex2idx[newvex])
                self.start_connected = True
                return
        if not self.end_connected:
            dist = Graph.distance(newvex, self.endpos)
            end_line = LineString([newvex, self.endpos])
            if dist < 2 * radius and not environment.intersects(end_line):
                endidx = self.add_vex(self.endpos)
                self.add_edge(self.vex2idx[newvex], endidx)
                self.end_connected = True
                return
    
    @staticmethod
    def newVertex(randvex, nearvex, stepSize):
        dirn = np.array(randvex) - np.array(nearvex)
        length = np.linalg.norm(dirn)
        dirn = (dirn / length) * min (stepSize, length)

        newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1])
        return newvex
    
    @staticmethod
    def distance(x, y):
        return np.linalg.norm(np.array(x) - np.array(y))
    

def RRT(startpos, endpos, G, environment, n_iter, radius, stepSize, top_right, bottom_left=(0,0)):
    ''' RRT algorithm '''
    G = Graph() if G is None else G
    startpos, endpos = tuple(startpos), tuple(endpos)
    G.set_endpoints(startpos, endpos)
    if not G.connect_endpoints_to_graph(environment, radius):
        return False

    for _ in range(n_iter):
        randvex = G.randomPosition(environment, top_right, bottom_left)

        nearvex, nearidx = G.nearest(randvex, environment)
        if nearvex is None:
            continue

        newvex = Graph.newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        G.add_edge(newidx, nearidx)
        G.connect_newvex_to_endpoints(newvex, environment, radius)
        if G.is_endpoints_connected(): 
            print('success\n', flush=True)
            break
    return G


def dijkstra(G):
    '''
  Dijkstra algorithm for finding shortest path from start position to end.
    '''
    srcIdx = G.vex2idx[G.startpos]
    dstIdx = G.vex2idx[G.endpos]

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    return list(path)


