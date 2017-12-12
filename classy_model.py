import sys
import osmnx as ox
import networkx as nx
import numpy as np
from heapq import *
from itertools import count
import time
from model import *
#import cPickle as pkl
import pickle as pkl

class Model(object):

    def set_view(self, vobj):
        self.vobj = vobj

    def get_route(self, G_proj, origin, destination, extra_travel, mode, strategy, bbox):
        shortest_path = get_shortest_path(G_proj, source=origin, target=destination, weight='length')
        print("Printing Statistics of Shortest path route")
        print(shortest_path)
        print_route_stats(G_proj, shortest_path)

        shortest_elevation_path = get_shortest_path(G_proj, source=origin, target=destination, weight='elevation')
        print("Printing Statistics of Shortest Elevation path route")
        print_route_stats(G_proj, shortest_elevation_path)

        shortest_path_length = getTotalLength(G_proj, shortest_path)
        can_travel = ((100.0 + extra_travel)*shortest_path_length)/100.0
        print("Distance you are willing to travel : ", can_travel)

        if strategy == 1:
            #Strategy1
            t = time.time()
            route_minimize_elevation1 = dijkstra_search(G_proj, origin, destination, can_travel, mode='minimize')
            print ("Algorithm 1 took :", time.time()-t, " seconds")
            print("Printing Statistics of our algorithm's minimum Elevation route")
            print_route_stats(G_proj, route_minimize_elevation1)
            self.vobj.show_route(G_proj, route_minimize_elevation1, bbox)

        if strategy == 2:
            #Strategy2
            t = time.time()
            route_minimize_elevation2, route_maximize_elevation2 = dfs_get_all_paths(G_proj, origin, destination, can_travel)
            print ("Algorithm 2 time:", time.time()-t)
            print("Printing Statistics of Minimum Elevation route")
            print_route_stats(G_proj, route_minimize_elevation2)
            if mode == 'minimize':
                print_route_stats(G_proj, route_minimize_elevation2)
                self.vobj.show_route(G_proj, route_minimize_elevation2, bbox)
            elif mode == 'maximize':
                print_route_stats(G_proj, route_maximize_elevation2)
                self.vobj.show_route(G_proj, route_maximize_elevation2, bbox)

        if strategy == 3:
            #Strategy3
            t = time.time()
            route_minimize_elevation3, route_maximize_elevation3 = dfs_get_all_paths_2(G_proj, origin, destination, can_travel, len(shortest_path)+4)
            print ("Algorithm 3 took :", time.time()-t, " seconds")
            print("Printing Statistics of Minimum Elevation route")
            if mode == 'minimize':
                print_route_stats(G_proj, route_minimize_elevation3)
                self.vobj.show_route(G_proj, route_minimize_elevation3, bbox)
            elif mode == 'maximize':
                print_route_stats(G_proj, route_maximize_elevation3)
                self.vobj.show_route(G_proj, route_maximize_elevation3, bbox)

    def getelevationcost(self, G_proj, a, b):
        return (G_proj.nodes[a]['elevation'] - G_proj.nodes[b]['elevation'])

    def getcost(self, G_proj, a, b):
        return G_proj.edges[a, b, 0]['length']

    def getpath(self, came_from, origin, destination):
        route_by_length_minele = []
        p = destination
        route_by_length_minele.append(p)
        while p != origin:
           p = came_from[p]
           route_by_length_minele.append(p)
        route_by_length_minele = route_by_length_minele[::-1]
        return route_by_length_minele

    def getTotalElevation(self, G_proj, route):
        if not route:
            return 0
        elevationcost = 0
        for i in range(len(route)-1):
             elevationData = self.getelevationcost(G_proj, route[i], route[i+1])
             if elevationData > 0:
                 elevationcost += elevationData
        return elevationcost

    def getTotalLength(self, G_proj, route):
        if not route:
            return 0
        cost = 0
        for i in range(len(route)-1):
             cost += self.getcost(G_proj, route[i], route[i+1])
        return cost

    def get_shortest_path(self, graph, source=0, target=0, weight='length'):
        frontier = []
        heappush(frontier, (0, source))
        came_from = {}
        cost_so_far = {}
        came_from[source] = None
        cost_so_far[source] = 0
        
        while len(frontier) != 0:
            (val, current) = heappop(frontier)
            if current == target:
                break;
            for u, next, data in graph.edges(current, data=True):
                new_cost = cost_so_far[current]
                if weight == 'length':
                    incCost = self.getcost(graph, u, next)
                elif weight == 'elevation':
                    incCost = self.getelevationcost(graph, u, next)
                if incCost > 0:
                    new_cost += incCost
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost
                    heappush(frontier, (priority, next))
                    came_from[next] = current
        return self.getpath(came_from, source, target)

    def dijkstra_search(self, graph, start, goal, viablecost, mode='minimize'):
        frontier = []
        heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {}
        cost_so_far_ele = {}
        came_from[start] = None
        cost_so_far[start] = 0
        cost_so_far_ele[start] = 0
        while len(frontier) != 0:
            (val, current) = heappop(frontier)
            if current == goal:
                if cost_so_far[current] <= viablecost:
                    break
            for u, next, data in graph.edges(current, data=True):
                new_cost = cost_so_far[current] + self.getcost(graph, current, next)
                new_cost_ele = cost_so_far_ele[current]
                elevationcost = self.getelevationcost(graph, current, next)
                if elevationcost > 0:
                    new_cost_ele = new_cost_ele + elevationcost 
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far_ele[next] = new_cost_ele
                    cost_so_far[next] = new_cost
                    priority = new_cost_ele
                    if mode=='maximize':
                        priority=priority
                    heappush(frontier, (priority, next))
                    came_from[next] = current
        return self.getpath(came_from, start, goal)

    def print_route_stats(self, G_proj, route):
        #route_grades = ox.get_route_edge_attributes(G_proj, route, 'grade_abs')
        #msg = 'The average grade is {:.1f}% and the max is {:.1f}%'
        #print(msg.format(np.mean(route_grades)*100, np.max(route_grades)*100))

        #route_rises = ox.get_route_edge_attributes(G_proj, route, 'rise')
        #ascent = np.sum([rise for rise in route_rises if rise >= 0])
        #descent = np.sum([rise for rise in route_rises if rise < 0])
        #msg = 'Total elevation change is {:.0f} meters: a {:.0f} meter ascent and a {:.0f} meter descent'
        #print(msg.format(np.sum(route_rises), ascent, abs(descent)))

        route_lengths = ox.get_route_edge_attributes(G_proj, route, 'length')
        print('Total trip distance: {:,.0f} meters'.format(self.getTotalLength(G_proj, route)))
        print('Total elevation change: {:,.0f}'.format(self.getTotalElevation(G_proj, route)))

    def dfs_get_all_paths(self, graph, start, goal, maxlength):
        paths = []
        def dfs(current, le, currentpath, visited):
            if current == goal:
                if le > maxlength:
                    return
                else:
                    currentpath.append(current)
                    paths.append(currentpath)
                    #print ("This path length:",length)
                    #print ("path found")
                    return
            if le > maxlength:
                return
            for u, nextnode, data in graph.edges(current, data=True):
                if nextnode in visited:
                    continue
                dfs(nextnode, le + abs(self.getcost(graph, current, nextnode)), currentpath + [current], visited + [nextnode])
            return
        dfs(start, 0, [], [])
        print ("Total paths:", len(paths))
        final_path = None
        
        minval = sys.maxsize
        maxval = -1*sys.maxsize
        min_path, max_path = [], []
        for path in paths:
            elevationData = self.getTotalElevation(graph, path)
            if minval != min(elevationData, minval):
                minval = elevationData
                min_path = path
            if maxval != max(elevationData, maxval):
                maxval = elevationData
                max_path = path
        return min_path, max_path


    def dfs_get_all_paths_2(self, G_proj, origin, destination, can_travel, cutoff):
        paths = list(nx.all_simple_paths(G_proj, source=origin, target=destination, cutoff=cutoff))
        minval = sys.maxsize
        maxval = -1*sys.maxsize
        min_path, max_path = [], []
        for path in paths:
            elevationData = self.getTotalElevation(G_proj, path)
            lengthData = self.getTotalLength(G_proj, path)
            if minval != min(elevationData, minval) and lengthData < can_travel:
                minval = elevationData
                min_path = path
            if maxval != max(elevationData, maxval) and lengthData < can_travel:
                maxval = elevationData
                max_path = path
        return min_path, max_path
