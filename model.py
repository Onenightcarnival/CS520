import osmnx as ox, networkx as nx, numpy as np
from heapq import *
from itertools import count
import time
import sys

# define some edge impedance function here
def impedance(length, grade):
    penalty = grade ** 2
    return length * penalty

# add impedance and elevation rise values to each edge in the projected graph
# use absolute value of grade in impedance function if you want to avoid uphill and downhill
def add_impedence(G_proj):
    for u, v, k, data in G_proj.edges(keys=True, data=True):
        data['impedance'] = impedance(data['length'], data['grade_abs'])
        data['rise'] = data['length'] * data['grade']

def getelevationcost(G_proj, a, b):
    return (G_proj.nodes[a]['elevation'] - G_proj.nodes[b]['elevation'])

def getcost(G_proj, a, b):
    return G_proj.edges[a, b, 0]['length']

def getpath(came_from, origin, destination):
    route_by_length_minele = []
    p = destination
    route_by_length_minele.append(p)
    while p != origin:
       p = came_from[p]
       route_by_length_minele.append(p)
    route_by_length_minele = route_by_length_minele[::-1]
    return route_by_length_minele

def getTotalElevation(G_proj, route):
    if not route:
        return 0
    elevationcost = 0
    for i in range(len(route)-1):
         elevationData = getelevationcost(G_proj, route[i], route[i+1])
         if elevationData > 0:
             elevationcost += elevationData
    return elevationcost

def getTotalLength(G_proj, route):
    if not route:
        return 0
    cost = 0
    for i in range(len(route)-1):
         cost += getcost(G_proj, route[i], route[i+1])
    return cost

def print_route_stats(G_proj, route):
    route_lengths = ox.get_route_edge_attributes(G_proj, route, 'length')
    print('Total trip distance: {:,.0f} meters'.format(getTotalLength(G_proj, route)))
    print('Total elevation change: {:,.0f}'.format(getTotalElevation(G_proj, route)))

#Algorithm to get Shortest length / Elevation path
def get_shortest_path(graph, source=0, target=0, weight='length'):
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
                incCost = getcost(graph, u, next)
            elif weight == 'elevation':
                incCost = getelevationcost(graph, u, next)
            if incCost > 0:
                new_cost += incCost
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                heappush(frontier, (priority, next))
                came_from[next] = current
    return getpath(came_from, source, target)

#Strategy1
def dijkstra_search(graph, start, goal, viablecost, mode='minimize'):
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
            new_cost = cost_so_far[current] + getcost(graph, current, next)
            new_cost_ele = cost_so_far_ele[current]
            elevationcost = getelevationcost(graph, current, next)
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
    return getpath(came_from, start, goal)


#Strategy2
def dfs_get_all_paths(graph, start, goal, maxlength):
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
            dfs(nextnode, le + abs(getcost(graph, current, nextnode)), currentpath + [current], visited + [nextnode])
        return
    dfs(start, 0, [], [])
    print ("Total paths:", len(paths))
    final_path = None
    
    minval = sys.maxsize
    maxval = -1*sys.maxsize
    min_path, max_path = [], []
    for path in paths:
        elevationData = getTotalElevation(graph, path)
        if minval != min(elevationData, minval):
            minval = elevationData
            min_path = path
        if maxval != max(elevationData, maxval):
            maxval = elevationData
            max_path = path
    return min_path, max_path

#Strategy3
def dfs_get_all_paths_2(G_proj, origin, destination, can_travel, cutoff):
    paths = list(nx.all_simple_paths(G_proj, source=origin, target=destination, cutoff=cutoff))
    minval = sys.maxsize
    maxval = -1*sys.maxsize
    min_path, max_path = [], []
    for path in paths:
        elevationData = getTotalElevation(G_proj, path)
        lengthData = getTotalLength(G_proj, path)
        if minval != min(elevationData, minval) and lengthData < can_travel:
            minval = elevationData
            min_path = path
        if maxval != max(elevationData, maxval) and lengthData < can_travel:
            maxval = elevationData
            max_path = path
    return min_path, max_path


