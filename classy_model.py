import osmnx as ox, networkx as nx, numpy as np
from heapq import *
from itertools import count
import time
import sys

class Model(object):
	# define some edge impedance function here
	def impedance(self, length, grade):
		penalty = grade ** 2
		return length * penalty

	# add impedance and elevation rise values to each edge in the projected graph
	# use absolute value of grade in impedance function if you want to avoid uphill and downhill
	def add_impedence(self, G_proj):
		for u, v, k, data in G_proj.edges(keys=True, data=True):
			data['impedance'] = impedance(data['length'], data['grade_abs'])
			data['rise'] = data['length'] * data['grade']

	def heuristic(self, G_proj, a, b):
		return abs(G_proj.nodes[a]['elevation'] - G_proj.nodes[b]['elevation'])

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


	def k_shortest_paths(self, G, source, target, k=1, weight='weight'):
		"""Returns the k-shortest paths from source to target in a weighted graph G.
		Parameters
		----------
		G : NetworkX graph
		source : node
		   Starting node
		target : node
		   Ending node
		   
		k : integer, optional (default=1)
			The number of shortest paths to find
		weight: string, optional (default='weight')
		   Edge data key corresponding to the edge weight
		Returns
		-------
		lengths, paths : lists
		   Returns a tuple with two lists.
		   The first list stores the length of each k-shortest path.
		   The second list stores each k-shortest path.  
		Raises
		------
		NetworkXNoPath
		   If no path exists between source and target.
		Examples
		--------
		G=nx.complete_graph(5)    
		print(k_shortest_paths(G, 0, 4, 4))
		([1, 2, 2, 2], [[0, 4], [0, 1, 4], [0, 2, 4], [0, 3, 4]])
		Notes
		------
		Edge weight attributes must be numerical and non-negative.
		Distances are calculated as sums of weighted edges traversed.
		"""
		if source == target:
			return ([0], [[source]]) 
		#import pdb; pdb.set_trace()   
		#length, path = nx.single_source_dijkstra(G, source, target, weight=weight)
		length, path = nx.single_source_dijkstra(G, source, None, weight=weight)
		if target not in length:
			raise nx.NetworkXNoPath("node %s not reachable from %s" % (source, target))
			
		lengths = [length[target]]
		paths = [path[target]]
		c = count()        
		B = []                        
		G_original = G.copy()    
		
		for i in range(1, k):
			for j in range(len(paths[-1]) - 1):            
				spur_node = paths[-1][j]
				root_path = paths[-1][:j + 1]
				
				edges_removed = []
				for c_path in paths:
					if len(c_path) > j and root_path == c_path[:j + 1]:
						u = c_path[j]
						v = c_path[j + 1]
						if G.has_edge(u, v):
							#edge_attr = G.edge[u][v]
							edge_attr = G[u][v]
							G.remove_edge(u, v)
							edges_removed.append((u, v, edge_attr))
				
				for n in range(len(root_path) - 1):
					node = root_path[n]
					# out-edges
					for u, v, edge_attr in G.edges_iter(node, data=True):
						G.remove_edge(u, v)
						edges_removed.append((u, v, edge_attr))
					
					if G.is_directed():
						# in-edges
						for u, v, edge_attr in G.in_edges_iter(node, data=True):
							G.remove_edge(u, v)
							edges_removed.append((u, v, edge_attr))
				
				spur_path_length, spur_path = nx.single_source_dijkstra(G, spur_node, target, weight=weight)            
				if target in spur_path and spur_path[target]:
					total_path = root_path[:-1] + spur_path[target]
					total_path_length = get_path_length(G_original, root_path, weight) + spur_path_length[target]                
					heappush(B, (total_path_length, next(c), total_path))
					
				for e in edges_removed:
					u, v, edge_attr = e
					G.add_edge(u, v, edge_attr)
						   
			if B:
				(l, _, p) = heappop(B)        
				lengths.append(l)
				paths.append(p)
			else:
				break
		
		return (lengths, paths)







