import osmnx as ox
import time
import networkx as nx
import numpy as np
from heapq import *
from itertools import count
import time
from model import *
import _pickle as pkl
from classy_model import Model

ox.config(log_console=True, use_cache=True)


class Main(Model):
    def __init__(self):
        pass
    def get_map(self, place='San Francisco', newPlace=False):
        if newPlace==False:
            return pkl.load(open("graph_projected.pkl","rb")), pkl.load(open("graph.pkl","rb"))
	
	#Downloading Local map
        place = 'San Francisco'
        place_query = {'city':'San Francisco', 'state':'California', 'country':'USA'}
        G = ox.graph_from_place(place_query, network_type='drive')

        #Adding Elevation data from GoogleMaps
        G = ox.add_node_elevations(G, api_key='AIzaSyDrIKxLj0P90vzCUJ_6huBSDxiq8wYo9LM')
        G = ox.add_edge_grades(G)
        pkl.dump(G, open("graph.pkl","wb"))

        #projecting map on to 2D space
        G_proj = ox.project_graph(G)
        pkl.dump(G_proj, open("graph_projected.pkl","wb"))
        return G_proj
if __name__ == "__main__":
    mainobj = Main()
    G_proj, G = mainobj.get_map()

    # Enter Latitude and Longitude to select origin and destination
    origin = ox.get_nearest_node(G, (37.77, -122.426))
    destination = ox.get_nearest_node(G, (37.773, -122.441))

    # Enter the percentage you want to travel extra
    extra_travel = 10
    bbox = ox.bbox_from_point((37.772, -122.434), distance=1500, project_utm=True)


    shortest_path = get_shortest_path(G_proj, source=origin, target=destination, weight='length')
    print("Printing Statistics of Shortest path route")
    print(shortest_path)
    print_route_stats(G_proj, shortest_path)

    shortest_elevation_path = get_shortest_path(G_proj, source=origin, target=destination, weight='elevation')
    print("Printing Statistics of Shortest Elevation path route")
    print_route_stats(G_proj, shortest_elevation_path)

    shortest_path_length = getTotalLength(G_proj, shortest_path)
    can_travel = ((100.0 + extra_travel)*shortest_path_length)/100.0
    print("Extra travel: ", can_travel)

    #Strategy1
    #As of now, maximize is not working properly, please dont use
    route_minimize_elevation1 = dijkstra_search(G_proj, origin, destination, can_travel, mode='maximize')
    print("Printing Statistics of our algorithm's minimum Elevation route")
    print_route_stats(G_proj, route_minimize_elevation1)

    #Strategy2
    t = time.time()
    route_minimize_elevation2, route_maximize_elevation2 = dfs_get_all_paths(G_proj, origin, destination, can_travel)
    #print ("Srikanth's algo time:", time.time()-t)
    print("Printing Statistics of Minimum Elevation route")
    print_route_stats(G_proj, route_minimize_elevation2)
    print("Printing Statistics of Maximum Elevation route")
    print_route_stats(G_proj, route_maximize_elevation2)

    #Strategy3
    route_minimize_elevation3, route_maximize_elevation3 = dfs_get_all_paths_2(G_proj, origin, destination, can_travel, len(shortest_path)+4)
    print("Printing Statistics of Minimum Elevation route")
    print_route_stats(G_proj, route_minimize_elevation3)
    print("Printing Statistics of Maximum Elevation route")
    print_route_stats(G_proj, route_maximize_elevation3)

    #Strategy4
    #route_by_impedance = nx.shortest_path(G_proj, source=origin, target=destination, weight='impedance')
    #fig, ax = ox.plot_graph_route(G_proj, route_by_length, bbox=bbox, node_size=0)
    '''
    #kpaths = k_shortest_paths(G_proj, source=origin,target=destination,k = 5,weight='length')
    #print (kpaths)
    '''
