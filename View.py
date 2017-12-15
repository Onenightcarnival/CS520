import osmnx as ox
import networkx as nx
import numpy as np
#import cPickle as pkl
import pickle as pkl

class View(object):
    def show_route(self, G_proj, route, bbox):
        print("Showing route")
        #fig, ax = ox.plot_graph_route(G_proj, route_by_length, bbox=bbox, node_size=0)
