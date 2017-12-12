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

class Controller(object):
    def __init__(self):
        self.VALID_MODES = ['minimize', 'maximize']
        self.origin_lat = None
        self.origin_long = None
        self.dest_lat = None
        self.dest_long = None
        self.extra_travel = None
        self.mode = None
        self.model = None
        self.get_data_from_user()

    def set_strategy(self, strategy):
        self.strategy = strategy

    def set_model(self, model):
        self.model = model
        self.model.get_route(self.G_proj, self.origin, self.destination, self.extra_travel, self.mode, self.strategy, self.bbox)

    def get_data_from_user(self):
        #Taking input
        self.origin_lat = input("Please enter the Latitude of the Origin \n")
        self.origin_long = input("Please enter the Longitude of the Origin \n")
        print("Latitude of origin : ", float(self.origin_lat), " and Longitude of origin", float(self.origin_long))
        self.dest_lat = input("Please enter the Latitude of the Destination \n")
        self.dest_long = input("Please enter the Longitude of the Destination \n")
        print("Latitude of destination : ", float(self.dest_lat), " and Longitude of destination", float(self.dest_long))
        self.extra_travel = float(input("Please enter the percentage of shortest path between above points that you are willing to travel extra \n"))
        print("x% : ", self.extra_travel)
        self.mode = input("To minimize elevation, please type 'minimize' \n To maximize elevation, please type 'maximize'")

        if self.mode not in self.VALID_MODES:
            print ("Mode invalid")
            sys.exit() 

        self.G_proj, self.G = self.get_map()

        self.origin = ox.get_nearest_node(self.G, (float(self.origin_lat), float(self.origin_long))) #(37.77, -122.426))
        self.destination = ox.get_nearest_node(self.G, (float(self.dest_lat), float(self.dest_long))) #(37.773, -122.441))
        self.bbox = ox.bbox_from_point((float(self.origin_lat), float(self.origin_long)), distance=1500, project_utm=True)


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
        return G_proj, G
