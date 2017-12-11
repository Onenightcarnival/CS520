import osmnx as ox
import networkx as nx
import numpy as np
from heapq import *
from itertools import count
import time
from model import *
import cPickle as pkl

ox.config(log_console=True, use_cache=True)


def get_map(place='San Francisco', newPlace=False):
    if newPlace == False:
        return pkl.load(open("graph_projected.pkl", "rb")), pkl.load(open("graph.pkl", "rb"))

    # Downloading Local map
    place = 'San Francisco'
    place_query = {'city': 'San Francisco', 'state': 'California', 'country': 'USA'}
    G = ox.graph_from_place(place_query, network_type='drive')

    # Adding Elevation data from GoogleMaps
    G = ox.add_node_elevations(G, api_key='AIzaSyDrIKxLj0P90vzCUJ_6huBSDxiq8wYo9LM')
    G = ox.add_edge_grades(G)
    pkl.dump(G, open("graph.pkl", "wb"))

    # projecting map on to 2D space
    G_proj = ox.project_graph(G)
    pkl.dump(G_proj, open("graph_projected.pkl", "wb"))
    return G_proj


def random_points_in_bbox(latitude, longitude, distance, num_points):
    bbox = ox.bbox_from_point((latitude, longitude), distance=distance)
    min_latitude = min(bbox[0], bbox[1])
    diff_latitude = abs(bbox[0] - bbox[1])
    min_longitude = min(bbox[2], bbox[3])
    diff_longitude = abs(bbox[2] - bbox[3])
    lat_long_pair = []
    for i in range(num_points):
        u_lat = np.random.uniform(0.0001, diff_latitude)
        lat = min_latitude + u_lat
        u_long = np.random.uniform(0.0001, diff_longitude)
        lgtd = min_longitude + u_long
        lat_long_pair.append([lat, lgtd])

    return lat_long_pair


def plot_results(s_dist, se_dist, algo_dist):
    x_axis = np.arange(len(s_dist))
    import matplotlib.pyplot as plt
    plt.plot(x_axis, s_dist, label="shortest_distance")
    plt.plot(x_axis, se_dist, label="shortest_elevation")
    plt.plot(x_axis, algo_dist, label="our_algo")
    plt.xticks(x_axis)
    plt.legend()
    plt.show()


def plot_results_from_dict(dic, plot_one=False, key_one=None):
    import matplotlib.pyplot as plt
    x_axis = None
    max_len = dic["MAX"]
    y_ticks = np.arange(0, int(max_len + 200))
    len_1 = 0
    for key in dic.keys():
        if key != "MAX":
            len_1 = len(dic[key])
            x_axis = np.arange(len(dic[key]))
            break
    fig, ax = plt.subplots()
    print dic
    for key in dic.keys():
        if key != "MAX":
            assert len_1 == len(dic[key])
            ax.plot(x_axis, dic[key], label=key)
    # ax.set_xlim((0, len_1))
    ax.set_ylim((0, int(max_len + 200)))
    plt.xticks(x_axis)
    plt.xlabel("Destination")
    plt.ylabel("Distance in m")
    plt.ylim((0, int(max_len + 1)))
    plt.legend()
    if not plot_one:
        plt.suptitle("Path comparison")
        plt.savefig("comparison_chart")
    else:
        plt.suptitle("Distance Graph")
        plt.savefig(key_one)
        # plt.show()


def set_max(max_curr, new_val):
    if new_val > max_curr:
        return new_val
    else:
        return max_curr


def compare_algorithms(G, G_proj, origin_lat_long, bbox_lat_long, bbox_dist, num_dest, extrapercent_travel, plot=False,
                       dump=True, dump_file="dist_dump.pkl", plot_individuals=True):
    origin = ox.get_nearest_node(G, (origin_lat_long[0], origin_lat_long[1]))
    dest_list = random_points_in_bbox(bbox_lat_long[0], bbox_lat_long[1], bbox_dist, num_dest)
    max_distance_val = 0.0
    cost_data = []
    shortest = []
    shortest_elev = []
    algo_dist = []
    dic = {"Shortest_Path": [], "Least_Elevation": [], "Modified_Dijkstra": [], "DFS_1_min_elevation": []
        , "DFS_1_max_elevation": [], "DFS_2_min_elevation": [], "DFS_2_max_elevation": []}
    for each in dest_list:
        # Shortest
        destination = ox.get_nearest_node(G, (each[0], each[1]))
        route_actual = nx.shortest_path(G_proj, source=origin, target=destination, weight='length')
        total_actual = getTotalLength(G_proj, route_actual)
        max_distance_val = set_max(max_distance_val, total_actual)
        dic["Shortest_Path"].append(total_actual)
        # Best Elevation
        route_actual_elev = nx.shortest_path(G_proj, source=origin, target=destination, weight='grade_abs')
        total_actual_elev = getTotalLength(G_proj, route_actual_elev)
        max_distance_val = set_max(max_distance_val, total_actual_elev)
        dic["Least_Elevation"].append(total_actual_elev)
        # Our Elevation
        extratravelpercent = extrapercent_travel
        can_travel = ((100 + extratravelpercent) * total_actual) / 100
        # in between path between shortest and least elevation under viable length
        route_by_length1 = dijkstra_search(G_proj, origin, destination, can_travel)
        total_cost_mod_dijkstra = getTotalLength(G_proj, route_by_length1)
        max_distance_val = set_max(max_distance_val, total_cost_mod_dijkstra)
        dic["Modified_Dijkstra"].append(total_cost_mod_dijkstra)

        route_minimize_elevation2, route_maximize_elevation2 = dfs_get_all_paths(G_proj, origin, destination,
                                                                                 can_travel)
        if len(route_minimize_elevation2) == 0:
            total_cost_dfs_gap_rmin2 = 0.0
        else:
            total_cost_dfs_gap_rmin2 = getTotalLength(G_proj, route_minimize_elevation2)
        max_distance_val = set_max(max_distance_val, total_cost_dfs_gap_rmin2)
        dic["DFS_1_min_elevation"].append(total_cost_dfs_gap_rmin2)
        if len(route_maximize_elevation2) == 0:
            total_cost_dfs_gap_rmax2 = 0.0
        else:
            total_cost_dfs_gap_rmax2 = getTotalLength(G_proj, route_maximize_elevation2)
        max_distance_val = set_max(max_distance_val, total_cost_dfs_gap_rmax2)
        dic["DFS_1_max_elevation"].append(total_cost_dfs_gap_rmax2)

        shortest_path = get_shortest_path(G_proj, source=origin, target=destination, weight='length')
        route_minimize_elevation3, route_maximize_elevation3 = dfs_get_all_paths_2(G_proj, origin, destination,
                                                                                   can_travel, len(shortest_path) + 4)
        if len(route_minimize_elevation3) == 0:
            total_cost_dfs_gap_rmin3 = 0.0
        else:
            total_cost_dfs_gap_rmin3 = getTotalLength(G_proj, route_minimize_elevation3)
        max_distance_val = set_max(max_distance_val, total_cost_dfs_gap_rmin3)
        dic["DFS_2_min_elevation"].append(total_cost_dfs_gap_rmin3)
        if len(route_maximize_elevation3) == 0:
            total_cost_dfs_gap_rmax3 = 0.0
        else:
            total_cost_dfs_gap_rmax3 = getTotalLength(G_proj, route_maximize_elevation3)
        max_distance_val = set_max(max_distance_val, total_cost_dfs_gap_rmax3)
        dic["DFS_2_max_elevation"].append(total_cost_dfs_gap_rmax3)

        # Test
        len_t = len(dic["Modified_Dijkstra"])
        for each in dic.keys():
            if len_t != len(dic[each]):
                print each
                print len(dic[each])
                assert len_t == len(dic[each])

        shortest.append(total_actual)
        shortest_elev.append(total_actual_elev)
        algo_dist.append(total_cost_mod_dijkstra)
        # cost_data.append([total_actual, total_actual_elev, total_cost_mod_dijkstra, (total_cost_mod_dijkstra/total_actual_elev)*100])
        # print ("%f %f %f %f"%(total_actual, total_actual_elev, total_cost, (total_cost/total_actual_elev)*100))
    # for each in cost_data:
    #     print each
    dic["MAX"] = max_distance_val
    if dump:
        pkl.dump(dic, open(dump_file, "wb"))

    if plot:
        # plot_results(shortest, shortest_elev, algo_dist)
        plot_results_from_dict(dic)

    if plot_individuals:
        for each in dic:
            if each != "MAX":
                temp_dict = {}
                temp_dict["MAX"] = dic["MAX"]
                temp_dict[each] = dic[each]
                plot_results_from_dict(temp_dict, True, each)


G_proj, G = get_map()

compare_algorithms(G, G_proj, origin_lat_long=[37.772, -122.434], bbox_lat_long=(37.772, -122.434),
                   bbox_dist=500, num_dest=5, extrapercent_travel=10, plot=True)


# # Enter Latitude and Longitude to select origin and destination
# origin = ox.get_nearest_node(G, (37.77, -122.426))
# destination = ox.get_nearest_node(G, (37.773, -122.441))
#
# # Enter the percentage you want to travel extra
# extra_travel = 10
# bbox = ox.bbox_from_point((37.772, -122.434), distance=1500, project_utm=True)
#
# shortest_path = get_shortest_path(G_proj, source=origin, target=destination, weight='length')
# print("Printing Statistics of Shortest path route")
# print(shortest_path)
# print_route_stats(G_proj, shortest_path)
#
# shortest_elevation_path = get_shortest_path(G_proj, source=origin, target=destination, weight='elevation')
# print("Printing Statistics of Shortest Elevation path route")
# print_route_stats(G_proj, shortest_elevation_path)
#
# shortest_path_length = getTotalLength(G_proj, shortest_path)
# can_travel = ((100.0 + extra_travel)*shortest_path_length)/100.0
# print("Extra travel: ", can_travel)
#
# #Strategy1
# #As of now, maximize is not working properly, please dont use
# route_minimize_elevation1 = dijkstra_search(G_proj, origin, destination, can_travel, mode='maximize')
# print("Printing Statistics of our algorithm's minimum Elevation route")
# print_route_stats(G_proj, route_minimize_elevation1)
#
# #Strategy2
# import time
# t = time.time()
# route_minimize_elevation2, route_maximize_elevation2 = dfs_get_all_paths(G_proj, origin, destination, can_travel)
# print ("Srikanth's algo time:", time.time()-t)
# print("Printing Statistics of Minimum Elevation route")
# print_route_stats(G_proj, route_minimize_elevation2)
# print("Printing Statistics of Maximum Elevation route")
# print_route_stats(G_proj, route_maximize_elevation2)
#
# #Strategy3
# route_minimize_elevation3, route_maximize_elevation3 = dfs_get_all_paths_2(G_proj, origin, destination, can_travel, len(shortest_path)+4)
# print("Printing Statistics of Minimum Elevation route")
# print_route_stats(G_proj, route_minimize_elevation3)
# print("Printing Statistics of Maximum Elevation route")
# print_route_stats(G_proj, route_maximize_elevation3)
#
# #Strategy4
# #route_by_impedance = nx.shortest_path(G_proj, source=origin, target=destination, weight='impedance')
# #fig, ax = ox.plot_graph_route(G_proj, route_by_length, bbox=bbox, node_size=0)
