import osmnx as ox
import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
from IPython.display import IFrame
import heapq as heapq
import geopy.distance
import json

ox.config(log_console=True, use_cache=True)

AlgoItterations1 = 0
AlgoItterations2 = 0
AlgoItterations3 = 0
totalWeight = 0


class Graph():
    def __init__(self):
        self.edges = defaultdict(list)
        self.weights = {}

    def add_edge(self, u, v, weight):
        self.edges[u].append(v)
        self.weights[(u, v)] = weight

def get_nodes(edges):
    temp = {}
    list_name = []
    list_osmid = []
    list_length = []
    list_u = []
    list_v = []
    uvd = []
    edge_len = len(edges)
    for i in edges.osmid:
        list_osmid.append(i)
    for i in edges.length:
        list_length.append(i)
    for i in edges.u:
        list_u.append(i)
    for i in edges.v:
        list_v.append(i)
    for i in edges.name:
        list_name.append(i)
    for item in range(edge_len):
        temp[item] = [list_name[item], list_osmid[item],
                      list_length[item], list_u[item], list_v[item]]
    for i, u in temp.items():
        temp_tup = (u[3], u[4], u[2])
        uvd.append(temp_tup)
    return uvd

# ---------------------- Initialising -------------------------------

with open('data/busstop.json') as f:
    data = json.load(f)
    f.close()

busGraph = nx.MultiDiGraph()

for stopNo in data.keys():
    name = data[stopNo]["name"]
    services = data[stopNo]["services"]
    contour = data[stopNo]["contour"]
    busGraph.add_node(stopNo, name=name, services=services, contour=contour)


graph_projected = ox.project_graph(busGraph)


# orig_node1 = ox.get_nearest_node(graph, org1)
# target_node1 = ox.get_nearest_node(graph, dest2)
