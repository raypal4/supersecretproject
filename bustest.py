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

with open('test/punggol_bus_stops.json') as f:
    data = json.load(f)
    f.close()

busGraph = nx.MultiDiGraph()

for stop in data:
    stopNo = stop["BusStopCode"]
    name = stop["Description"]
    lat = stop["Latitude"]
    lon = stop["Longitude"]
    busGraph.add_node(stopNo, name=name, lat=lat, long=lon)

with open('test/punggol_bus_routes.json') as f:
    data = json.load(f)
    f.close()

prev = 0
prevdirection = 0
prevservice = 0
for route in data:
    current = route["BusStopCode"]
    service = route["ServiceNo"]
    distance = route["Distance"]
    direction = route["Direction"]

    if distance == 0:
        prev = current
        prevdirection = direction
        prevservice = service
        continue
    else:
        busGraph.add_edge(prev, current, service=service, distance=distance, direction=direction)
        # ----------to visualise------------
        # import re
        # service = re.sub("[A-Za-z]+", "", service);
        # busGraph.add_edge(prev, current, weight=int(service), distance=distance, direction=direction)
        prev = current
        prevdirection = direction
        prevservice = service

edge_labels=dict([((u,v,),d['service'])
                 for u,v,d in busGraph.edges(data=True)])

pos=nx.spring_layout(busGraph)
nx.draw_networkx_edge_labels(busGraph,pos,edge_labels=edge_labels)
nx.draw(busGraph,pos)
plt.show()
