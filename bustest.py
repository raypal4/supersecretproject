import osmnx as ox
import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
from IPython.display import IFrame
import heapq as heapq
import geopy.distance
import json

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
    busGraph.add_node(stopNo, name=name, y=lat, x=lon)

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
        busGraph.add_edge(prev, current, service=service, length=distance, direction=direction)
        prev = current
        prevdirection = direction
        prevservice = service

edge_labels=dict([((u,v,),d['service'])
                 for u,v,d in busGraph.edges(data=True)])


pos=nx.spring_layout(busGraph)
nx.draw_networkx_edge_labels(busGraph,pos,edge_labels=edge_labels)
nx.draw(busGraph,pos)
plt.show()
