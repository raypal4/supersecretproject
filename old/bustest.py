import osmnx as ox
import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
from IPython.display import IFrame
import heapq as heapq
import geopy.distance
import json
from shapely.geometry import Point, LineString

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

with open('data/busroute0.json') as f:
    geometry = json.load(f)
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
        lat = busGraph.nodes[prev]['y']
        lon = busGraph.nodes[prev]['x']

        clat = busGraph.nodes[current]['y']
        clon = busGraph.nodes[current]['x']

        geo = []
        if service in geometry:
            line = geometry[service]['coordinates']
            flag = None
            for point in line:
                if [lon, lat] == point:
                    flag = 1
                if not flag:
                    geo.append(point)
                    if point == [clon, clat]:
                        break

        if len(geo) != 0:
            busGraph.add_edge(prev, current, service=service, length=distance, direction=direction, geometry=LineString(geo))
        prev = current
        prevdirection = direction
        prevservice = service

edge_labels=dict([((u,v,),d['service'])
                 for u,v,d in busGraph.edges(data=True)])


pos=nx.spring_layout(busGraph)
plt.gca().invert_yaxis()
plt.gca().invert_xaxis()
nx.draw_networkx_edge_labels(busGraph,pos,edge_labels=edge_labels)
nx.draw(busGraph,pos)
plt.show()
