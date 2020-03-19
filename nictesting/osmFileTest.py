import osmnx as ox
import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
from IPython.display import IFrame
import heapq as heapq
import geopy.distance
from shapely.geometry import Point, LineString
from itertools import count
import json
import heapq
import folium
import math

from manualPatch.pois import *

# ------------------------------------------INIT START-----------------------------------------------------------------
# ox.config(log_console=True)
print("Loading OSM")
graph = ox.graph_from_file(
    "nictesting\punggol.osm", bidirectional=False, simplify=True, retain_all=False)

print("Loading JSON")
stops = json.loads(open("nictesting/stops.json").read())
services = json.loads(open("nictesting/services.json").read())
routes = json.loads(open("nictesting/routes.json").read())

busRoute0 = json.loads(
    open("nictesting/punggolBusData/busroute0.json").read())
busRoute1 = json.loads(
    open("nictesting/punggolBusData/busroute1.json").read())

print("Initializing tables")
stop_desc_map = {stop["Description"]: stop for stop in stops}
stop_code_map = {stop["BusStopCode"]: stop for stop in stops}

print("Creating bus route map")
routes_map = {}

for route in routes:
    key = (route["ServiceNo"], route["Direction"])
    if key not in routes_map:
        routes_map[key] = []
    # hack around broken data
    if (route["StopSequence"] == 4
        and route["Distance"] == 9.1
            and key == ("34", 1)):
        route["StopSequence"] = 14
    routes_map[key] += [route]

print("Initializing Graph")
busGraph = {}
for service, path in routes_map.items():
    # hack around broken data
    path.sort(key=lambda r: r["StopSequence"])
    for route_index in range(len(path) - 1):
        key = path[route_index]["BusStopCode"]
        if key not in busGraph:
            busGraph[key] = {}
        curr_route_stop = path[route_index]
        next_route_stop = path[route_index + 1]
        curr_distance = curr_route_stop["Distance"] or 0
        next_distance = next_route_stop["Distance"] or curr_distance
        distance = next_distance - curr_distance
        assert distance >= 0, (curr_route_stop, next_route_stop)
        curr_code = curr_route_stop["BusStopCode"]
        next_code = next_route_stop["BusStopCode"]
        busGraph[curr_code][(next_code, service)] = distance

# ------------------------------------------------INIT END-----------------------------------------


# def weightCalc(G, weight):
# 	if callable(weight):
# 		return weight
# 	if G.is_multigraph():
# 		return lambda u, v, d: min(attr.get(weight, 1) for attr in d.values())
# 	return lambda u, v, data: data.get(weight, 1)

def heuristic(G, u, v):
    u = (G.nodes[u]['y'], G.nodes[u]['x'])
    v = (G.nodes[v]['y'], G.nodes[v]['x'])
    return geopy.distance.distance(u, v).km


# def astar_path(G, source, target, weight='weight'):
def astar_path(G, source, target):
    if source not in G or target not in G:
        raise nx.NodeNotFound(
            f"Either source {source} or target {target} is not in G")

    push = heapq.heappush
    pop = heapq.heappop
    # weight = weightCalc(G, weight)
    def weight(u, v, d): return min(attr.get(weight, 1) for attr in d.values())

    c = count()
    queue = [(0, next(c), source, 0, None)]

    enqueued = {}
    explored = {}

    while queue:
        _, __, curnode, dist, parent = pop(queue)

        if curnode == target:
            path = [curnode]
            node = parent
            while node is not None:
                path.append(node)
                node = explored[node]
            path.reverse()
            return path

        if curnode in explored:
            if explored[curnode] is None:
                continue
            qcost, h = enqueued[curnode]
            if qcost < dist:
                continue

        explored[curnode] = parent

        for neighbor, w in G[curnode].items():
            ncost = dist + weight(curnode, neighbor, w)
            if neighbor in enqueued:
                qcost, h = enqueued[neighbor]
                if qcost <= ncost:
                    continue
            else:
                h = heuristic(G, neighbor, target)
            enqueued[neighbor] = ncost, h
            push(queue, (ncost + h, next(c), neighbor, ncost, curnode))

    raise nx.NetworkXNoPath(f"Node {target} not reachable from {source}")


def bfs(graph, start, end):
    cost_per_transfer = 5
    cost_per_stop = 1
    seen = set()
    # maintain a queue of paths
    queue = []
    # push the first path into the queue
    heapq.heappush(queue, (0, 0, 0, [(start, None)]))
    while queue:
        # get the first path from the queue
        (curr_cost, curr_distance, curr_transfers, path) = heapq.heappop(queue)

        # get the last node from the path
        (node, curr_service) = path[-1]

        # path found
        if node == end:
            return (curr_cost, curr_distance, curr_transfers-1, path)

        if (node, curr_service) in seen:
            continue

        seen.add((node, curr_service))
        # enumerate all adjacent nodes, construct a new path and push it into the queue
        for (adjacent, service), distance in graph.get(node, {}).items():
            new_path = list(path)
            new_path.append((adjacent, service))
            new_distance = curr_distance + distance
            new_cost = distance + curr_cost
            new_transfers = curr_transfers
            if curr_service != service:
                new_cost += cost_per_transfer
                new_transfers += 1
            new_cost += cost_per_stop

            heapq.heappush(
                queue, (new_cost, new_distance, new_transfers, new_path))


def bus(busGraph, start, end):
    tags = {
        'highway': 'bus_stop',
    }

    words_rep = {
        "avenue": "ave",
        "block": "blk",
        "boulevard": "blvd",
        "central": "ctrl",
        "close": "cl",
        "crescent": "cres",
        "drive": "dr",
        "expressway": "e'way",
        "highway": "hway",
        "industrial park": "ind park",
        "mount": "mt",
        "place": "pl",
        "ring road": "ring rd",
        "road": "rd",
        "service road": "service rd",
        "square": "sq",
        "station": "stn",
        "street": "st",
        "punggol stn/waterway point": "punggol stn",
        "opposite": "opp",
        "primary": "pr",
        "school": "sch"
    }

    startlat = start[0]
    startLon = start[1]
    R = 6378137
    dn = 50
    de = 50
    dLat = dn/R
    dLon = de/(R*math.cos(math.pi*startlat/180))
    maxstartLat = startlat + dLat * 180/math.pi
    maxstartLon = startLon + dLon * 180/math.pi
    dn = -50
    de = -50
    dLat = dn/R
    dLon = de/(R*math.cos(math.pi*startlat/180))
    minstartLat = startlat + dLat * 180/math.pi
    minstartLon = startLon + dLon * 180/math.pi

    bus = pois_from_polygon(
        box(minstartLon, minstartLat, maxstartLon, maxstartLat), tags=tags)
    # print(bus.columns)
    busStopStart = None
    busStopStartName = None

    for name in bus["name"]:
        name = name.lower()
        for word, initial in words_rep.items():
            name = name.replace(word, initial)
        busStopStartName = name.title()

    endlat = end[0]
    endLon = end[1]
    R = 6378137
    dn = 50
    de = 50
    dLat = dn/R
    dLon = de/(R*math.cos(math.pi*endlat/180))
    maxendLat = endlat + dLat * 180/math.pi
    maxendLon = endLon + dLon * 180/math.pi
    dn = -50
    de = -50
    dLat = dn/R
    dLon = de/(R*math.cos(math.pi*endlat/180))
    minendLat = endlat + dLat * 180/math.pi
    minendLon = endLon + dLon * 180/math.pi

    bus = pois_from_polygon(
        box(minendLon, minendLat, maxendLon, maxendLat), tags=tags)
    # print(bus.columns)
    busStopEnd = None
    busStopEndName = None
    for name in bus["name"]:
        name = name.lower()
        for word, initial in words_rep.items():
            name = name.replace(word, initial)
        busStopEndName = name.title()

    startStation = dict(
        filter(lambda item: busStopStartName in item[0], stop_desc_map.items()))
    endStation = dict(
        filter(lambda item: busStopEndName in item[0], stop_desc_map.items()))

    results = []
    for x in startStation:
        for y in endStation:
            results.append(
                bfs(busGraph, startStation[x]["BusStopCode"], endStation[y]["BusStopCode"]))

    cheapest = None
    cheapestCost = float("Infinity")
    for cost, distance, transfers, path in results:
        if cost < cheapestCost:
            cheapest = (cost, distance, transfers, path)
            cheapestCost = cost

    print(results)
    cost, distance, transfers, path = cheapest
    for code, service in path:
        print(service, stop_code_map[code]["Description"])
    print(len(path)-1, "stops")
    print("cost", cost)
    print("distance", distance, "km")
    print("transfers", transfers)
    return path
    # STORE START AND END BUS STOPS THEN THROW INTO THE BUS ROUTING FUNCTION


# ------------------------------------START OF MAIN--------------------------------------------

# start = ox.geocode("Singapore, Punggol MRT")
# end = ox.geocode("singapore, Cove Stn")
start = (1.40525, 103.90235)
end = (1.39960, 103.91646)

start_node = ox.get_nearest_node(graph, start)
end_node = ox.get_nearest_node(graph, end)

nodes, edges = ox.graph_to_gdfs(graph)

# TO CREATE BUS ROUTING
path = bus(busGraph, start, end)
indexing = 0
line = []
prevService = None
prevIndex = None
i = 0
markers = []
while i < len(path):
    stopCode, service = path[i]
    # in the case of first stop, no bus service stated, take next
    if service is None:
        service = path[i+1][1]

    if service != prevService:
        indexing = 0

    qlat = stop_code_map[stopCode]["Latitude"]
    qlon = stop_code_map[stopCode]["Longitude"]

    # get routes for respective direction
    if service[1] == 1:
        routing = busRoute0[service[0]]["coordinates"]
    else:
        routing = busRoute1[service[0]]["coordinates"]
    while indexing < len(routing):
        clon, clat = routing[indexing]
        u = (qlat, qlon)
        v = (clat, clon)
        # stop found in range of 30 meters, latlong accuracy difference from two sources
        if geopy.distance.distance(u, v).km < 0.03:
            # first bus stop
            if prevService is None:
                line.append(v)
            else:
                if prevService == service:
                    for x, y in routing[prevIndex: indexing+1]:
                        line.append((y, x))
                else:
                    prevLatLong = line[-1]
                    tempIndex = 0
                    while tempIndex < len(routing):
                        plon, plat = routing[tempIndex]
                        p = (plat, plon)
                        if geopy.distance.distance(prevLatLong, p).km < 0.03:
                            for x, y in routing[tempIndex: indexing+1]:
                                line.append((y, x))
                            break
                        tempIndex += 1
            prevIndex = indexing
            prevService = service
            markers.append((v, stopCode))
            break
        indexing += 1
    i += 1
# print(line)

# TO CREATE WALK ROUTING
nodepath = astar_path(graph, start_node, end_node)
print(line)

# TO DISPLAY ROUTE ON MATPLOTLIB
fig, ax = ox.plot_graph_route(graph, nodepath, fig_height=10, fig_width=10,
                              orig_dest_node_color='green', route_color='green', show=False, close=False)
ax.scatter(start[1], start[0], c='red', s=50)
ax.scatter(end[1], end[0], c='blue', s=50)
plt.show()

# FOLIUM
# m = ox.plot_route_folium(graph, nodepath, route_color='green')
# opacity 0 just to make the driving line disappear
m = ox.plot_route_folium(graph, nodepath, route_color='green', route_opacity=0)
folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
    color='red', icon='flag')).add_to(m)
folium.Marker(location=(end[0], end[1]), popup='END',
              icon=folium.Icon(color='blue', icon='flag')).add_to(m)
for loc, code in markers:
    folium.Marker(location=loc, popup='Bus stop number:'+str(code),
                  icon=folium.Icon(color='green', icon='bus', prefix='fa')).add_to(m)
folium.PolyLine(line, color="red", weight=2.5, opacity=1).add_to(m)
folium.PolyLine([line[0], start], color="blue", weight=2.5,
                opacity=1, dasharray="4").add_to(m)
folium.PolyLine([line[-1], end], color="blue", weight=2.5,
                opacity=1, dasharray="4").add_to(m)
m
