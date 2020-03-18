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

from manualPatch.pois import *

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
		raise nx.NodeNotFound(f"Either source {source} or target {target} is not in G")

	push = heapq.heappush
	pop = heapq.heappop
	# weight = weightCalc(G, weight)
	weight = lambda u, v, d: min(attr.get(weight, 1) for attr in d.values())

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

			heapq.heappush(queue, (new_cost, new_distance, new_transfers, new_path))

def bus(busGraph, start, end):
	tags = {
		'highway':'bus_stop',
	}

	words_rep = {
		"avenue":"ave",
		"block":"blk",
		"boulevard":"blvd",
		"central":"ctrl",
		"close":"cl",
		"crescent":"cres",
		"drive":"dr",
		"expressway":"e'way",
		"highway":"hway",
		"industrial park":"ind park",
		"mount":"mt",
		"place":"pl",
		"ring road":"ring rd",
		"road":"rd",
		"service road":"service rd",
		"square":"sq",
		"station":"stn",
		"street":"st"
		}

	startX = start[1]
	startY = start[0]
	bus = pois_from_polygon(box(startX-0.008999,startY-0.000094, startX+0.008999, startY+0.000094), tags=tags)
	# print(bus.columns)
	busStopStart = None

	for name in bus["name"]:
		name = name.lower()
		for word, initial in words_rep.items():
			name = name.replace(word, initial)
		busStopStart = name.title()
		print(busStopStart)

	endX = end[1]
	endY = end[0]

	bus = pois_from_polygon(box(endX-0.008999,endY-0.000094, endX+0.008999, endY+0.000094), tags=tags)
	# print(bus.columns)
	busStopEnd = None
	for name in bus["name"]:
		name = name.lower()
		for word, initial in words_rep.items():
			name = name.replace(word, initial)
		busStopEnd = name.title()
		print(busStopEnd)

	(cost, distance, transfers, path) = bfs(busGraph, stop_desc_map[busStopStart]["BusStopCode"], stop_desc_map[busStopEnd]["BusStopCode"])
	for code, service in path:
		print(service, stop_code_map[code]["Description"])
	print (len(path)-1, "stops")
	print ("cost", cost)
	print ("distance", distance, "km")
	print ("transfers", transfers)
	# STORE START AND END BUS STOPS THEN THROW INTO THE BUS ROUTING FUNCTION

# ------------------------------------------INIT START-----------------------------------------------------------------
# ox.config(log_console=True)
print("Loading OSM")
graph = ox.graph_from_file("punggol.osm", bidirectional=False, simplify=True, retain_all=False)

print("Loading JSON")
stops = json.loads(open("stops.json").read())
services = json.loads(open("services.json").read())
routes = json.loads(open("routes.json").read())

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
	path.sort(key = lambda r: r["StopSequence"])
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

# start = ox.geocode("Singapore, Punggol MRT")
# end = ox.geocode("singapore, Cove Stn")

start = (1.40525, 103.90235)
end = (1.39960, 103.91646)

start_node = ox.get_nearest_node(graph, start)
end_node = ox.get_nearest_node(graph, end)

nodes, edges = ox.graph_to_gdfs(graph)

# TO CREATE BUS ROUTING
bus(busGraph, start, end)

# TO CREATE WALK ROUTING
nodepath = astar_path(graph, start_node, end_node)


# TO DISPLAY ROUTE ON MAP
graph_projected = ox.project_graph(graph)
fig, ax = ox.plot_graph_route(graph_projected, nodepath, origin_point=start,destination_point=end)
plt.tight_layout()