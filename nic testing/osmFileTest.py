import osmnx as ox
import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
from IPython.display import IFrame
import heapq as heapq
import geopy.distance
from shapely.geometry import Point, LineString
from itertools import count

from manualPatch.pois import *

def weightCalc(G, weight):
	if callable(weight):
		return weight
	if G.is_multigraph():
		return lambda u, v, d: min(attr.get(weight, 1) for attr in d.values())
	return lambda u, v, data: data.get(weight, 1)

def heuristic(u, v):
	u = (G.nodes[u]['y'], G.nodes[u]['x'])
	v = (G.nodes[v]['y'], G.nodes[v]['x'])
	return geopy.distance.distance(u, v).km


def astar_path(G, source, target, weight='weight'):
	if source not in G or target not in G:
		msg = f"Either source {source} or target {target} is not in G"
		raise nx.NodeNotFound(msg)

	push = heapq.heappush
	pop = heapq.heappop
	weight = weightCalc(G, weight)

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
				h = heuristic(neighbor, target)
			enqueued[neighbor] = ncost, h
			push(queue, (ncost + h, next(c), neighbor, ncost, curnode))

	raise nx.NetworkXNoPath(f"Node {target} not reachable from {source}")

def bus(start, end):
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
	print(bus.columns)
	
	for name in bus["name"]:
		print(name)
		name = name.lower()
		for word, initial in words_rep.items():
			name = name.replace(word, initial)
		name = name.title()
		print(name)

	endX = end[1]
	endY = end[0]

	bus = pois_from_polygon(box(endX-0.008999,endY-0.000094, endX+0.008999, endY+0.000094), tags=tags)
	print(bus.columns)

	for name in bus["name"]:
		print(name)
		name = name.lower()
		for word, initial in words_rep.items():
			name = name.replace(word, initial)
		name = name.title()
		print(name)

	# STORE START AND END BUS STOPS THEN THROW INTO THE BUS ROUTING FUNCTION


ox.config(log_console=True)
graph = ox.graph_from_file("punggol.osm", bidirectional=False, simplify=True, retain_all=False)

# start = ox.geocode("Singapore, Punggol MRT")
# end = ox.geocode("singapore, Cove Stn")

start = (1.40525, 103.90235)
end = (1.39960, 103.91646)

start_node = ox.get_nearest_node(graph, start)
end_node = ox.get_nearest_node(graph, end)

nodes, edges = ox.graph_to_gdfs(graph)

nodepath = astar_path(graph, start_node, end_node)

prev = None
for i in nodepath:
	if prev:
		pass
		# print(graph[prev][i])
	prev = i

route_nodes = nodes.loc[nodepath]
# print(route_nodes)

bus(start, end)

graph_projected = ox.project_graph(graph)
# ox.plot.plot_graph(graph_projected)


fig, ax = ox.plot_graph_route(graph_projected, nodepath, origin_point=start,destination_point=end)
plt.tight_layout()