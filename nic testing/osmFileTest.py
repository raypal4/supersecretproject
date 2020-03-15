import osmnx as ox
import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
from IPython.display import IFrame
import heapq as heapq
import geopy.distance
from shapely.geometry import Point, LineString
from itertools import count

def _weight_function(G, weight):
	if callable(weight):
		return weight
	if G.is_multigraph():
		return lambda u, v, d: min(attr.get(weight, 1) for attr in d.values())
	return lambda u, v, data: data.get(weight, 1)

def astar_path(G, source, target, weight='weight'):
	if source not in G or target not in G:
		msg = f"Either source {source} or target {target} is not in G"
		raise nx.NodeNotFound(msg)

	def heuristic(u, v):
		u = (G.nodes[u]['y'], G.nodes[u]['x'])
		v = (G.nodes[v]['y'], G.nodes[v]['x'])
		return geopy.distance.distance(u, v).km

	push = heapq.heappush
	pop = heapq.heappop
	weight = _weight_function(G, weight)

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
			# temp = graph[curnode][neighbor][0]
			# print(temp)
			# if "highway" in temp.keys():
			# 	if isinstance(temp["highway"], list):
			# 		flag = 0
			# 		for i in temp["highway"]:
			# 			if i == "bus_stop" or i == "footway" or i == "corridor":
			# 				flag == 1
			# 		if flag != 1:
			# 			continue
			# 	else:
			# 		if not (temp["highway"] == "bus_stop" or temp["highway"] == "footway" or temp["highway"] == "corridor"):
			# 			continue
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

ox.config(log_console=True)
graph = ox.graph_from_file("punggol.osm", bidirectional=False, simplify=True, retain_all=False)

# start = ox.geocode("Singapore, Punggol MRT")
# end = ox.geocode("singapore, Cove Stn")

start = (1.405227, 103.902360)
end = (1.406300, 103.899606)

start_node = ox.get_nearest_node(graph, start)
end_node = ox.get_nearest_node(graph, end)

nodes, edges = ox.graph_to_gdfs(graph)



# temp comment

nodepath = astar_path(graph, start_node, end_node)

prev = None
for i in nodepath:
	if prev:
		print(graph[prev][i])
	prev = i

route_nodes = nodes.loc[nodepath]
print(route_nodes)

graph_projected = ox.project_graph(graph)
# ox.plot.plot_graph(graph_projected)


fig, ax = ox.plot_graph_route(graph_projected, nodepath, origin_point=start,destination_point=end)
plt.tight_layout()