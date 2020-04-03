import heapq
from itertools import count

import geopy.distance
import networkx as nx


def heuristic(G, u, v):
	u = (G.nodes[u]['y'], G.nodes[u]['x'])
	v = (G.nodes[v]['y'], G.nodes[v]['x'])
	return geopy.distance.distance(u, v).km


def astar_path(G, source, target):
	if source not in G or target not in G:
		raise nx.NodeNotFound(
			f"Either source {source} or target {target} is not in G")

	push = heapq.heappush
	pop = heapq.heappop

	def weight(u, v, d):
		return min(attr.get(weight, 1) for attr in d.values())

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
