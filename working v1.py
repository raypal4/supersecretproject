import osmnx as ox
import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
from IPython.display import IFrame
import heapq as heapq
import geopy.distance

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


def dijsktra(graphs, initial, end):
    # shortest paths is a dict of nodes
    # whose value is a tuple of (previous node, weight)
    shortest_paths = {initial: (None, 0)}
    current_node = initial
    visited = set()
    global AlgoItterations1

    while current_node != end:
        visited.add(current_node)
        destinations = graphs.edges[current_node]
        weight_to_current_node = shortest_paths[current_node][1]

        for next_node in destinations:
            AlgoItterations1 += 1
            weight = graphs.weights[(
                current_node, next_node)] + weight_to_current_node
            if next_node not in shortest_paths:
                shortest_paths[next_node] = (current_node, weight)
            else:
                current_shortest_weight = shortest_paths[next_node][1]
                if current_shortest_weight > weight:
                    shortest_paths[next_node] = (current_node, weight)

        next_destinations = {
            node: shortest_paths[node] for node in shortest_paths if node not in visited}
        if not next_destinations:
            return "Route is Not Possible"
        # next node is the destination with the lowest weight
        current_node = min(next_destinations,
                           key=lambda k: next_destinations[k][1])

    # Work back through destinations in shortest path
    path = []
    weightSum = 0
    while current_node is not None:
        AlgoItterations1 += 1
        path.append(current_node)
        next_node = shortest_paths[current_node][0]
        current_node = next_node
    # Reverse path
    path = path[::-1]
    return path


def prioritydijsktra(node_data, initial, end):
    global AlgoItterations2
    g = defaultdict(list)
    for e1, e2, cost in node_data:
        g[e1].append((cost, e2))

    pq = [(0, initial, ())]
    seen = set()
    mins = {initial: 0}
    while len(pq) > 0:
        (cost, v1, path) = heapq.heappop(pq)
        if v1 not in seen:
            seen.add(v1)
            path += (v1, )
            if v1 == target_node:
                return path

            for c, v2 in g.get(v1, ()):
                AlgoItterations2 += 1
                prev = mins.get(v2, None)
                next = cost + c
                if prev is None or next < prev:
                    mins[v2] = next
                    heapq.heappush(pq, (next, v2, path))

    return float("Infinity")

def astar(nodes, node_data, initial, end):
    global AlgoItterations3
    g = defaultdict(list)
    
    targetx = nodes.x[end]
    targety = nodes.y[end]
    target_coord = (targety, targetx)
    initialx = nodes.x[initial]
    initialy = nodes.y[initial]
    init_coord = (initialy, initialx)

    for e1, e2, cost in node_data:
        g[e1].append((cost, e2))
    pq = [(0, initial, ())]
    seen = set()
    mins = {initial: 0}
    while len(pq) > 0:
        (cost, v1, path) = heapq.heappop(pq)
        if v1 not in seen:
            seen.add(v1)
            path += (v1, )
            if v1 == target_node:
                return path
            for c, v2 in g.get(v1, ()):
                AlgoItterations3 += 1
                x = nodes.x[v2]
                y = nodes.y[v2]
                current_coord = (y,x)
                prev = mins.get(v2, None)
                next = cost + c*100 + geopy.distance.distance(target_coord, current_coord).km
                if prev is None or next < prev:
                    mins[v2] = next
                    heapq.heappush(pq, (next, v2, path))
    return float("Infinity")

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


def creator(node_data, orig_node, target_node):
    j = prioritydijsktra(node_data, orig_node, target_node)
    return j


def creator2(node_data, orig_node, target_node):
    graphs = Graph()
    for edge in node_data:
        graphs.add_edge(*edge)
    j = dijsktra(graphs, orig_node, target_node)
    return j

def creator3(nodes, node_data, orig_node, target_node):
    j = astar(nodes, node_data, orig_node, target_node)
    return j

def getDistanceTravelled(nodes, node_data, route):
    sum = 0
    for i in range(0, len(route) - 1):
        for n in node_data:
            if(n[0] == route[i] and n[1] == route[i+1]):
                sum += n[2] * 100
    return sum

# ---------------------- Initialising -------------------------------
# org = (1.394290, 103.913011)
# dest = (1.410208, 103.905988)
org = (1.40130, 103.90920)
dest = (1.39620, 103.91270)

# # for tester data: 2 diff train station
# org1 = (1.4052523, 103.9085982)
# dest2 = (1.3996010, 103.9164448)

# drive walk
graph1 = ox.graph_from_point(org, distance=1000, network_type='drive')
graph2 = ox.graph_from_point(org, distance=1000, network_type='walk')
graph = nx.compose(graph2, graph1)

graph_projected = ox.project_graph(graph)

# get cloest node to the point of search
orig_node = ox.get_nearest_node(graph, org)
target_node = ox.get_nearest_node(graph, dest)

# orig_node1 = ox.get_nearest_node(graph, org1)
# target_node1 = ox.get_nearest_node(graph, dest2)

nodes, edges = ox.graph_to_gdfs(graph)

# # just a test route using default dijkstra
# Testroute = nx.shortest_path(graph, source=orig_node1,
#                              target=target_node1, weight='length', method='dijkstra')

node_data = get_nodes(edges)
ourRoute = list(creator(node_data, orig_node, target_node))
ourRoute2 = list(creator2(node_data, orig_node, target_node))
ourRoute3 = list(creator3(nodes, node_data, orig_node, target_node))


print("\nDijkstra Number of nodes (yellow):" , len(ourRoute2)," | algo it:", AlgoItterations1, " | distance:", getDistanceTravelled(nodes, node_data, ourRoute2))

print("\nPriority Dijkstra Number of nodes (red):", len(ourRoute)," | algo it:", AlgoItterations2, " | distance:", getDistanceTravelled(nodes, node_data, ourRoute))

print("\nA-Star Number of nodes (blue):", len(ourRoute3), " | algo it:", AlgoItterations3, " | distance:", getDistanceTravelled(nodes, node_data, ourRoute3))

# print("\nNumber of nodes (Test route):", len(Testroute))

# --------------------------- PLotting -------------------------------------------

route_list = [ourRoute2, ourRoute, ourRoute3]

# create route colors
list_of_colors = ['red', 'yellow', 'blue']
color_list = []

for i in range(len(route_list)):
    num_lines = len(route_list[i]) - 1
    color_elements = num_lines * [list_of_colors[i]]
    color_list = color_list + color_elements

# plot the routes
fig, ax = ox.plot_graph_routes(
    graph_projected, route_list, route_color=color_list)
