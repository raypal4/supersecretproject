import osmnx as ox
import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
from IPython.display import IFrame
ox.config(log_console=True, use_cache=True)


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

    while current_node != end:
        visited.add(current_node)
        destinations = graphs.edges[current_node]
        weight_to_current_node = shortest_paths[current_node][1]

        for next_node in destinations:
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
            return "Route Not Possible"
        # next node is the destination with the lowest weight
        current_node = min(next_destinations,
                           key=lambda k: next_destinations[k][1])

    # Work back through destinations in shortest path
    path = []
    while current_node is not None:
        path.append(current_node)
        next_node = shortest_paths[current_node][0]
        current_node = next_node
    # Reverse path
    path = path[::-1]
    return path


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
    graphs = Graph()
    for edge in node_data:
        graphs.add_edge(*edge)
    j = dijsktra(graphs, orig_node, target_node)
    return j


org = (1.394290, 103.913011)
dest = (1.410208, 103.905988)

graph = ox.graph_from_point(org, distance=1000, network_type='walk')
graph_projected = ox.project_graph(graph)

# get cloest node to the point of search
orig_node = ox.get_nearest_node(graph, org)
target_node = ox.get_nearest_node(graph, dest)

nodes, edges = ox.graph_to_gdfs(graph)

# just a test route using default dijkstra
Testroute = nx.shortest_path(graph, source=orig_node,
                             target=target_node, weight='length', method='dijkstra')

node_data = get_nodes(edges)
ourRoute = list(creator(node_data, orig_node, target_node))
print("Number of nodes (Our route): ", len(ourRoute))
print("Number of nodes (Test route): ", len(Testroute))

print(ourRoute)
print(Testroute)

# # --------------------------- PLotting -------------------------------------------

# create route colors
rc1 = ['r'] * (len(ourRoute) - 1)
rc2 = ['b'] * len(Testroute)
rc = rc1 + rc2
nc = ['r', 'r', 'b', 'b']

# plot the routes
fig, ax = ox.plot_graph_routes(
    graph, [Testroute, ourRoute], route_color=rc, orig_dest_node_color=nc, node_size=0)
