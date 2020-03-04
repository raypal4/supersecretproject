import osmnx as ox
import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
from IPython.display import IFrame
ox.config(log_console=True, use_cache=True)

temp = {}
list_name = []
list_osmid = []
list_length = []
list_u = []
list_v = []
uvd = []
org = (1.394290, 103.913011)
dest = (1.410208, 103.905988)
graph = ox.graph_from_point(org, distance=2000, network_type='drive')
#temp_poi = ox.pois_from_point(org, distance=10, amenities=yes)
cols = ['osmid', 'name']
#temp_loc = temp_poi[cols]
graph_projected = ox.project_graph(graph)
orig_node = ox.get_nearest_node(graph, org)
target_node = ox.get_nearest_node(graph, dest)
def creator():
        graphs = Graph()
        for edge in uvd:
            graphs.add_edge(*edge)

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
                    weight = graphs.weights[(current_node, next_node)] + weight_to_current_node
                    if next_node not in shortest_paths:
                        shortest_paths[next_node] = (current_node, weight)
                    else:
                        current_shortest_weight = shortest_paths[next_node][1]
                        if current_shortest_weight > weight:
                            shortest_paths[next_node] = (current_node, weight)
        
                next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
                if not next_destinations:
                    return "Route Not Possible"
                # next node is the destination with the lowest weight
                current_node = min(next_destinations, key=lambda k: next_destinations[k][1])
    
            # Work back through destinations in shortest path
            path = []
            while current_node is not None:
                path.append(current_node)
                next_node = shortest_paths[current_node][0]
                current_node = next_node
            # Reverse path
            path = path[::-1]
            return path
            #return  ', '.join(path)
          
        j = dijsktra(graphs, orig_node, target_node)
        return j
class Graph():
    def __init__(self):
        self.edges = defaultdict(list)
        self.weights = {}
    
    def add_edge(self, u, v, weight):
        # Note: assumes edges are bi-directional
        self.edges[u].append(v)
        self.weights[(u, v)] = weight
nodes, edges = ox.graph_to_gdfs(graph)
route = nx.shortest_path(graph, source=orig_node, target=target_node, weight='length', method='dijkstra')
#node_route = nodes.columns  # [route]
edge_len = len(edges)
#col = ['name', 'osmid', 'length', 'u', 'v']

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
    temp[item] = [list_name[item],list_osmid[item], list_length[item], list_u[item], list_v[item]]
for i,u in temp.items():
    temp_tup = (u[3],u[4],u[2])
    uvd.append(temp_tup)
n = creator()
print(n)
print(route)
fig, ax = ox.plot_graph_route(graph_projected, n, fig_height=10, fig_width=10, save=True, file_format='svg')
# you can also plot/save figures as SVGs to work with in Illustrator later
#fig, ax = ox.plot_graph_route(graph_projected, n, fig_height=10, fig_width=10, save=True, file_format='svg')
#fig, ax = ox.plot_graph(graph_projected, fig_height=10, fig_width=10, save=True, file_format='svg')
