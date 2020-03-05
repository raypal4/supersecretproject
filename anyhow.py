import networkx as nx
import osmnx as ox
ox.config(log_console=True, use_cache=True)

G = ox.graph_from_place('Piedmont, CA, USA', network_type='drive')

# pick 4 random nodes as origins/destinations for the 2 routes
orig1 = list(G.nodes())[0]
dest1 = list(G.nodes())[-1]
orig2 = list(G.nodes())[50]
dest2 = list(G.nodes())[-50]

# calculate shortest paths for the 2 routes
route1 = nx.shortest_path(G, orig1, dest1, weight='length')
route2 = nx.shortest_path(G, orig2, dest2, weight='length')

# create route colors
rc1 = ['r'] * (len(route1) - 1)
rc2 = ['b'] * len(route2)
rc = rc1 + rc2
nc = ['r', 'r', 'b', 'b']

print(route1)
print(route2)

# plot the routes
fig, ax = ox.plot_graph_routes(
    G, [route1, route2], route_color=rc, orig_dest_node_color=nc, node_size=0)
