from django.shortcuts import render

from django.http import HttpResponseRedirect
from .forms import AddressForm

import osmnx as ox
import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
from IPython.display import IFrame
import heapq as heapq
import geopy.distance
from shapely.geometry import Point, LineString

import folium
import folium.plugins
from geopy.geocoders import Nominatim

ox.config(log_console=True, use_cache=True)

AlgoItterations1 = 0

def index(request):
	if request.method == 'POST':
		retrieveForm = AddressForm(request.POST)
		if retrieveForm.is_valid():
			org_var = retrieveForm.cleaned_data['var_org']
			dst_var = retrieveForm.cleaned_data['var_dst']
			
			geolocator = Nominatim(user_agent="test")
			org_addr = geolocator.geocode(org_var)
			dst_addr = geolocator.geocode(dst_var)
			org = (org_addr.latitude, org_addr.longitude)
			dest = (dst_addr.latitude, dst_addr.longitude)
			
			if org:
				graph1 = ox.graph_from_point(org, distance=2000, network_type='drive')
				graph2 = ox.graph_from_point(org, distance=2000, network_type='walk')
				graph = nx.compose(graph2, graph1)
			else:
				(graph1, org) = ox.graph_from_address(address, distance=5000, network_type='walk', return_coords=True)
				graph2 = ox.graph_from_address(address, distance=5000, network_type='drive')
				graph = nx.compose(graph1, graph2)
			
			# get cloest node to the point of search
			orig_node = ox.get_nearest_node(graph, org)
			target_node = ox.get_nearest_node(graph, dest)
			
			nodes, edges = ox.graph_to_gdfs(graph)
			
			node_data = get_nodes(edges)
			ourRoute2 = list(creator2(node_data, orig_node, target_node))
				
			route_map = ox.plot_route_folium(graph, ourRoute2, popup_attribute='name', tiles='openstreetmap' ,route_color='blue')
			folium.Marker(location=org, popup=org_addr.address, icon=folium.Icon(color='red')).add_to(route_map)
			folium.Marker(location=dest, popup=dst_addr.address, icon=folium.Icon(color='blue')).add_to(route_map)
			filepath = 'mapper/route.html'
			route_map.save(filepath)
			return HttpResponseRedirect('../result/')

	else:
		inputForm = AddressForm()
		return render(request,"index.html",{'form':inputForm})

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

def creator2(node_data, orig_node, target_node):
    graphs = Graph()
    for edge in node_data:
        graphs.add_edge(*edge)
    j = dijsktra(graphs, orig_node, target_node)
    return j

def getDistanceTravelled(nodes, node_data, route):
    sum = 0
    for i in range(0, len(route) - 1):
        for n in node_data:
            if(n[0] == route[i] and n[1] == route[i+1]):
                sum += n[2] * 100
    return sum