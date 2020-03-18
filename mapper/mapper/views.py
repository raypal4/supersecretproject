from django.shortcuts import render

from django.http import HttpResponseRedirect
from .forms import AddressForm
from django.template import Context, Template

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

import json
from mysite.settings import JSON_FOLDER, ROUTE_FOLDER

ox.config(log_console=True, use_cache=True)

AlgoItterations1 = 0
AlgoItterations2 = 0
AlgoItterations3 = 0
totalWeight = 0

def index(request):
	if request.method == 'POST':
		retrieveForm = AddressForm(request.POST)
		if retrieveForm.is_valid():
			org_var = retrieveForm.cleaned_data['var_org'] #828867
			dst_var = retrieveForm.cleaned_data['var_dst'] #824679
			
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
			global target_node
			orig_node = ox.get_nearest_node(graph, org)
			target_node = ox.get_nearest_node(graph, dest)
			
			with open(JSON_FOLDER + 'punggol_bus_stops.json') as f:
				data = json.load(f)
				f.close()

			busGraph = nx.MultiDiGraph()

			for stop in data:
				stopNo = stop["BusStopCode"]
				name = stop["Description"]
				lat = stop["Latitude"]
				lon = stop["Longitude"]
				busGraph.add_node(stopNo, name=name, y=lat, x=lon)

			with open(JSON_FOLDER + 'punggol_bus_routes.json') as f:
				data = json.load(f)
				f.close()

			with open(JSON_FOLDER + 'busroute0.json') as f:
				geometry = json.load(f)
				f.close()

			prev = 0
			prevdirection = 0
			prevservice = 0
			for route in data:
				current = route["BusStopCode"]
				service = route["ServiceNo"]
				distance = route["Distance"]
				direction = route["Direction"]

				if distance == 0:
					prev = current
					prevdirection = direction
					prevservice = service
					continue
				else:
					lat = busGraph.nodes[prev]['y']
					lon = busGraph.nodes[prev]['x']

					clat = busGraph.nodes[current]['y']
					clon = busGraph.nodes[current]['x']

					geo = []
					if service in geometry:
						line = geometry[service]['coordinates']
						flag = None
						for point in line:
							if [lon, lat] == point:
								flag = 1
							if not flag:
								geo.append(point)
								if point == [clon, clat]:
									break

					if len(geo) != 0:
						busGraph.add_edge(prev, current, service=service, length=distance, direction=direction, geometry=LineString(geo))
					prev = current
					prevdirection = direction
					prevservice = service

			graph = nx.compose_all([busGraph, graph1, graph2])
			
			nodes, edges = ox.graph_to_gdfs(graph)
			
			node_data = get_nodes(edges)
			ourRoute = list(creator(node_data, orig_node, target_node))
			ourRoute2 = list(creator2(node_data, orig_node, target_node))
			ourRoute3 = list(creator3(nodes, node_data, orig_node, target_node))
			
			# usage of folium to create interactive web map
			dijskra_map = ox.plot_route_folium(graph, ourRoute2, popup_attribute='name', tiles='openstreetmap' ,route_color='blue')
			pdijskra_map = ox.plot_route_folium(graph, ourRoute, popup_attribute='name', tiles='openstreetmap' ,route_color='blue')
			astar_map = ox.plot_route_folium(graph, ourRoute3, popup_attribute='name', tiles='openstreetmap' ,route_color='blue')
			folium.Marker(location=org, popup=org_addr.address, icon=folium.Icon(color='red')).add_to(dijskra_map)
			folium.Marker(location=dest, popup=dst_addr.address, icon=folium.Icon(color='blue')).add_to(dijskra_map)
			folium.Marker(location=org, popup=org_addr.address, icon=folium.Icon(color='red')).add_to(pdijskra_map)
			folium.Marker(location=dest, popup=dst_addr.address, icon=folium.Icon(color='blue')).add_to(pdijskra_map)
			folium.Marker(location=org, popup=org_addr.address, icon=folium.Icon(color='red')).add_to(astar_map)
			folium.Marker(location=dest, popup=dst_addr.address, icon=folium.Icon(color='blue')).add_to(astar_map)
			filepath1 = ROUTE_FOLDER + 'dijskra_route.html'
			filepath2 = ROUTE_FOLDER + 'pdijskra_route.html'
			filepath3 = ROUTE_FOLDER + 'astar_route.html'
			dijskra_map.save(filepath1)
			pdijskra_map.save(filepath2)
			astar_map.save(filepath3)
			#return HttpResponseRedirect('../result/')
			contentDict = {"org_addr": org_addr.address, "dst_addr": dst_addr.address, "node_dijkstra": len(ourRoute2), "node_pdijkstra": len(ourRoute), "node_astar": len(ourRoute3), 
			"itterations_dijkstra": AlgoItterations1, "itterations_pdijkstra": AlgoItterations2, "itterations_astar": AlgoItterations3, "dist_dijkstra": getDistanceTravelled(nodes, node_data, ourRoute2)
			, "dist_pdijkstra": getDistanceTravelled(nodes, node_data, ourRoute), "dist_astar": getDistanceTravelled(nodes, node_data, ourRoute3)}
			return render(request, "main.html", contentDict)

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