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
import math
from itertools import count
from .pois import *

ox.config(log_console=True, use_cache=True)

AlgoItterations1 = 0
AlgoItterations2 = 0
AlgoItterations3 = 0
totalWeight = 0

stop_desc_map = {}
stop_code_map = {}

def index(request):
	if request.method == 'POST':
		retrieveForm = AddressForm(request.POST)
		if retrieveForm.is_valid():
			org_var = retrieveForm.cleaned_data['var_org'] #828867
			dst_var = retrieveForm.cleaned_data['var_dst'] #824679
			type_var = retrieveForm.cleaned_data['var_type']
			
			geolocator = Nominatim(user_agent="test")
			org_addr = geolocator.geocode(org_var)
			dst_addr = geolocator.geocode(dst_var)
			#org = (org_addr.latitude, org_addr.longitude)
			#dest = (dst_addr.latitude, dst_addr.longitude)
			
			# doesnt work with random locations
			org = (1.40525, 103.90235)
			dest = (1.39960, 103.91646)
			
			if type_var == 'Walk' or type_var == 'Drive':
				if type_var == 'Walk':
					if org:
						graph = ox.graph_from_point(org, distance=2000, network_type='walk')
					else:
						(graph, org) = ox.graph_from_address(address, distance=5000, network_type='walk', return_coords=True)
				elif type_var == 'Drive':
					if org:
						graph = ox.graph_from_point(org, distance=2000, network_type='drive')
					else:
						graph = ox.graph_from_address(address, distance=5000, network_type='drive')
				
				# get cloest node to the point of search
				global target_node
				orig_node = ox.get_nearest_node(graph, org)
				target_node = ox.get_nearest_node(graph, dest)
				
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
				contentDict = {"org_addr": org_addr.address, "dst_addr": dst_addr.address, "node_dijkstra": len(ourRoute2), "node_pdijkstra": len(ourRoute), "node_astar": len(ourRoute3), 
				"itterations_dijkstra": AlgoItterations1, "itterations_pdijkstra": AlgoItterations2, "itterations_astar": AlgoItterations3, "dist_dijkstra": getDistanceTravelled(nodes, node_data, ourRoute2)
				, "dist_pdijkstra": getDistanceTravelled(nodes, node_data, ourRoute), "dist_astar": getDistanceTravelled(nodes, node_data, ourRoute3), "mode_var": type_var}
				return render(request, "main.html", contentDict)
			elif type_var == 'Bus':
				print("Loading OSM")
				graph = ox.graph_from_file(JSON_FOLDER + "punggol.osm", bidirectional=False, simplify=True, retain_all=False)
				print("Loading JSON")
				stops = json.loads(open(JSON_FOLDER + "stops.json").read())
				services = json.loads(open(JSON_FOLDER + "services.json").read())
				routes = json.loads(open(JSON_FOLDER + "routes.json").read())
				busRoute0 = json.loads(open(JSON_FOLDER + "busroute0.json").read())
				busRoute1 = json.loads(open(JSON_FOLDER + "busroute1.json").read())
				
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
				
				start_node = ox.get_nearest_node(graph, org)
				end_node = ox.get_nearest_node(graph, dest)

				nodes, edges = ox.graph_to_gdfs(graph)
				
				# TO CREATE BUS ROUTING
				path = bus(busGraph, org, dest)
				indexing = 0
				line = []
				prevService = None
				prevIndex = None
				i = 0
				markers = []
				while i < len(path):
					stopCode, service = path[i]
					# in the case of first stop, no bus service stated, take next
					if service is None:
						service = path[i+1][1]

					if service != prevService:
						indexing = 0

					qlat = stop_code_map[stopCode]["Latitude"]
					qlon = stop_code_map[stopCode]["Longitude"]

					# get routes for respective direction
					if service[1] == 1:
						routing = busRoute0[service[0]]["coordinates"]
					else:
						routing = busRoute1[service[0]]["coordinates"]
					while indexing < len(routing):
						clon, clat = routing[indexing]
						u = (qlat, qlon)
						v = (clat, clon)
						# stop found in range of 30 meters, latlong accuracy difference from two sources
						if geopy.distance.distance(u, v).km < 0.03:
							# first bus stop
							if prevService is None:
								line.append(v)
							else:
								if prevService == service:
									for x, y in routing[prevIndex: indexing+1]:
										line.append((y,x))
								else:
									prevLatLong = line[-1]
									tempIndex = 0
									while tempIndex < len(routing):
										plon, plat = routing[tempIndex]
										p = (plat, plon)
										if geopy.distance.distance(prevLatLong, p).km < 0.03:
											for x, y in routing[tempIndex: indexing+1]:
												line.append((y,x))
											break
										tempIndex += 1
							prevIndex = indexing
							prevService = service
							markers.append((v, stopCode))
							break
						indexing += 1
					i += 1

				# TO CREATE WALK ROUTING
				nodepath = astar_path(graph, start_node, end_node)

				# usage of folium to create interactive web map
				bus_map = ox.plot_route_folium(graph, nodepath, route_color='green', route_opacity=0) # opacity 0 just to make the driving line disappear
				folium.Marker(location=(start[0],start[1]),popup='START', icon=folium.Icon(color='red', icon='flag')).add_to(bus_map)
				folium.Marker(location=(end[0],end[1]),popup='END', icon=folium.Icon(color='blue', icon='flag')).add_to(bus_map)
				for loc, code in markers:
					folium.Marker(location=loc, popup='Bus stop number:'+str(code), icon=folium.Icon(color='green', icon='bus', prefix='fa')).add_to(bus_map)
				folium.PolyLine(line, color="red", weight=2.5, opacity=1).add_to(bus_map)
				folium.PolyLine([line[0], start], color="blue", weight=2.5, opacity=1, dasharray="4").add_to(bus_map)
				folium.PolyLine([line[-1], end], color="blue", weight=2.5, opacity=1, dasharray="4").add_to(bus_map)
				filepath4 = ROUTE_FOLDER + 'bus_route.html'
				m.save(filepath4)
				contentDict = {"org_addr": org_addr.address, "dst_addr": dst_addr.address, "mode_var": type_var, "service" : service,
				"code" : code, "path" : path, "stop_code_map" : stop_code_map, "distance" : distance, "transfers" : transfers}
				return render(request, "busmain.html", contentDict)

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

def heuristic(G, u, v):
	u = (G.nodes[u]['y'], G.nodes[u]['x'])
	v = (G.nodes[v]['y'], G.nodes[v]['x'])
	return geopy.distance.distance(u, v).km

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
		"street":"st",
		"punggol stn/waterway point":"punggol stn",
		"opposite":"opp",
		"primary":"pr",
		"school":"sch"
		}

	startlat = start[0]
	startLon = start[1]
	R=6378137
	dn = 50
	de = 50
	dLat = dn/R
	dLon = de/(R*math.cos(math.pi*startlat/180))
	maxstartLat = startlat + dLat * 180/math.pi
	maxstartLon = startLon + dLon * 180/math.pi
	dn = -50
	de = -50
	dLat = dn/R
	dLon = de/(R*math.cos(math.pi*startlat/180))
	minstartLat = startlat + dLat * 180/math.pi
	minstartLon = startLon + dLon * 180/math.pi

	buss = pois_from_polygon(box(minstartLon, minstartLat, maxstartLon, maxstartLat), tags=tags)
	print(buss.columns)
	busStopStart = None
	busStopStartName = None

	for name in buss["name"]:
		name = name.lower()
		for word, initial in words_rep.items():
			name = name.replace(word, initial)
		busStopStartName = name.title()

	endlat = end[0]
	endLon = end[1]
	R=6378137
	dn = 50
	de = 50
	dLat = dn/R
	dLon = de/(R*math.cos(math.pi*endlat/180))
	maxendLat = endlat + dLat * 180/math.pi
	maxendLon = endLon + dLon * 180/math.pi
	dn = -50
	de = -50
	dLat = dn/R
	dLon = de/(R*math.cos(math.pi*endlat/180))
	minendLat = endlat + dLat * 180/math.pi
	minendLon = endLon + dLon * 180/math.pi

	buss = pois_from_polygon(box(minendLon, minendLat, maxendLon, maxendLat), tags=tags)
	# print(bus.columns)
	busStopEnd = None
	busStopEndName = None
	for name in buss["name"]:
		name = name.lower()
		for word, initial in words_rep.items():
			name = name.replace(word, initial)
		busStopEndName = name.title()

	startStation = dict(filter(lambda item: busStopStartName in item[0], stop_desc_map.items()))
	endStation = dict(filter(lambda item: busStopEndName in item[0], stop_desc_map.items()))

	results = []
	for x in startStation:
		for y in endStation:
			results.append(bfs(busGraph, startStation[x]["BusStopCode"], endStation[y]["BusStopCode"]))

	cheapest = None
	cheapestCost = float("Infinity")
	for cost, distance, transfers, path in results:
		if cost < cheapestCost:
			cheapest = (cost, distance, transfers, path)
			cheapestCost = cost

	print(results)
	cost, distance, transfers, path = cheapest
	for code, service in path:
		print(service, stop_code_map[code]["Description"])
	print (len(path)-1, "stops")
	print ("cost", cost)
	print ("distance", distance, "km")
	print ("transfers", transfers)
	return path