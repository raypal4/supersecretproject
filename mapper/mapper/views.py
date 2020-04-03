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

from shapely import ops
from mapper.bus_functions import *
from mapper.lrt_function import *

import math

ox.config(log_console=True, use_cache=True)

AlgoItterations1 = 0
AlgoItterations2 = 0
AlgoItterations3 = 0
totalWeight = 0


def index(request):
    try:
        if request.method == 'POST':
            retrieveForm = AddressForm(request.POST)
            if retrieveForm.is_valid():
                # punggol, singapore
                org_var = retrieveForm.cleaned_data['var_org']
                # blk 128B, punggol, singapore
                dst_var = retrieveForm.cleaned_data['var_dst']
                type_var = retrieveForm.cleaned_data['var_type']

                geolocator = Nominatim(user_agent="test")
                org_addr = geolocator.geocode(org_var)
                dst_addr = geolocator.geocode(dst_var)
                org = (org_addr.latitude, org_addr.longitude)
                dest = (dst_addr.latitude, dst_addr.longitude)

                if type_var == 'Bus':
                    graph = ox.graph_from_file(
                        JSON_FOLDER + "punggol.osm", bidirectional=True, simplify=True, retain_all=False)

                    start = ox.geocode(org_var)
                    end = ox.geocode(dst_var)

                    print("Found a starting node", start)
                    print("Found a ending node", end)

                    start_node = ox.get_nearest_node(graph, start)
                    end_node = ox.get_nearest_node(graph, end)

                    nodes, edges = ox.graph_to_gdfs(graph)

                    # TO CREATE BUS ROUTING
                    pathcheck = bus(busGraph, graph, start,
                                    end, start_node, end_node)

                    # IF BUS ROUTE IS AVAILABLE
                    if pathcheck[1] == 0:
                        startStopCoords = pathcheck[2]
                        endStopCoords = pathcheck[3]

                        print(startStopCoords)
                        print(endStopCoords)

                        start_Bus = ox.get_nearest_node(graph, startStopCoords)
                        end_Bus = ox.get_nearest_node(graph, endStopCoords)

                        pathToBusstop = astar_path(
                            graph, start_node, start_Bus)
                        pathFromBusstop = astar_path(graph, end_Bus, end_node)

                        latlontobus = []
                        latlonfrombus = []

                        # walk from start to bus start
                        startbuscoord = (
                            graph.nodes[start_Bus]['y'], graph.nodes[start_Bus]['x'])
                        prev = None
                        splice = None
                        ptr = 0
                        temp = float("Infinity")
                        for item in pathToBusstop:
                            if prev is None:
                                prev = item
                            else:
                                try:
                                    line = graph[prev][item][0]["geometry"]
                                    for point in list(line.coords):
                                        if splice is None:
                                            splice = ptr
                                            temp = geopy.distance.distance(
                                                startbuscoord, (point[1], point[0])).km
                                        elif geopy.distance.distance(startbuscoord, (point[1], point[0])).km < temp:
                                            splice = ptr
                                            temp = geopy.distance.distance(
                                                startbuscoord, (point[1], point[0])).km
                                        latlontobus.append(
                                            (point[1], point[0]))
                                        ptr += 1
                                except:
                                    pass
                                finally:
                                    prev = item
                        latlontobus = latlontobus[:splice+1]

                        # walk for bus end to dst
                        endbuscoord = (
                            graph.nodes[end_Bus]['y'], graph.nodes[end_Bus]['x'])
                        prev = None
                        splice = None
                        ptr = 0
                        temp = float("Infinity")
                        for item in pathFromBusstop:
                            if prev is None:
                                prev = item
                            else:
                                try:
                                    line = graph[prev][item][0]["geometry"]
                                    for point in list(line.coords):
                                        if splice is None:
                                            splice = ptr
                                            temp = geopy.distance.distance(
                                                endbuscoord, (point[1], point[0])).km
                                        elif geopy.distance.distance(endbuscoord, (point[1], point[0])).km < temp:
                                            splice = ptr
                                            temp = geopy.distance.distance(
                                                endbuscoord, (point[1], point[0])).km
                                        latlonfrombus.append(
                                            (point[1], point[0]))
                                        ptr += 1
                                except:
                                    pass
                                finally:
                                    prev = item
                        latlonfrombus = latlonfrombus[:splice+1]

                        path = pathcheck[0]
                        indexing = 0
                        line = []
                        prevService = None
                        prevIndex = None
                        i = 0
                        markers = []
                        routing = None
                        while i < len(path):
                            stopCode, service = path[i]
                            # in the case of first stop, no bus service stated, take next
                            if service is None:
                                service = path[i + 1][1]

                            if service != prevService:
                                indexing = 0
                                prevIndex = 0

                            qlat = bus_stop_code_map[stopCode]["Latitude"]
                            qlon = bus_stop_code_map[stopCode]["Longitude"]

                            # get routes for respective direction
                            if service[1] == 1:
                                routing = busRoute0[service[0]]["coordinates"]
                            else:
                                routing = busRoute1[service[0]]["coordinates"]
                            while indexing < len(routing):
                                clon, clat = routing[indexing]
                                u = (qlat, qlon)
                                v = (clat, clon)
                                # stop found in range of 30 meters, latlong accuracy difference from
                                # two sources
                                if geopy.distance.distance(u, v).km < 0.03:
                                    # first bus stop
                                    if prevService is None:
                                        line.append(v)
                                    else:
                                        if prevService == service:
                                            print("ALWAYS COME HERE?",
                                                  prevIndex, indexing)
                                            for x, y in routing[prevIndex: indexing + 1]:
                                                line.append((y, x))
                                        else:
                                            print(
                                                "ENTER ELSE ---------------------------------")
                                            prevLatLong = line[-1]
                                            tempIndex = 0
                                            while tempIndex < len(routing):
                                                plon, plat = routing[tempIndex]
                                                p = (plat, plon)
                                                if geopy.distance.distance(prevLatLong, p).km < 0.03:
                                                    for x, y in routing[tempIndex: indexing + 1]:
                                                        line.append((y, x))
                                                    break
                                                tempIndex += 1
                                    prevIndex = indexing
                                    prevService = service
                                    markers.append((v, stopCode))
                                    break
                                indexing += 1
                            i += 1

                        # TO CREATE ROUTING WITH BUS
                        nodepath = astar_path(graph, start_node, end_node)
                        m = ox.plot_route_folium(
                            graph, nodepath, tiles='openstreetmap', route_color='green', route_opacity=0)
                        folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
                            color='red', icon='flag')).add_to(m)
                        folium.Marker(location=(end[0], end[1]), popup='END', icon=folium.Icon(
                            color='blue', icon='flag')).add_to(m)
                        for loc, code in markers:
                            folium.Marker(location=loc, popup='Bus stop number:' + str(
                                code), icon=folium.Icon(color='green', icon='bus', prefix='fa')).add_to(m)
                        folium.PolyLine(line, color="red",
                                        weight=2.5, opacity=1).add_to(m)

                        # start point to start busstop
                        folium.PolyLine([start, latlontobus[0]], color="blue",
                                        weight=2.5, opacity=1, dasharray="4").add_to(m)
                        folium.PolyLine(latlontobus, color="green",
                                        weight=2.5, opacity=1).add_to(m)
                        folium.PolyLine([latlontobus[-1], line[0]], color="blue",
                                        weight=2.5, opacity=1, dasharray="4").add_to(m)

                        # End  bus stop to end point
                        folium.PolyLine([line[-1], latlonfrombus[0]], color="blue",
                                        weight=2.5, opacity=1, dasharray="4").add_to(m)
                        folium.PolyLine(
                            latlonfrombus, color="green", weight=2.5, opacity=1, dasharray="4").add_to(m)
                        folium.PolyLine([latlonfrombus[-1], end], color="blue",
                                        weight=2.5, opacity=1, dasharray="4").add_to(m)

                        m.save(ROUTE_FOLDER + 'bus_routing.html')

                    # IF BUS ROUTE NOT FOUND, RUN WALK ROUTE
                    if pathcheck[1] == 1:
                        nodepath = pathcheck[0]
                        m = ox.plot_route_folium(
                            graph, nodepath, tiles='openstreetmap', route_color='green')
                        folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
                            color='red', icon='flag')).add_to(m)
                        folium.Marker(location=(end[0], end[1]), popup='END', icon=folium.Icon(
                            color='blue', icon='flag')).add_to(m)
                        m.save(ROUTE_FOLDER + 'bus_routing.html')

                    print("bus_routing.html created!")

                    speedBus = 40
                    stopsBus = getStops()
                    distanceBus = round(getDistance(), 3)
                    timeBus = math.ceil((getDistance() / speedBus) * 60)
                    timeBus = timeBus + stopsBus
                    if distanceBus <= 3.2:
                        costBus = 0.92
                    elif distanceBus <= 4.2:
                        costBus = 1.02
                    elif distanceBus <= 5.2:
                        costBus = 1.12
                    elif distanceBus <= 6.2:
                        costBus = 1.22
                    elif distanceBus <= 7.2:
                        costBus = 1.31
                    elif distanceBus <= 8.2:
                        costBus = 1.38
                    elif distanceBus <= 9.2:
                        costBus = 1.44
                    else:
                        costBus = 2
                    busRoute = getBus().split(',')

                    contentDict = {"org_addr": org_addr.address, "dst_addr": dst_addr.address, "mode_var": type_var, "distance": distanceBus,
                                   "stops": getStops(), "bus": busRoute, "time": timeBus, "cost": costBus}
                    return render(request, "bus.html", contentDict)

                elif type_var == 'LRT':
                    print("Loading OSM")
                    graph = ox.graph_from_file(
                        JSON_FOLDER + "punggol.osm", bidirectional=True, simplify=True, retain_all=False)

                    start = ox.geocode(org_var)
                    end = ox.geocode(dst_var)
                    print("Found a starting node", start)
                    print("Found a ending node", end)

                    start_node = ox.get_nearest_node(graph, start)
                    end_node = ox.get_nearest_node(graph, end)

                    nodes, edges = ox.graph_to_gdfs(graph)

                    # TO CREATE ROUTE TO AND FROM LRT
                    path_to_Lrt = findNearestLrt(
                        graph, start, end, start_node, end_node)

                    # LRT ROUTING
                    lrtline = []
                    lrtMarkers = []

                    if path_to_Lrt is not None:
                        # if LRT path is found
                        if path_to_Lrt[1] == 0:
                            pathDict = path_to_Lrt[0]
                            for path in pathDict:
                                indexing = 0
                                prevService = None
                                prevIndex = None
                                i = 0
                                for item in path:
                                    direction = item[0][4]
                                    stationName = item[0][1]
                                    loop = item[0][-1]
                                if direction == 1:
                                    print("D 1 \n")
                                    routing = LrtRoute0[loop]["coordinates"]
                                else:
                                    print("D 2 \n")
                                    routing = LrtRoute1[loop]["coordinates"]

                                while i < len(path[item]):
                                    qlat = path[item][i][2]
                                    qlon = path[item][i][3]
                                    while indexing < len(routing):
                                        clon, clat = routing[indexing]
                                        u = (qlat, qlon)
                                        v = (clat, clon)
                                        if geopy.distance.distance(u, v).km < 0.03:
                                            if prevService is None:
                                                lrtline.append(v)
                                            else:
                                                prevLatLong = lrtline[-1]
                                                tempIndex = 0
                                                while tempIndex < len(routing):
                                                    plon, plat = routing[tempIndex]
                                                    p = (plat, plon)
                                                    if geopy.distance.distance(prevLatLong, p).km < 0.01:
                                                        for x, y in routing[tempIndex: indexing+1]:
                                                            lrtline.append(
                                                                (y, x))
                                                        break
                                                    tempIndex += 1
                                            prevIndex = indexing
                                            prevService = path[item]
                                            lrtMarkers.append(
                                                (v, path[item][i][1], path[item][i][-1], path[item][i][-2]))
                                            break
                                        indexing += 1
                                    i += 1

                            nearestStartStop = path_to_Lrt[2]
                            nearestEndStop = path_to_Lrt[3]

                            start_Lrt = ox.get_nearest_node(
                                graph, nearestStartStop)
                            end_Lrt = ox.get_nearest_node(
                                graph, nearestEndStop)

                            pathToLrtStop = astar_path(
                                graph, start_node, start_Lrt)
                            pathFromLrtStop = astar_path(
                                graph, end_Lrt, end_node)

                            latlontolrt = []
                            latlonfromlrt = []

                            # walk from start to lrt start
                            startlrtcoord = (
                                graph.nodes[start_Lrt]['y'], graph.nodes[start_Lrt]['x'])
                            prev = None
                            splice = None
                            ptr = 0
                            temp = float("Infinity")
                            for item in pathToLrtStop:
                                if prev is None:
                                    prev = item
                                else:
                                    try:
                                        line = graph[prev][item][0]["geometry"]
                                        for point in list(line.coords):
                                            if splice is None:
                                                splice = ptr
                                                temp = geopy.distance.distance(
                                                    startlrtcoord, (point[1], point[0])).km
                                            elif geopy.distance.distance(startlrtcoord, (point[1], point[0])).km < temp:
                                                splice = ptr
                                                temp = geopy.distance.distance(
                                                    startlrtcoord, (point[1], point[0])).km
                                            latlontolrt.append(
                                                (point[1], point[0]))
                                            ptr += 1
                                    except:
                                        pass
                                    finally:
                                        prev = item
                            if splice is not None:
                                latlontolrt = latlontolrt[:splice+1]

                            # walk from lrt end to end
                            endlrtcoord = (
                                graph.nodes[end_Lrt]['y'], graph.nodes[end_Lrt]['x'])
                            prev = None
                            splice = None
                            ptr = 0
                            temp = float("Infinity")
                            for item in pathFromLrtStop:
                                if prev is None:
                                    prev = item
                                else:
                                    try:
                                        line = graph[prev][item][0]["geometry"]
                                        for point in list(line.coords):
                                            if splice is None:
                                                splice = ptr
                                                temp = geopy.distance.distance(
                                                    endlrtcoord, (point[1], point[0])).km
                                            elif geopy.distance.distance(endlrtcoord, (point[1], point[0])).km < temp:
                                                splice = ptr
                                                temp = geopy.distance.distance(
                                                    endlrtcoord, (point[1], point[0])).km
                                            latlonfromlrt.append(
                                                (point[1], point[0]))
                                            ptr += 1
                                    except:
                                        pass
                                    finally:
                                        prev = item
                            if splice is not None:
                                latlonfromlrt = latlonfromlrt[:splice+1]
                        else:
                            print("LRT route unable to be established")

                    # default route
                    nodepath = astar_path(graph, start_node, end_node)

                    if path_to_Lrt[1] == 0:
                        # INIT
                        m = ox.plot_route_folium(
                            graph, nodepath, tiles='openstreetmap', route_color='green', route_opacity=0)

                        # LRT LINE
                        folium.PolyLine(lrtline, color="black",
                                        weight=2.5, opacity=1).add_to(m)

                        # Variables for Display
                        strStation = ''
                        stationCount = 0

                        # LRT Markers
                        for loc, station, loop, direction in lrtMarkers:
                            folium.Marker(location=loc, popup='<b>Station Name:</b> ' + str(station) + '<br><b>Lat:</b> ' + str(loc[0]) + '<br><b>Lon:</b> ' + str(loc[1]) + '<br><b>Loop:</b> ' + str(loop) + '<br><b>Direction: </b>' + str(direction),
                                          icon=folium.Icon(color='black', icon='train', prefix='fa')).add_to(m)
                            strStation = strStation + str(station) + ','
                            stationCount = stationCount + 1
                        strStation = strStation + 'End'

                        # START AND END MARKERS
                        folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
                            color='red', icon='flag')).add_to(m)
                        folium.Marker(location=(end[0], end[1]), popup='END', icon=folium.Icon(
                            color='blue', icon='flag')).add_to(m)

                        if len(latlontolrt) > 0:
                            # start point to start lRT
                            folium.PolyLine(
                                [start, latlontolrt[0]], color="blue", weight=2.5, opacity=1, dasharray="4").add_to(m)
                            folium.PolyLine(
                                latlontolrt, color="green", weight=2.5, opacity=1).add_to(m)
                            folium.PolyLine([latlontolrt[-1], lrtline[0]], color="blue",
                                            weight=2.5, opacity=1, dasharray="4").add_to(m)
                        if len(latlonfromlrt) > 0:
                            # End LRT stop to end point
                            folium.PolyLine([lrtline[-1], latlonfromlrt[0]], color="blue",
                                            weight=2.5, opacity=1, dasharray="4").add_to(m)
                            folium.PolyLine(
                                latlonfromlrt, color="green", weight=2.5, opacity=1, dasharray="4").add_to(m)
                            folium.PolyLine(
                                [latlonfromlrt[-1], end], color="blue", weight=2.5, opacity=1, dasharray="4").add_to(m)

                        m.save(ROUTE_FOLDER + 'lrt_routing.html')

                        print("LRT_Routing.html created!")

                    # IF LRT ROUTE NOT FOUND, RUN WALK ROUTE
                    if path_to_Lrt[1] == 1:
                        # INIT
                        m = ox.plot_route_folium(
                            graph, nodepath, tiles='openstreetmap', route_color='green')
                        folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
                            color='red', icon='flag')).add_to(m)
                        folium.Marker(location=(end[0], end[1]), popup='END', icon=folium.Icon(
                            color='blue', icon='flag')).add_to(m)
                        m.save(ROUTE_FOLDER + 'lrt_routing.html')
                        print("LRT_Routing.html created!")

                    strStation = strStation.split(',')
                    timeTrain = 2 * stationCount
                    if getDistanceLRT() <= 3.2:
                        costLRT = 0.92
                    elif getDistanceLRT() <= 4.2:
                        costLRT = 1.02
                    elif getDistanceLRT() <= 5.2:
                        costLRT = 1.12
                    elif getDistanceLRT() <= 6.2:
                        costLRT = 1.22
                    elif getDistanceLRT() <= 7.2:
                        costLRT = 1.31
                    elif getDistanceLRT() <= 8.2:
                        costLRT = 1.38
                    elif getDistanceLRT() <= 9.2:
                        costLRT = 1.44
                    else:
                        costLRT = 2

                    contentDict = {"org_addr": org_addr.address, "dst_addr": dst_addr.address, "mode_var": type_var,
                                   "count": stationCount, "route": strStation, "time": timeTrain, "cost": costLRT}
                    return render(request, "lrt.html", contentDict)

                else:
                    if type_var == 'Walk':
                        if org:
                            graph = ox.graph_from_point(
                                org, distance=2000, network_type='walk')
                        else:
                            (graph, org) = ox.graph_from_address(
                                address, distance=5000, network_type='walk', return_coords=True)
                    elif type_var == 'Drive':
                        if org:
                            graph = ox.graph_from_point(
                                org, distance=2000, network_type='drive')
                        else:
                            graph = ox.graph_from_address(
                                address, distance=5000, network_type='drive')

                    # get cloest node to the point of search
                    global target_node
                    orig_node = ox.get_nearest_node(graph, org)
                    target_node = ox.get_nearest_node(graph, dest)

                    nodes, edges = ox.graph_to_gdfs(graph)

                    node_data = get_nodes(edges)
                    ourRoute3 = list(
                        creator3(nodes, node_data, orig_node, target_node))

                    # usage of folium to create interactive web map
                    astar_map = ox.plot_route_folium(
                        graph, ourRoute3, popup_attribute='name', tiles='openstreetmap', route_color='blue')
                    folium.Marker(location=org, popup=org_addr.address, icon=folium.Icon(
                        color='red')).add_to(astar_map)
                    folium.Marker(location=dest, popup=dst_addr.address, icon=folium.Icon(
                        color='blue')).add_to(astar_map)
                    filepath3 = ROUTE_FOLDER + 'astar_route.html'
                    astar_map.save(filepath3)

                    distanceAstar = round(getDistanceTravelled(
                        nodes, node_data, ourRoute3), 3)
                    if type_var == 'Walk':
                        estSpeed = 5
                    else:
                        estSpeed = 50
                    timeAstar = math.ceil((distanceAstar / estSpeed) * 60)

                    contentDict = {"org_addr": org_addr.address, "dst_addr": dst_addr.address,
                                   "dist_astar": distanceAstar, "mode_var": type_var, "time": timeAstar, "estSpeed": estSpeed}
                    return render(request, "main.html", contentDict)

        else:
            inputForm = AddressForm()
            return render(request, "index.html", {'form': inputForm})
    except:
        inputForm = AddressForm()
        return render(request, "error.html", {'form': inputForm})


class Graph():
    def __init__(self):
        self.edges = defaultdict(list)
        self.weights = {}

    def add_edge(self, u, v, weight):
        self.edges[u].append(v)
        self.weights[(u, v)] = weight


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
                current_coord = (y, x)
                prev = mins.get(v2, None)
                next = cost + c*100 + \
                    geopy.distance.distance(target_coord, current_coord).km
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
