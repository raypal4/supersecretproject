import folium
import osmapi as osm
import osmnx as ox
from shapely import geometry, ops
import geopy.distance

from bus_functions import *

print("Loading OSM")
graph = ox.graph_from_file(
    "punggol.osm", bidirectional=True, simplify=True, retain_all=False)


start = ox.geocode("safra punggol, singapore")
end = ox.geocode("blk 288B, punggol, singapore")

print("Found a starting node", start)
print("Found a ending node", end)

# graph = ox.graph_from_point(start, distance=5000, network_type='all')

start_node = ox.get_nearest_node(graph, start)
end_node = ox.get_nearest_node(graph, end)

nodes, edges = ox.graph_to_gdfs(graph)

# TO CREATE BUS ROUTING
pathcheck = bus(busGraph, graph, start, end, start_node, end_node)

# IF BUS ROUTE IS AVAILABLE
if pathcheck[1] == 0:
    startStopCoords = pathcheck[2]
    endStopCoords = pathcheck[3]

    print(startStopCoords)
    print(endStopCoords)

    # start_Bus = ox.get_nearest_nodes(graph, [startStopCoords[1]], [startStopCoords[0]])
    # end_Bus = ox.get_nearest_nodes(graph, [endStopCoords[1]], [endStopCoords[0]])

    # for startwalk in start_Bus:
    # 	pathToBusstop = astar_path(graph, start_node, startwalk)
    # for endwalk in end_Bus:
    # 	pathFromBusstop = astar_path(graph, endwalk, end_node)

    start_Bus = ox.get_nearest_node(graph, startStopCoords)
    end_Bus = ox.get_nearest_node(graph, endStopCoords)

    # geom, u, v = ox.get_nearest_edge(graph, startStopCoords)
    # start_Bus = min((u, v), key=lambda n: ox.great_circle_vec(startStopCoords[0], startStopCoords[1], graph.nodes[n]['y'], graph.nodes[n]['x']))
    # geom, u, v = ox.get_nearest_edge(graph, endStopCoords)
    # end_Bus = min((u, v), key=lambda n: ox.great_circle_vec(endStopCoords[0], endStopCoords[1], graph.nodes[n]['y'], graph.nodes[n]['x']))

    pathToBusstop = astar_path(graph, start_node, start_Bus)
    pathFromBusstop = astar_path(graph, end_Bus, end_node)

    print(pathToBusstop)
    print(pathFromBusstop)

    latlontobus = []
    latlonfrombus = []

    # walk from start to bus start
    startbuscoord = (graph.nodes[start_Bus]['y'], graph.nodes[start_Bus]['x'])
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
                    latlontobus.append((point[1], point[0]))
                    ptr += 1
            except:
                print("you shall not pass to")
                pass
            finally:
                prev = item
    if splice is not None:
        latlontobus = latlontobus[:splice+1]

    # walk for bus end to dst
    endbuscoord = (graph.nodes[end_Bus]['y'], graph.nodes[end_Bus]['x'])
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
                    latlonfrombus.append((point[1], point[0]))
                    ptr += 1
            except:
                print("you shall not pass from")
                pass
            finally:
                prev = item
    if splice is not None:
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
            # if prevService is not None:
            #     # print(prevService, service)
            #     if service[1] == 1:
            #         routing = busRoute0[service[0]]["coordinates"]
            #     else:
            #         routing = busRoute1[service[0]]["coordinates"]
            #     print(path[i])
            #     prevStopCode, a = path[i-1]
            #     print("HERE", prevStopCode)
            #     # find prev bus stop on new service route
            #     while indexing < len(routing):
            #         clon, clat = routing[indexing]
            #         qlat = bus_stop_code_map[prevStopCode]["Latitude"]
            #         qlon = bus_stop_code_map[prevStopCode]["Longitude"]
            #         u = (qlat, qlon)
            #         v = (clat, clon)
            #         if geopy.distance.distance(u, v).km < 0.03:
            #             print(u, v)
            #             print("FOUND INDEX: ", prevIndex, indexing)
            #             for x, y in routing[prevIndex: indexing + 1]:
            #                 print("ENTER APPENDED?")
            #                 line.append((y, x))
            #             break
            #         prevIndex = indexing
            #         indexing += 1
            #     markers.append((v, stopCode))
            #     prevService = service
            #     break

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
                        # print("ALWAYS COME HERE?", prevIndex, indexing)
                        for x, y in routing[prevIndex: indexing + 1]:
                            line.append((y, x))
                    else:
                        # print("ENTER ELSE ---------------------------------")
                        prevLatLong = line[-1]
                        tempIndex = 0
                        while tempIndex < len(routing):
                            plon, plat = routing[tempIndex]
                            p = (plat, plon)
                            # print("ELSE COME HERE?", tempIndex, indexing)
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
        graph, nodepath, route_color='green', route_opacity=0)
    folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
        color='red', icon='flag')).add_to(m)
    folium.Marker(location=(end[0], end[1]), popup='END', icon=folium.Icon(
        color='blue', icon='flag')).add_to(m)
    for loc, code in markers:
        folium.Marker(location=loc, popup='Bus stop number:' + str(code),
                      icon=folium.Icon(color='green', icon='bus', prefix='fa')).add_to(m)
    folium.PolyLine(line, color="red", weight=2.5, opacity=1).add_to(m)

    print(latlontobus)
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

    folium.PolyLine(latlonfrombus, color="green", weight=2.5,
                    opacity=1, dasharray="4").add_to(m)

    folium.PolyLine([latlonfrombus[-1], end], color="blue",
                    weight=2.5, opacity=1, dasharray="4").add_to(m)

    m.save('Bus_routing.html')

# IF BUS ROUTE NOT FOUND, RUN WALK ROUTE
if pathcheck[1] == 1:
    nodepath = pathcheck[0]
    m = ox.plot_route_folium(
        graph, nodepath, route_color='green')
    folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
        color='red', icon='flag')).add_to(m)
    folium.Marker(location=(end[0], end[1]), popup='END', icon=folium.Icon(
        color='blue', icon='flag')).add_to(m)
    m.save('Bus_routing.html')

print("Bus_routing.html created!")
