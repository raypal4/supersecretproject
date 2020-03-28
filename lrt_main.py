import folium
import osmapi as osm
import osmnx as ox

from lrt_function import *

print("Loading OSM")
graph = ox.graph_from_file(
    "punggol.osm", bidirectional=True, simplify=True, retain_all=False)

start = ox.geocode("Waterway Sundew, punggol, singapore")
end = ox.geocode("punggol cove primary school, punggol, singapore")
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

# lrtFull = []

if path_to_Lrt is not None:
    # if LRT path is found
    if path_to_Lrt[1] == 0:
        pathDict = path_to_Lrt[0]
        # print(path)
        for path in pathDict:
            indexing = 0
            prevService = None
            prevIndex = None
            i = 0
            for item in path:
                # print(path[item])
                # lrtFull = lrtFull + path[item]
                direction = item[0][4]
                stationName = item[0][1]
                loop = item[0][-1]
            # print(direction, stationName, loop)
            if direction == 1:
                print("D 1 \n")
                routing = LrtRoute0[loop]["coordinates"]
            else:
                print("D 2 \n")
                routing = LrtRoute1[loop]["coordinates"]

            # print(routing)
            while i < len(path[item]):
                qlat = path[item][i][2]
                qlon = path[item][i][3]
                # print(qlat, qlon)
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
                                        lrtline.append((y, x))
                                    break
                                tempIndex += 1
                        prevIndex = indexing
                        prevService = path[item]
                        lrtMarkers.append((v, path[item][i][1]))
                        break
                    indexing += 1
                i += 1

        nearestStartStop = path_to_Lrt[2]
        nearestEndStop = path_to_Lrt[3]

        start_Lrt = ox.get_nearest_node(graph, nearestStartStop)
        end_Lrt = ox.get_nearest_node(graph, nearestEndStop)

        pathToLrtStop = astar_path(graph, start_node, start_Lrt)
        pathFromLrtStop = astar_path(graph, end_Lrt, end_node)

        latlontolrt = []
        latlonfromlrt = []

        # walk from start to lrt start
        startlrtcoord = (graph.nodes[start_Lrt]['y'],
                         graph.nodes[start_Lrt]['x'])
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
                        latlontolrt.append((point[1], point[0]))
                        ptr += 1
                except:
                    pass
                finally:
                    prev = item
        if latlonfromlrt[:splice+1] is not None:
            latlontolrt = latlontolrt[:splice+1]

        # walk from lrt end to end
        endlrtcoord = (graph.nodes[end_Lrt]['y'], graph.nodes[end_Lrt]['x'])
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
                        latlonfromlrt.append((point[1], point[0]))
                        ptr += 1
                except:
                    pass
                finally:
                    prev = item
        if latlonfromlrt[:splice+1] is not None:
            latlonfromlrt = latlonfromlrt[:splice+1]
    else:
        print("LRT route unable to be established")


# default route
nodepath = astar_path(graph, start_node, end_node)

if path_to_Lrt[1] == 0:

    # INIT
    m = ox.plot_route_folium(
        graph, nodepath, route_color='green', route_opacity=0)

    # LRT LINE
    folium.PolyLine(lrtline, color="black", weight=2.5, opacity=1).add_to(m)

    # LRT Markers
    for loc, station in lrtMarkers:
        folium.Marker(location=loc, popup='Station Name:' + str(station),
                      icon=folium.Icon(color='black', icon='train', prefix='fa')).add_to(m)

    # START AND END MARKERS
    folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
        color='red', icon='flag')).add_to(m)
    folium.Marker(location=(end[0], end[1]), popup='END', icon=folium.Icon(
        color='blue', icon='flag')).add_to(m)

    # start point to start lRT
    folium.PolyLine([start, latlontolrt[0]], color="blue",
                    weight=2.5, opacity=1, dasharray="4").add_to(m)

    folium.PolyLine(latlontolrt, color="green",
                    weight=2.5, opacity=1).add_to(m)

    folium.PolyLine([latlontolrt[-1], lrtline[0]], color="blue",
                    weight=2.5, opacity=1, dasharray="4").add_to(m)

    # End LRT stop to end point
    folium.PolyLine([lrtline[-1], latlonfromlrt[0]], color="blue",
                    weight=2.5, opacity=1, dasharray="4").add_to(m)

    folium.PolyLine(latlonfromlrt, color="green", weight=2.5,
                    opacity=1, dasharray="4").add_to(m)

    folium.PolyLine([latlonfromlrt[-1], end], color="blue",
                    weight=2.5, opacity=1, dasharray="4").add_to(m)

    m.save('LRT_Routing.html')

    print("LRT_Routing.html created!")


# IF LRT PATH NOT ESTABLISHED. DO WALK
if path_to_Lrt[1] == 1:
    # INIT
    m = ox.plot_route_folium(
        graph, nodepath, route_color='green')
    folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
        color='red', icon='flag')).add_to(m)
    folium.Marker(location=(end[0], end[1]), popup='END', icon=folium.Icon(
        color='blue', icon='flag')).add_to(m)
    m.save('LRT_routing.html')
    print("LRT_Routing.html created!")
