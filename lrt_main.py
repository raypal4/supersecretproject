import folium
import osmapi as osm
import osmnx as ox

from lrt_function import *
from bus_functions import *

print("Loading OSM")
graph = ox.graph_from_file(
    "punggol.osm", bidirectional=True, simplify=True, retain_all=False)

start = ox.geocode("nibong, punggol, singapore")
end = ox.geocode("horizon primary school, singapore")
print("Found a starting node", start)
print("Found a ending node", end)

# start = "Oasis Station"
# end = "Sam Kee Station"

start_node = ox.get_nearest_node(graph, start)
end_node = ox.get_nearest_node(graph, end)

nodes, edges = ox.graph_to_gdfs(graph)

startLrtLat = 0
startLrtLong = 0
endLrtLat = 0
endLrtLong = 0

# TO CREATE ROUTE TO AND FROM LRT
path_to_Lrt = findNearestBusStopFromLRT(
    graph, start, end, start_node, end_node)
for items in path_to_Lrt[0]:
    for key, value in items:
        startLrtLat = key[2]
        startLrtLong = key[3]
        endLrtLat = value[2]
        endLrtLong = value[3]

# LRT ROUTING
lrtline = []
lrtMarkers = []

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
                            if prevService == service:
                                for x, y in routing[prevIndex: indexing+1]:
                                    lrtline.append((y, x))
                            else:
                                prevLatLong = lrtline[-1]
                                tempIndex = 0
                                while tempIndex < len(routing):
                                    plon, plat = routing[tempIndex]
                                    p = (plat, plon)
                                    if geopy.distance.distance(prevLatLong, p).km < 0.03:
                                        for x, y in routing[tempIndex: indexing]:
                                            lrtline.append((y, x))
                                        break
                                    tempIndex += 1
                        prevIndex = indexing
                        prevService = service
                        lrtMarkers.append((v, path[item][i][1]))
                        break
                    indexing += 1
                i += 1

nodepath = astar_path(graph, start_node, end_node)
m = ox.plot_route_folium(
    graph, nodepath, route_color='green', route_opacity=0)

folium.PolyLine(lrtline, color="black", weight=2.5, opacity=1).add_to(m)
for loc, station in lrtMarkers:
    folium.Marker(location=loc, popup='Station Name:' + str(station),
                  icon=folium.Icon(color='black', icon='train', prefix='fa')).add_to(m)


m.save('index.html')

# # TO CREATE BUS ROUTING IF NON WALKABLE DISTANCE
# start_Lrt_node = ox.get_nearest_node(graph, (startLrtLat, startLrtLong))
# end_Lrt_node = ox.get_nearest_node(graph, (endLrtLat, endLrtLong))
# pathcheckToLrtBus = bus(busGraph, graph, start,
#                         (startLrtLat, startLrtLong), start_node, start_Lrt_node)
# pathcheckFromLrtBus = bus(
#     busGraph, graph, (endLrtLat, endLrtLong), end, end_Lrt_node, end_node)
