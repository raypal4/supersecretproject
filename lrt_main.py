import folium
import osmapi as osm
import osmnx as ox

from lrt_function import *
from bus_functions import *

print("Loading OSM")
graph = ox.graph_from_file(
    "punggol.osm", bidirectional=True, simplify=True, retain_all=False)

start = ox.geocode("punggol, singapore")
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
path_to_Lrt = findNearestBusStopFromLRT(graph, start, end, start_node, end_node)
for items in path_to_Lrt[0]:
    for key, value in items:
        startLrtLat = key[2]
        startLrtLong = key[3]
        endLrtLat = value[2]
        endLrtLong = value[3]
# TO CREATE BUS ROUTING
start_Lrt_node = ox.get_nearest_node(graph, (startLrtLat, startLrtLong))
end_Lrt_node = ox.get_nearest_node(graph, (endLrtLat, endLrtLong))
pathcheckToLrtBus = bus(busGraph, graph, start, (startLrtLat, startLrtLong), start_node, start_Lrt_node)
pathcheckFromLrtBus = bus(busGraph, graph, (endLrtLat, endLrtLong), end, end_Lrt_node, end_node)
# 			# TO CREATE ROUTING WITH BUS
# 			nodepath = astar_path(graph, start_node, end_node)
# 			m = ox.plot_route_folium(
# 				graph, nodepath, route_color='green', route_opacity=0)
# 			folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
# 				color='red', icon='flag')).add_to(m)
# 			folium.Marker(location=(end[0], end[1]), popup='END', icon=folium.Icon(color='blue', icon='flag')).add_to(m)
# 			for loc, code in markers:
# 				folium.Marker(location=loc, popup='Lrt stop:' + str(code),
# 							icon=folium.Icon(color='green', icon='train', prefix='fa')).add_to(m)
# 			folium.PolyLine(line, color="red", weight=2.5, opacity=1).add_to(m)

# 			# start point to start busstop
# 			folium.PolyLine([start, latlontoLrt[0]], color="blue", weight=2.5, opacity=1, dasharray="4").add_to(m)

# 			folium.PolyLine(latlontoLrt, color="green", weight=2.5, opacity=1).add_to(m)

# 			folium.PolyLine([latlontoLrt[-1], line[0]], color="green", weight=2.5, opacity=1, dasharray="4").add_to(m)

# 			# End  bus stop to end point
# 			folium.PolyLine([line[-1], latlonfromLrt[0]], color="green", weight=2.5, opacity=1, dasharray="4").add_to(m)

# 			folium.PolyLine(latlonfromLrt, color="green", weight=2.5, opacity=1, dasharray="4").add_to(m)

# 			folium.PolyLine([latlonfromLrt[-1], end], color="blue", weight=2.5, opacity=1, dasharray="4").add_to(m)

# 			m.save('index.html')

# 		# IF BUS ROUTE NOT FOUND, RUN WALK ROUTE
# 		if pathcheck[1] == 1:
# 			nodepath = pathcheck[0]
# 			m = ox.plot_route_folium(
# 				graph, nodepath, route_color='green')
# 			folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
# 				color='red', icon='flag')).add_to(m)
# 			folium.Marker(location=(end[0], end[1]), popup='END', icon=folium.Icon(color='blue', icon='flag')).add_to(m)
# 			m.save('index.html')
