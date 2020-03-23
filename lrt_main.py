import folium
import osmapi as osm
import osmnx as ox

from lrt_function import *

print("Loading OSM")
graph = ox.graph_from_file("punggol.osm", bidirectional=True, simplify=True, retain_all=False)

start = ox.geocode("punggol, singapore")
end = ox.geocode("horizon primary school, singapore")
print("Found a starting node", start)
print("Found a ending node", end)

start_node = ox.get_nearest_node(graph, start)
end_node = ox.get_nearest_node(graph, end)

nodes, edges = ox.graph_to_gdfs(graph)

# TO CREATE BUS ROUTING
pathcheck = lrt(lrtGraph, graph, start, end, start_node, end_node)

if pathcheck is not None:
		# IF BUS ROUTE IS AVAILABLE
		if pathcheck[1] == 0:
			startStopCoords = pathcheck[2]
			endStopCoords = pathcheck[3]

			start_Lrt = ox.get_nearest_node(graph, startStopCoords)
			end_Lrt = ox.get_nearest_node(graph, endStopCoords)

			pathToLrt = astar_path(graph, start_node, start_Lrt)
			pathFromLrt = astar_path(graph, end_Lrt, end_node)

			latlontoLrt = []
			latlonfromLrt = []

			api = osm.OsmApi()  # this instantiate the OsmApi class,

			for item in pathToLrt:
				node = api.NodeGet(item)
				latlontoLrt.append((node["lat"], node["lon"]))

			for item in pathFromLrt:
				node = api.NodeGet(item)
				latlonfromLrt.append((node["lat"], node["lon"]))

			path = pathcheck[0]
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
					service = path[i + 1][1]

				if service != prevService:
					indexing = 0

				qlat = stop_desc_map[stopCode]["Latitude"]
				qlon = stop_desc_map[stopCode]["Longitude"]

				# get routes for respective direction
				if service[1] == 1:
					routing = LrtRoute0[service[0]]["coordinates"]
				else:
					routing = LrtRoute1[service[0]]["coordinates"]
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
								for x, y in routing[prevIndex: indexing + 1]:
									line.append((y, x))
							else:
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
				graph, nodepath, route_color='green', route_opacity=0)
			folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
				color='red', icon='flag')).add_to(m)
			folium.Marker(location=(end[0], end[1]), popup='END', icon=folium.Icon(color='blue', icon='flag')).add_to(m)
			for loc, code in markers:
				folium.Marker(location=loc, popup='Lrt stop:' + str(code),
							icon=folium.Icon(color='green', icon='train', prefix='fa')).add_to(m)
			folium.PolyLine(line, color="red", weight=2.5, opacity=1).add_to(m)

			# start point to start busstop
			folium.PolyLine([start, latlontoLrt[0]], color="blue", weight=2.5, opacity=1, dasharray="4").add_to(m)

			folium.PolyLine(latlontoLrt, color="green", weight=2.5, opacity=1).add_to(m)

			folium.PolyLine([latlontoLrt[-1], line[0]], color="green", weight=2.5, opacity=1, dasharray="4").add_to(m)

			# End  bus stop to end point
			folium.PolyLine([line[-1], latlonfromLrt[0]], color="green", weight=2.5, opacity=1, dasharray="4").add_to(m)

			folium.PolyLine(latlonfromLrt, color="green", weight=2.5, opacity=1, dasharray="4").add_to(m)

			folium.PolyLine([latlonfromLrt[-1], end], color="blue", weight=2.5, opacity=1, dasharray="4").add_to(m)

			m.save('index.html')

		# IF BUS ROUTE NOT FOUND, RUN WALK ROUTE
		if pathcheck[1] == 1:
			nodepath = pathcheck[0]
			m = ox.plot_route_folium(
				graph, nodepath, route_color='green')
			folium.Marker(location=(start[0], start[1]), popup='START', icon=folium.Icon(
				color='red', icon='flag')).add_to(m)
			folium.Marker(location=(end[0], end[1]), popup='END', icon=folium.Icon(color='blue', icon='flag')).add_to(m)
			m.save('index.html')
