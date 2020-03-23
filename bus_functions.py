import json
import math

from astar import *
from manualPatch.pois import *

print("Loading JSON")
stops = json.loads(open("punggolBusData/stops.json").read())
services = json.loads(open("punggolBusData/services.json").read())
routes = json.loads(open("punggolBusData/routes.json").read())
busRoute0 = json.loads(open("punggolBusData/busroute0.json").read())
busRoute1 = json.loads(open("punggolBusData/busroute1.json").read())

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
    if route["StopSequence"] == 4 and route["Distance"] == 9.1 and key == ("34", 1):
        route["StopSequence"] = 14
    routes_map[key] += [route]

print("Initializing Graph")
busGraph = {}
for service, path in routes_map.items():
    # hack around broken data
    path.sort(key=lambda r: r["StopSequence"])
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
            return curr_cost, curr_distance, curr_transfers - 1, path

        if (node, curr_service) in seen:
            continue

        seen.add((node, curr_service))
        # enumerate all adjacent nodes, construct a new path and push it into
        # the queue
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

            heapq.heappush(
                queue, (new_cost, new_distance, new_transfers, new_path))


def bus(busGraph, graph, start, end, start_node, end_node):
    tags = {
        'highway': 'bus_stop',
    }

    words_rep = {
        "avenue": "ave",
        "block": "blk",
        "boulevard": "blvd",
        "central": "ctrl",
        "close": "cl",
        "crescent": "cres",
        "drive": "dr",
        "expressway": "e'way",
        "highway": "hway",
        "industrial park": "ind park",
        "mount": "mt",
        "place": "pl",
        "ring road": "ring rd",
        "road": "rd",
        "service road": "service rd",
        "square": "sq",
        "station": "stn",
        "street": "st",
        "punggol stn/waterway point": "punggol stn",
        "opposite": "opp",
        "primary": "pr",
        "school": "sch",
        "before": "bef",
        "after": "aft"
    }
    # flag to check if there is a possible bus route (0 if have, 1 if loc is walkable)
    busflag = 0

    startlat = start[0]
    startLon = start[1]
    R = 6378137
    dn = 250
    de = 250
    dLat = dn / R
    dLon = de / (R * math.cos(math.pi * startlat / 180))
    maxstartLat = startlat + dLat * 180 / math.pi
    maxstartLon = startLon + dLon * 180 / math.pi
    dn = -250
    de = -250
    dLat = dn / R
    dLon = de / (R * math.cos(math.pi * startlat / 180))
    minstartLat = startlat + dLat * 180 / math.pi
    minstartLon = startLon + dLon * 180 / math.pi

    bus = pois_from_polygon(
        box(minstartLon, minstartLat, maxstartLon, maxstartLat), tags=tags)
    # print(bus.columns)
    busStopStartName = []

    # if there is no start bus stop within 1km, person should default to walk --------------- test
    if bus.empty:
        busflag = 1
        return [astar_path(graph, start_node, end_node), busflag]

    print("\nPossible Starting Stops:")
    for name in bus["name"]:
        name = name.lower()
        for word, initial in words_rep.items():
            name = name.replace(word, initial)
        busStopStartName.append(name.title())
    print(busStopStartName)

    endlat = end[0]
    endLon = end[1]
    R = 6378137
    dn = 250
    de = 250
    dLat = dn / R
    dLon = de / (R * math.cos(math.pi * endlat / 180))
    maxendLat = endlat + dLat * 180 / math.pi
    maxendLon = endLon + dLon * 180 / math.pi
    dn = -250
    de = -250
    dLat = dn / R
    dLon = de / (R * math.cos(math.pi * endlat / 180))
    minendLat = endlat + dLat * 180 / math.pi
    minendLon = endLon + dLon * 180 / math.pi

    bus = pois_from_polygon(
        box(minendLon, minendLat, maxendLon, maxendLat), tags=tags)
    busStopEndName = []

    print("\nPossible Ending Stops:")
    for name in bus["name"]:
        name = name.lower()
        for word, initial in words_rep.items():
            name = name.replace(word, initial)
        busStopEndName.append(name.title())
    print(busStopEndName, "\n")

    startStation = {}
    for stop in busStopStartName:
        startStation.update(
            dict(filter(lambda item: stop in item[0], stop_desc_map.items())))
    endStation = {}
    for stop in busStopEndName:
        endStation.update(
            dict(filter(lambda item: stop in item[0], stop_desc_map.items())))

    results = []
    for x in startStation:
        for y in endStation:
            if startStation[x]["BusStopCode"] != endStation[y]["BusStopCode"]:
                results.append(
                    bfs(busGraph, startStation[x]["BusStopCode"], endStation[y]["BusStopCode"]))
            else:
                print("same start and end stop:",
                      startStation[x]["BusStopCode"])
                busflag = 1
                return [astar_path(graph, start_node, end_node), busflag]

    cheapest = None
    cheapestCost = float("Infinity")

    for cost, distance, transfers, path in results:
        # print(cost, cheapestCost)
        if cost < cheapestCost:
            cheapest = (cost, distance, transfers, path)
            cheapestCost = cost

    cheapestStopsArray = []

    print("CHEAPEST ROUTE: ", cheapest, "\n")
    print("----------------------ROUTE DESCRIPTION-----------------------")
    cost, distance, transfers, path = cheapest
    for code, service in path:
        print(service, stop_code_map[code]["Description"])
        cheapestStopsArray.append(
            (stop_code_map[code]["Latitude"], stop_code_map[code]["Longitude"]))
    print("--------------------------------------------------------------")
    print("Number of stops: ", len(path) - 1)
    print("cost: ", cost)
    print("distance: ", distance, "km")
    print("transfers: ", transfers)

    startStop = cheapestStopsArray[0]
    endStop = cheapestStopsArray[-1]
    return [path, busflag, startStop, endStop]
