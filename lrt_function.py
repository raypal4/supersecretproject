import json
import math
import osmnx as ox

from astar import *
from manualPatch.pois import *

print("Loading LRT JSON")
stops = json.loads(open("punggollrtData/stops.json").read())
routes = json.loads(open("punggollrtData/routes.json").read())
LrtRoute0 = json.loads(open("punggollrtData/LrtRoute0.json").read())
LrtRoute1 = json.loads(open("punggollrtData/LrtRoute1.json").read())
# print("Initializing tables")
stop_desc_map = {stop["Description"]: stop for stop in stops}

# print("Creating lrt route map")
routes_map = {}

for route in routes:
    key = (route["Loop"], route["Direction"])
    if key not in routes_map:
        routes_map[key] = []
    routes_map[key] += [route]

# print("Initializing Graph")
EastLoopGraph = {}
WestLoopGraph = {}

for item in routes_map:
    for index in range(len(routes_map[item])):
        order = routes_map[item][index]["Distance"]
        stationName = routes_map[item][index]["Description"]
        direction = routes_map[item][index]["Direction"]
        loop = routes_map[item][index]["Loop"]
        lat = routes_map[item][index]["Latitude"]
        lon = routes_map[item][index]["Longitude"]
        key = (loop, direction)
        if loop == "East":
            if key not in EastLoopGraph:
                EastLoopGraph[key] = []
            EastLoopGraph[(loop, direction)] += [(order,
                                                  stationName, lat, lon, direction, loop)]
        if loop == "West":
            if key not in WestLoopGraph:
                WestLoopGraph[key] = []
            WestLoopGraph[(loop, direction)] += [(order,
                                                  stationName, lat, lon, direction, loop)]


def isStationInLoop(LoopGraph, station):
    for loop in LoopGraph:
        for index in range(len(LoopGraph[loop])):
            stationX = LoopGraph[loop][index][1]
            if station == stationX:
                # print(station, "found in", loop[0])
                return True
    return False


def findNearestLrt(graph, start, end, start_node, end_node):
    tags = {
        'building': 'train_station',
        'station': 'subway',
        'subway': 'yes'
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
        "punggol mrt/lrt stn/waterway point": "punggol mrt/lrt stn",
        "point": "pt",
        "opposite": "opp",
        "primary": "pr",
        "school": "sch",
        "before": "bef",
        "after": "aft"
    }
    # flag to check if there is a possible lrt route (0 if have, 1 if loc is walkable)
    lrtflag = 0

    startlat = start[0]
    startLon = start[1]
    R = 6378137
    dn = 3000
    de = 3000
    dLat = dn / R
    dLon = de / (R * math.cos(math.pi * startlat / 180))
    maxstartLat = startlat + dLat * 180 / math.pi
    maxstartLon = startLon + dLon * 180 / math.pi
    dn = -3000
    de = -3000
    dLat = dn / R
    dLon = de / (R * math.cos(math.pi * startlat / 180))
    minstartLat = startlat + dLat * 180 / math.pi
    minstartLon = startLon + dLon * 180 / math.pi

    lrt = pois_from_polygon(
        box(minstartLon, minstartLat, maxstartLon, maxstartLat), tags=tags)
    # print(lrt.columns)
    lrtStopStartName = []
    startStationName = []
    # if there is no start lrt stop within 1km, person should default to walk --------------- test
    if lrt.empty:
        lrtflag = 1
        return [astar_path(graph, start_node, end_node), lrtflag]

    # print(lrt["wikipedia"])
    for name in lrt["wikipedia"]:
        # print(name.split(":")[1])
        if isinstance(name, float):
            continue
        name = name.split(":")[1].lower()
        for word, initial in words_rep.items():
            name = name.replace(word, initial)
        startStationName.append(name.title())

    print("\nPossible Starting LRT:")
    print(startStationName)

    shortestStartDistance = float("Infinity")
    nearestStartStop = None
    for s in startStationName:
        try:
            temp = stop_desc_map[s]
        except KeyError:
            continue
        # print(temp)
        distance = geopy.distance.distance(
            start, (temp["Latitude"], temp["Longitude"])).km
        if distance < shortestStartDistance:
            shortestStartDistance = distance
            nearestStartStop = s

    endlat = end[0]
    endLon = end[1]
    R = 6378137
    dn = 3000
    de = 3000
    dLat = dn / R
    dLon = de / (R * math.cos(math.pi * endlat / 180))
    maxendLat = endlat + dLat * 180 / math.pi
    maxendLon = endLon + dLon * 180 / math.pi
    dn = -3000
    de = -3000
    dLat = dn / R
    dLon = de / (R * math.cos(math.pi * endlat / 180))
    minendLat = endlat + dLat * 180 / math.pi
    minendLon = endLon + dLon * 180 / math.pi

    lrt = pois_from_polygon(
        box(minendLon, minendLat, maxendLon, maxendLat), tags=tags)
    lrtStopEndName = []
    endStationName = []

    if lrt.empty:
        lrtflag = 1
        return [astar_path(graph, start_node, end_node), lrtflag]

    # print(lrt["wikipedia"])
    for name in lrt["wikipedia"]:
        # print(name.split(":")[1])
        if isinstance(name, float):
            continue
        name = name.split(":")[1].lower()
        for word, initial in words_rep.items():
            name = name.replace(word, initial)
        endStationName.append(name.title())

    print("\nPossible Ending LRT:")
    print(endStationName)

    shortestEndDistance = float("Infinity")
    nearestEndStop = None
    for s in endStationName:
        try:
            temp = stop_desc_map[s]
        except KeyError:
            continue
            # print(temp)
        distance = geopy.distance.distance(
            end, (temp["Latitude"], temp["Longitude"])).km
        if distance < shortestEndDistance:
            shortestEndDistance = distance
            nearestEndStop = s

    # TO CREATE LRT ROUTING
    path = lrtRouting(EastLoopGraph, WestLoopGraph,
                      nearestStartStop, nearestEndStop)
    return [path, lrtflag, nearestStartStop, nearestEndStop]


def shortestLrt(graph, start, end):
    # print(start, "GO TO", end, "\n")
    endResult = {}
    shortestNumberOfStops = 999999

    for loop in graph:
        startNumber = None
        endNumber = None
        stopsArray = []
        storeArray = []
        for index in range(len(graph[loop])):
            # print(graph[loop][index])
            stopsArray.append(graph[loop][index])
            StartName = graph[loop][index][1]
            EndName = graph[loop][index][1]
            if StartName == start:
                startNumber = index
            if EndName == end:
                endNumber = index

        loopLength = len(stopsArray)
        # print(startNumber, endNumber)
        # max range is the length of the loop - one round
        for i in range(1, loopLength):
            nextIndex = startNumber + i
            if nextIndex <= loopLength:
                storeArray.append(stopsArray[nextIndex - 1])
                if nextIndex - 1 == endNumber:
                    break
            if nextIndex >= loopLength:
                startNumber = 0
                storeArray.append(stopsArray[startNumber])
                if startNumber == endNumber:
                    break

        # print(storeArray, "\n")
        newnumberofstops = len(storeArray)
        if newnumberofstops != loopLength:
            if newnumberofstops < shortestNumberOfStops:
                shortestNumberOfStops = newnumberofstops
                endResult.clear()
                key = (storeArray[0], storeArray[-1])
                if key not in endResult:
                    endResult[key] = []
                endResult[key] = storeArray

    # print("\n------------------------------------------------\n")
    # print(endResult, "\n")
    return endResult


def lrtRouting(EastLoopGraph, WestLoopGraph, start, end):
    finalRoute = []

    startInEastLoop = False
    endInEastLoop = False

    startInEastLoop = isStationInLoop(EastLoopGraph, start)
    endInEastLoop = isStationInLoop(EastLoopGraph, end)

    if (startInEastLoop and endInEastLoop) or (start == "Punggol Mrt/Lrt Stn" and endInEastLoop)or (end == "Punggol Mrt/Lrt Stn" and startInEastLoop):
        print("Do east loop only")
        finalRoute.append(shortestLrt(EastLoopGraph, start, end))
    elif (not startInEastLoop and not endInEastLoop) or (start == "Punggol Mrt/Lrt Stn" and not endInEastLoop) or (end == "Punggol Mrt/Lrt Stn" and not startInEastLoop):
        print("Do west loop only")
        finalRoute.append((shortestLrt(WestLoopGraph, start, end)))
    elif not startInEastLoop and endInEastLoop:
        print("Do west then east loops")
        finalRoute.append(shortestLrt(
            WestLoopGraph, start, "Punggol Mrt/Lrt Stn"))
        finalRoute.append(
            (shortestLrt(EastLoopGraph, "Punggol Mrt/Lrt Stn", end)))
    elif startInEastLoop and not endInEastLoop:
        print("Do east then west loops")
        finalRoute.append(
            (shortestLrt(EastLoopGraph, start, "Punggol Mrt/Lrt Stn")))
        finalRoute.append(
            (shortestLrt(WestLoopGraph, "Punggol Mrt/Lrt Stn", end)))
    else:
        print("No routes found for station", start, "to station", end)

    # for item in finalRoute:
    #     print(item, "\n")

    if finalRoute[0] != None:
        return finalRoute
        # print("No routes found for station", start, "to station", end)
