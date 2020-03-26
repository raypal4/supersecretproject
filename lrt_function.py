import json
import math

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


def findNearestBusStopFromLRT(graph, start, end, start_node, end_node):
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
    dn = 400
    de = 400
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
    startStationName = ""
    # if there is no start bus stop within 1km, person should default to walk --------------- test
    if bus.empty:
        busflag = 1
        return [astar_path(graph, start_node, end_node), busflag]

    print("\nPossible Starting LRT:")
    for name in bus["name"]:
        if isinstance(name, str) and ("Stn" in name):
            name = name.lower()
            for word, initial in words_rep.items():
                name = name.replace(word, initial)
            busStopStartName.append(name.title())
    for name in busStopStartName:
        namesplit = name.split(" ")
        for ind in range(len(namesplit)):
            if namesplit[ind].__eq__("Stn"):
                for dex in range(ind + 1):
                    if dex == ind and (namesplit[dex] not in startStationName) and ("Station" not in startStationName):
                        startStationName += "Station"
                    elif dex != ind and namesplit[dex] not in startStationName:
                        startStationName += namesplit[dex] + " "
    print(startStationName)

    endlat = end[0]
    endLon = end[1]
    R = 6378137
    dn = 400
    de = 400
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
    endStationName = ""

    print("\nPossible Ending LRT:")
    for name in bus["name"]:
        if isinstance(name, str) and ("Stn" in name):
            name = name.lower()
            for word, initial in words_rep.items():
                name = name.replace(word, initial)
            busStopEndName.append(name.title())
    for name in busStopEndName:
        namesplit = name.split(" ")
        for ind in range(len(namesplit)):
            if namesplit[ind].__eq__("Stn"):
                for dex in range(ind + 1):
                    if dex == ind and (namesplit[dex] not in endStationName) and ("Station" not in endStationName):
                        endStationName += "Station"
                    elif dex != ind and namesplit[dex] not in endStationName:
                        endStationName += namesplit[dex] + " "
    print(endStationName, "\n")
    # TO CREATE LRT ROUTING
    pathcheck = lrtRouting(EastLoopGraph, WestLoopGraph,
                           startStationName, endStationName)
    return pathcheck


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
    lrtflag = 0

    startInEastLoop = False
    startInWestLoop = False
    endInEastLoop = False
    endInWestLoop = False

    startInEastLoop = isStationInLoop(EastLoopGraph, start)
    if (startInEastLoop == False):
        startInWestLoop = True
        # print(start, "found in West")

    endInEastLoop = isStationInLoop(EastLoopGraph, end)
    if endInEastLoop == False:
        endInWestLoop = True
        # print(end, "found in West")

    if (startInEastLoop == True and endInEastLoop == True) or (start == "Punggol Station" and endInEastLoop == True):
        print("Do east loop only")
        finalRoute.append(shortestLrt(EastLoopGraph, start, end))
    elif (startInWestLoop == True and endInWestLoop == True) or (start == "Punggol Station" and endInWestLoop == True):
        print("Do west loop only")
        finalRoute.append((shortestLrt(WestLoopGraph, start, end)))
    elif startInWestLoop == True and endInEastLoop == True:
        print("Do west then east loops")
        finalRoute.append(shortestLrt(
            WestLoopGraph, start, "Punggol Station"))
        finalRoute.append(
            (shortestLrt(EastLoopGraph, "Punggol Station", end)))
    elif (startInEastLoop == True and endInWestLoop == True):
        print("Do east then west loops")
        finalRoute.append(
            (shortestLrt(EastLoopGraph, start, "Punggol Station")))
        finalRoute.append(
            (shortestLrt(WestLoopGraph, "Punggol Station", end)))

    # for item in finalRoute:
    #     print(item, "\n")

    if finalRoute[0] == None:
        print("No routes found for station", start, "to station", end)
    else:
        return [finalRoute, lrtflag]
