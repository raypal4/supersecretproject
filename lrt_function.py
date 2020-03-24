import json
import math

from astar import *
from manualPatch.pois import *

print("Loading JSON")
stops = json.loads(open("punggollrtData/stops.json").read())
routes = json.loads(open("punggollrtData/routes.json").read())
LrtRoute0 = json.loads(open("punggollrtData/LrtRoute0.json").read())
LrtRoute1 = json.loads(open("punggollrtData/LrtRoute1.json").read())
print("Initializing tables")
stop_desc_map = {stop["Description"]: stop for stop in stops}

print("Creating lrt route map")
routes_map = {}

for route in routes:
    key = (route["Loop"], route["Direction"])
    if key not in routes_map:
        routes_map[key] = []
    routes_map[key] += [route]

print("Initializing Graph")
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


def shortestLrt(graph, start, end):
    # print(start, "GO TO", end, "\n")
    endResult = {}
    shortestNumberOfStops = 999999

    for loop in graph:
        storeArray = []
        startNumber = None
        endNumber = None
        # print(loop, "-----------------------------\n")
        for index in range(len(graph[loop])):
            StartName = graph[loop][index][1]
            if StartName == start:
                startNumber = graph[loop][index][0]
                storeArray.append((graph[loop][index]))
                # print("Start Stop: ", StartName, startNumber, "\n")
                break
        for index in range(len(graph[loop])):
            EndName = graph[loop][index][1]
            if EndName == end:
                endNumber = graph[loop][index][0]
                storeArray.append((graph[loop][index]))
                # print("End Stop: ", EndName, endNumber, "\n")
                break

        if (endNumber == None) or (startNumber == None):
            return None

        newShortestNumberOfStops = endNumber - startNumber
        if newShortestNumberOfStops < shortestNumberOfStops:
            shortestNumberOfStops = newShortestNumberOfStops
            endResult.clear()
            key = (storeArray[0], storeArray[1])
            if key not in endResult:
                endResult[key] = 0
            endResult[key] += shortestNumberOfStops

    # for item in endResult:
    #     print(item, endResult[item])
    if len(endResult) > 0:
        return endResult


# for item in EastLoopGraph:
#     print(item, EastLoopGraph[item], "\n")

# print("-----------------------------------------------------\n")

# for item in WestLoopGraph:
#     print(item, WestLoopGraph[item], "\n")

def lrtRouting(EastLoopGraph, WestLoopGraph, start, end):
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
        # print("Do east loop only")
        finalRoute.append(shortestLrt(EastLoopGraph, start, end))
    elif (startInWestLoop == True and endInWestLoop == True) or (start == "Punggol Station" and endInWestLoop == True):
        # print("Do west loop only")
        finalRoute.append((shortestLrt(WestLoopGraph, start, end)))
    elif startInWestLoop == True and endInEastLoop == True:
        # print("Do west then east loops")
        finalRoute.append(shortestLrt(
            WestLoopGraph, start, "Punggol Station"))
        finalRoute.append(
            (shortestLrt(EastLoopGraph, "Punggol Station", end)))
    elif (startInEastLoop == True and endInWestLoop == True):
        # print("Do east then west loops")
        finalRoute.append(
            (shortestLrt(EastLoopGraph, start, "Punggol Station")))
        finalRoute.append(
            (shortestLrt(WestLoopGraph, "Punggol Station", end)))

    if finalRoute[0] == None:
        print("No routes found for station", start, "to station", end)
    else:
        return [lrtflag, finalRoute]
