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
lrtGraph = {}

for item in routes_map:
    for index in range(len(routes_map[item])):
        order = routes_map[item][index]["Distance"]
        stationName = routes_map[item][index]["Description"]
        direction = routes_map[item][index]["Direction"]
        loop = routes_map[item][index]["Loop"]
        lat = routes_map[item][index]["Latitude"]
        lon = routes_map[item][index]["Longitude"]
        key = (loop, direction)
        if key not in lrtGraph:
            lrtGraph[key] = []
        lrtGraph[(loop, direction)] += [(order,
                                         stationName, lat, lon, direction)]


def LrtNotBFS(graph, start, end):
    print(start, "GO TO", end)
    endResult = {}
    shortestNumberOfStops = 999999
    secondloopFlag = 0

    for loop in graph:
        storeArray = []
        print(loop, "-----------------------------\n")
        for index in range(len(graph[loop])):
            StartName = graph[loop][index][1]
            if StartName == start:
                startNumber = graph[loop][index][0]
                storeArray.append((graph[loop][index]))
                print("Start Stop: ", StartName, startNumber, "\n")
                break
        for index in range(len(graph[loop])):
            EndName = graph[loop][index][1]
            if EndName == end:
                secondloopFlag = 0
                endNumber = graph[loop][index][0]
                storeArray.append((graph[loop][index]))
                print("End Stop: ", EndName, endNumber, "\n")
                break
            else:
                secondloopFlag = 1
                storeArray.append((graph[loop][0]))
                endNumber = len(graph[loop])

        newShortestNumberOfStops = endNumber - startNumber
        if abs(newShortestNumberOfStops) < shortestNumberOfStops:
            shortestNumberOfStops = abs(newShortestNumberOfStops)
            key = (storeArray[0], storeArray[1])
            endResult.clear()
            if key not in endResult:
                endResult[key] = 0
            endResult[key] += shortestNumberOfStops

        # if secondloopFlag == 1:
        #     storeArray = []
        #     StartName = (graph[loop][0])
        #     for index in range(len(graph[loop])):
        #         EndName = graph[loop][index][1]
        #         if EndName == end:
        #             endNumber = graph[loop][index][0]
        #             storeArray.append((graph[loop][index]))
        #             print("End Stop: ", EndName, endNumber, "\n")
        #             break

    for item in endResult:
        print(item, endResult[item], "\n")


LrtNotBFS(lrtGraph, "Cove Station", "Nibong Station")


def lrt(lrtGraph, graph, start, end, start_node, end_node):
    tags = {
        'highway': 'bus_stop',
        'building': 'train_station'
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
