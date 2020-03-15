import json
import operator

with open("test/full_bus_stops.json","r") as f:
    data = json.load(f)
    f.close()

stopsvalue = data["value"]
i = 0
stops = []
while i < len(stopsvalue):
	y = stopsvalue[i]["Latitude"]
	x = stopsvalue[i]["Longitude"]
	if not ((103.895516 < x < 103.923669) and (1.393925 < y < 1.415805)):
		del stopsvalue[i]
	else:
		stops.append(stopsvalue[i]["BusStopCode"])
		i += 1

# print(json.dumps(stopsvalue, sort_keys=True, indent=4))

with open("test/full_bus_routes.json","r") as f:
    data = json.load(f)
    f.close()

routesvalue = data["value"]
i = 0
while i < len(routesvalue):
	if routesvalue[i]["BusStopCode"] not in stops:
		del routesvalue[i]
	else:
		i += 1

routesvalue.sort(key = operator.itemgetter("ServiceNo", "Direction", "StopSequence"))

distance = 0
service = 0
direction = 0
for route in routesvalue:
	if service != route["ServiceNo"] or direction != route["Direction"]:
		distance = route["Distance"]
		route["Distance"] = 0
		service = route["ServiceNo"]
		direction = route["Direction"]
	else:
		route["Distance"] -= distance
		distance = route["Distance"]

# print(json.dumps(routesvalue, indent=4))

with open("test/punggol_bus_stops.json","w") as f:
	json.dump(stopsvalue, f, indent=4,ensure_ascii=False)
	f.close()

with open("test/punggol_bus_routes.json","w") as f:
	json.dump(routesvalue, f, indent=4,ensure_ascii=False)
	f.close()

# 1.3925,103.8904
# 1.4216,103.9241