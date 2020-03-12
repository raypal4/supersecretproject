import json

with open('routes.geojson') as f:
	data = json.load(f)
	f.close()

busroute0 = {}
busroute1 = {}
for i in data["features"]:
	j = 0
	while j < len(i["geometry"]["coordinates"]):
		latlong = i["geometry"]["coordinates"][j]
		if len(latlong) == 2:
			x,y = latlong
		if not ((1.393925 < y < 1.415805) and (103.895516 < x < 103.923669)):
			del i["geometry"]["coordinates"][j]
		else:
			j += 1
		if len(i["geometry"]["coordinates"]) == 0:
			del i
			break

	if 'i' not in vars().keys():
		continue
	bus = i["properties"]
	if bus["route"] == 0:
		busroute0[bus["number"]] = i["geometry"]
	elif bus["route"] == 1:
		busroute1[bus["number"]] = i["geometry"]
	else:
		print("uhhhh")

with open('busroute0.json', 'w') as json_file:
	json.dump(busroute0, json_file, indent=4)
	json_file.close()

with open('busroute1.json', 'w') as json_file:
	json.dump(busroute1, json_file, indent=4)
	json_file.close()


# 1.393925,103.895516
# 1.415805,103.923669