import json

with open('stops.json') as f:
	data = json.load(f)
	f.close()

busstop = {}
for i in data:
	number = i["number"]
	services = i["services"]
	name = i["name"]
	contour = i["contour"]
	for c in contour:
		a = 0
		while a < len(c):
			x, y = c[a]
			if not ((1.393925 < y < 1.415805) and (103.895516 < x < 103.923669)):
				del c[a]
			else:
				a += 1
		if len(c) == 0:
			del i
	if 'i' not in vars().keys():
		continue
	busstop[number] = {"name":name, "services":services, "contour":contour}

with open('busstop.json', 'w') as json_file:
	json.dump(busstop, json_file, indent=4)
	json_file.close()


# 1.393925,103.895516
# 1.415805,103.923669