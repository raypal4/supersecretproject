import json
import urllib
import httplib2 as http #External library

if __name__=="__main__":
#Authentication parameters
	headers = { 'AccountKey' : 'CYj6hSM1TjmtZD2VwzrY4A==', 'accept' : 'application/json'}

#API parameters
uri = 'http://datamall2.mytransport.sg/' #Resource URL
path = '/ltaodataservice/BusStops?$skip='
for i in range(0, 50000, 500):
	#Build query string & specify type of API call
	target = uri + path + str(i)
	print(target)
	method = 'GET'
	body = ''

	#Get handle to http
	h = http.Http()
	#Obtain results
	response, content = h.request(target, method, body, headers)
	#Parse JSON to print
	jsonObj = json.loads(content)
	# print(json.dumps(jsonObj, sort_keys=True, indent=4))
	#Save result to file
	with open("test/bus_stops.json","a") as outfile:
		#Saving jsonObj["d"]
		json.dump(jsonObj, outfile, sort_keys=True, indent=4,ensure_ascii=False)