import requests  
import json

url = "http://aero.go.usu.edu:8080"
data = {'data':[{'key1':'val1'}, {'key2':'val2'}]}
headers = {'content-type': 'application/json'}

r = requests.post(url, data=json.dumps(data), headers=headers)
print(r.content)