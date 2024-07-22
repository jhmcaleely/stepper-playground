import json

cursor = 90
newCursor = 89

with open("config.json", "w") as f:
    json.dump(cursor, f)
    
with open("config.json", "r") as f:
    newCursor = json.load(f)
    print(newCursor)