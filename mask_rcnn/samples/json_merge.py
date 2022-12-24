import json
import glob

result = []
for f in glob.glob("/media/student/data10/arl_aws/ir_labels/json_split/val/class1/*.json"):
    with open(f, "rb") as infile:
        j1=json.load(infile)
        j1['imageData']=None
        result.append(j1)

with open("merged_file.json", "w") as outfile:
    json.dump({"items": result}, outfile)
