import glob
import shutil
import json
# convert labelme json files to a single via format json file
# for jackal dataset only
# mainly convert the shapes entry in labelme to regions entry in via format
# result saved in balloon2/train, val

#Usage: python json_labelme2via.py
#      set json_l and json_dest accordingly

# uncomment to convert train 
json_l=glob.glob("/media/student/data10/arl_aws/ir_labels_s_relabel/albert/json_split/train/class1/*.json")
json_dest = "/media/student/data11/coco/balloon/balloon2/train/via_region_data.json"

# uncomment to convert val
#json_l=glob.glob("/media/student/data10/arl_aws/ir_labels_s_relabel/albert/json_split/val/class1/*.json")
#json_dest = "/media/student/data11/coco/balloon/balloon2/val/via_region_data.json"

#print(json_l)

def get_j_poly(j_a):
    poly2={}
    if len(j_a['shapes'])>0:
        #polygons=a['shapes'][0]['points']
        for poly in j_a['shapes']:
            pts =poly['points']
            xs=[int(x[0]) for x in pts]
            ys=[int(x[1]) for x in pts]
            xs.append(xs[0])
            ys.append(ys[0])
            poly2 = {'name': 'polygon', 'all_points_x': xs, 'all_points_y': ys}
    return poly2

j_j={}
for i in range(len(json_l)):
    fn =json_l[i].split('.')[0].split('/')[-1]
    fn_orig="/media/student/data10/arl_aws/ir_labels/class1/"+fn+".json"
    fn_albert=json_l[i]
    with open(fn_albert, "rb") as infile1:
        j_albert=json.load(infile1)
        j_albert['imageData']=None
        jfn = j_albert['imagePath']
        j_albert['filename']=jfn
        j_poly=get_j_poly(j_albert)
        j_shape={"shape_attributes": j_poly}
        j_regions={'0': j_shape}
        j_albert['regions']=j_regions
        img_id = jfn.split('.')[0].split('/')[-1]
        j_j[img_id]=j_albert
with open(json_dest, "w") as outfile:
#    print(j_j)
    json_object = json.dumps(j_j, indent=4)
    outfile.write(json_object)
