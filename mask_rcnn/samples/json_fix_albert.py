import glob
import shutil
import json
# fix json files in ir_labels_s_relabel/albert/'s ImagePath field
# use original json files in /media/student/data10/arl_aws/ir_labels/class1/
#result saved in /media/student/data10/arl_aws/ir_labels_s_relabel/albert/class1
json_l=glob.glob("/media/student/data10/arl_aws/ir_labels_s_relabel/albert/*.json")
json_l_orig=glob.glob("/media/student/data10/arl_aws/ir_labels/class1/*.json")
dest_folder = "/media/student/data10/arl_aws/ir_labels_s_relabel/albert/class1/"
for i in range(len(json_l)):
    fn =json_l[i].split('.')[0].split('/')[-1]
    fn_orig="/media/student/data10/arl_aws/ir_labels/class1/"+fn+".json"
    fn_new=dest_folder+fn+".json"
    fn_albert=json_l[i]
    with open(fn_albert, "rb") as infile1:
        with open(fn_orig, "rb") as infile2:
            j_albert=json.load(infile1)
            j_orig=json.load(infile2)
            j_albert['imagePath']=j_orig['imagePath']
            json_object = json.dumps(j_albert, indent=4)
            with open(fn_new, "w") as outfile:
                outfile.write(json_object)
