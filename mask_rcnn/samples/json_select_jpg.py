import glob
import shutil
# copy .jpg file from arl_aws/ir_labels/ to arl_aws/ir_labels_s/ based on the .json files
# in arl_aws/ir_labels_s/class1/

json_l=glob.glob("/media/student/data10/arl_aws/ir_labels_s_relabel/albert/json_split/train/class1/*.json")
#json_l=glob.glob("/media/student/data10/arl_aws/ir_labels_s_relabel/albert/json_split/val/class1/*.json")
#json_l=glob.glob("/media/student/data10/arl_aws/ir_labels_s/class1/*.json")

fn_dst_folder='/media/student/data11/coco/balloon/balloon2/train/'
#fn_dst_folder='/media/student/data11/coco/balloon/balloon2/val/'
#fn_dst_folder='/media/student/data10/arl_aws/ir_labels_s/'

for i in range(len(json_l)):
    fn =json_l[i].split('.')[0].split('/')[-1]
    fn_src="/media/student/data10/arl_aws/ir_labels/"+fn+".jpg"
    fn_dst=fn_dst_folder+fn+".jpg"
    shutil.copyfile(fn_src, fn_dst)
