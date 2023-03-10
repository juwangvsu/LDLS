from lidar_segmentation.segmentation import *
from lidar_segmentation.evaluation import evaluate_instance_segmentation
rst = LidarSegmentationResult.load_file('results.npz')
inst_labels = rst.instance_labels()
results_class_ids = rst.class_labels()
print(results_class_ids.shape)
print(rst.class_ids)
print(inst_labels[7000:8000])
print(results_class_ids[7000:8000])
