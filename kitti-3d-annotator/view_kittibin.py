from annotator.example_viewer import ExampleViewer

from skimage.io import imread, imshow
from pathlib import Path

def main():
    # image_path = parser['image_path']
    # if image_path is not None:
    #     if not Path(image_path).exists():
    #         print("Image path %s not found" % image_path)
    #         image_path = None

    print("Enter kitti bin file path to view. Leave blank and hit ENTER to quit.")
    binfilepath = input("Input bin filepath: ")
    viewer = ExampleViewer()
    viewer.view1(binfilepath)

if __name__ == "__main__":
    main()
