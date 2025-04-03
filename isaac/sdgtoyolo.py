import numpy as np
import os
from enum import Enum
from glob import glob
import shutil
import json
from PIL import Image

data_input_folder = "10000_img_run"
cameras = 3
num_images = 10000
data_output_folder = "yolo_training_dataset"
test_images = num_images > 100
train = os.path.join("images", "train")
val = os.path.join("images", "val")
test = os.path.join("images", "test")
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480



class DataClasses(Enum):
    STEEL = 2
    NYLON = 1
    BRASS = 0

def create_file_structure():
    os.makedirs(data_output_folder, exist_ok=True)
    path = data_output_folder
    
    os.makedirs(os.path.join(path, train), exist_ok=True)
    os.makedirs(os.path.join(path, os.path.join("labels", "train")), exist_ok=True)
    
    os.makedirs(os.path.join(path, val), exist_ok=True)
    os.makedirs(os.path.join(path, os.path.join("labels", "val")), exist_ok=True)
    
    if test_images:
        os.makedirs(os.path.join(path, test), exist_ok=True)
        os.makedirs(os.path.join(path, os.path.join("labels", "test")), exist_ok=True)

  


    with open(os.path.join(data_output_folder, "dataset.yaml"), "w+") as f:
        f.write(f"""path: .
train: {train}
val: {val}
{f'test: {test}' if test_images else ''}

names:
""")
        f.writelines([f"    {e.value}: {e.name}\n" for e in DataClasses])
    
def copy_images():
    for filename in glob(os.path.join(data_input_folder, "rgb", "*.png")):
        frame_mod_10 = int(filename[-5:-4])
        print(filename, frame_mod_10)
        img = Image.open(filename)
        match frame_mod_10:
            case num if 0 <= num <= 5:
                print("Train")
                shutil.copy2(filename, os.path.join(data_output_folder, train))
            case num if 6 <= num <= 7:
                print("Val")
                shutil.copy2(filename, os.path.join(data_output_folder, val))
            case num if 8 <= num <= 9:
                if test_images:
                    print("Test")
                    shutil.copy2(filename, os.path.join(data_output_folder, test))
                else:
                    print("Val")
                    shutil.copy2(filename, os.path.join(data_output_folder, val))

            case _:
                print("Train")

def remake_labels():
    for filename in glob(os.path.join(data_input_folder, "bounding_box_2d_tight", "*.npy")):
        filename_parts = filename.split("_")
        filename_parts.insert(-1, "labels")
        labels_filename = "_".join(filename_parts)[:-4] + ".json"
        metadata = np.load(filename)
        with open(labels_filename, "r") as f:
            labels = json.loads(f.read())
        print(filename, labels_filename)
        print(labels)
        
        yolo_format = []
        for instance in metadata:
            label, x_min, y_min, x_max, y_max, occlusionRatio = instance
            print(instance)
            width = x_max - x_min
            height = y_max - y_min
            center_x = x_min + width // 2
            center_y = y_min + height // 2
            
            class_label = DataClasses[str(labels[str(label)]["class"]).upper()].value
            yolo_format.append((class_label, center_x / IMAGE_WIDTH, center_y / IMAGE_HEIGHT, width / IMAGE_WIDTH, height / IMAGE_HEIGHT))
        
        section = "train"
        num_mod_10 = int(filename[-5:-4])
        if num_mod_10 >= 6:
            if test_images and num_mod_10 >= 8:
                section = "test"
            else:
                section = "val"
        
        paths = filename.split("/")[2].split("_")
        paths.pop(-2)
        paths.pop(-2)
        paths.pop(-2)
        paths.pop(-2)
        paths.insert(-1, "rgb")
        
        text_path = os.path.join(data_output_folder, "labels", section, "_".join(paths))[:-4] + ".txt"
        print(text_path)
        print(section)
        
        with open(text_path, "w+") as f: 
            f.writelines([" ".join([str(seg) for seg in x]) + "\n" for x in yolo_format])

        
def main():
    create_file_structure()
    copy_images()
    remake_labels()

if __name__ == "__main__":
    main()


