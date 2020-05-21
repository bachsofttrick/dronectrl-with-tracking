import cv2
import os
from customlibs.resnet_lib import Resnetdnn
import tensorflow as tf

raw_folder = 'dataset/raw'
processed_folder = 'dataset/processed2'
model_rnet = Resnetdnn()

for class_folder in os.listdir(raw_folder):
    img_file = os.path.join(raw_folder,class_folder)
    output_class_dir = os.path.join(processed_folder,class_folder)
    if not os.path.exists(output_class_dir):
        os.makedirs(output_class_dir)
    for file in os.listdir(img_file):
        load_name = os.path.join(img_file,file)
        print('read ',load_name)
        frame = cv2.imread(load_name)
        file = os.path.splitext(file)[0]
        bounding_boxes = model_rnet.detect(frame, 0.3)
        faces_found = len(bounding_boxes)
        if faces_found > 0:
            for i in range(faces_found):
                save_name = os.path.join(output_class_dir,file) + ".png"
                cropped = frame[bounding_boxes[i][1]:bounding_boxes[i][3], bounding_boxes[i][0]:bounding_boxes[i][2], :]
                scaled = cv2.resize(cropped, (160, 160),interpolation=cv2.INTER_CUBIC)
                cv2.imwrite(save_name, scaled)
                print('saved ', save_name)
        else:
            print('no face found ',load_name)
            continue
