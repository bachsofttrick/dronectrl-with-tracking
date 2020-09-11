# Object Detection and Monitoring Through Unmanned Aerial Vehicle
### By Nguyen Hoang An, Phan Xuan Bach
This project offer a solution for object detection and monitoring through unmanned aerial vehicle, specifically a drone. The drone will search, identify and follow a designated object. It can also transmit images directly to a computer, smartphone or any devices which supports RTSP protocol. It allows the observer to monitor, control, and at the same time, it can learn and remember the object temporarily so that whenever they meet, the drone can be ready to follow this object autonomously.
It is built from the ground up with a custom drone and controlled using RX701 remote control or automatic controller.

## DM107s Specifications:
Brand: DA MING
Item Name:  Foldable Drone
Item No.: DM107S
Frequency: 2.4 GHz
Channels: 4
Gyro: 6 Axis
Product Battery: 3.7V 1000mAh (Included)
Charging Time: About 80-100mins
Flying Time: About 8-10mins
Transmitter Battery: 1.5V AA X 2 (Not Included)
R/C Distance: 80-100m
WIFI Distance: 50-80m
Color: Black
Camera: 2MP Wide-angle

## Naza Drone Specifications:
* Frame: Tarot Iron man 650
* Flight controller: NAZA-M Lite
* Engine: 4x980 KV
* Remote control: RX701 + Devo 7

## Automatic Controller Specifications:
* Main unit: Raspberry Pi 3 Model B+
* Camera: Raspberry Pi Camera V2 - 8MP
* Control unit: Arduino Uno R3

## Getting started
1. Create 2 folders "dataset" and "raw" inside of "dataset". In "raw", create any number of folders based on how many people needed to be recognized by the drone. Also, create "unknown" folder for random faces that doesn't belong to any one of the faces in dataset.

2. Create a dataset with only 160x160 faces.
```
python align_dataset_mtcnn.py dataset/raw dataset/processed --image_size 160 --random_order --gpu_memory_fraction 0.6
```
gpu_memory_fraction 0.6 is needed to avoid overloading GPU VRAM.

3. Train the network
```
python classifier.py TRAIN dataset/processed Models/20180402-114759.pb models/facemodel.pkl --batch_size 1000
```

4. Use 1 of 3 files to run program:
* demo.py is for example with pre-existing video files
* demo_threaded.py is for DM107s drone
* demo_naza.py is for NAZA-M Lite drone

5. At this line, it can be either "tiny" or "full" for 2 version of YOLOv3 as person detector.
```
yolo = YOLO('tiny')
```
This line has 3 options: "mtcnn" for MTCNN, "yolov2" for YOLOv2-tiny, "resnet10" for SSD ResNet10 as face detector.
```
face_dettect = Recognizer('resnet10')
```

6. Choose person to detect face and follow.
```
person_to_follow = 'bach'
```

7. When in program GUI, use the following keys:
* w/s is for throttle up/down
* a/d is for yaw left/right
* i/k is for pitch forward/backward
* j/l is for roll left/right
* r is for changing between face recognition and person tracking (not available on demo.py)
* t is for auto takeoff (changing between face recognition and person tracking on demo.py)
* u is for auto-throttle (algorithm takes cantrol of altitude)
* o is for autopilot (if auto-throttle is disabled, only yaw and pitch are taken over)
* q is for exit
* e is for emergency shutdown (only available on demo_threaded.py)
* v is for gyroscope calibration (only available on demo_threaded.py)

8. In normal operation, the program detects face first, then after a face is recognized, the program will switch to person tracking. However, if you are in these senarios:
* use "r" to change mode
* want to cancel current person being tracked

then press "c" to cancel the person's id being tracked. After that, press "o" to disable autopilot. Type your preferred ID to track and press Enter.