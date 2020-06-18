import cv2

# Library for MobileNet SSD object detector in OpenCV dnn module
class Mobilenetdnn:
    def __init__(self):
        modelFile = "Models/MobileNetSSD_deploy.caffemodel"
        configFile = "Models/MobileNetSSD_deploy.prototxt"
        self.model_mnet = cv2.dnn.readNetFromCaffe(configFile, modelFile)
        self.classNames = { 0: 'background',
        1: 'aeroplane', 2: 'bicycle', 3: 'bird', 4: 'boat',
        5: 'bottle', 6: 'bus', 7: 'car', 8: 'cat', 9: 'chair',
        10: 'cow', 11: 'diningtable', 12: 'dog', 13: 'horse',
        14: 'motorbike', 15: 'person', 16: 'pottedplant',
        17: 'sheep', 18: 'sofa', 19: 'train', 20: 'tvmonitor' }

    def detect(self, frame, conf_threshold):
        frameWidth = frame.shape[1]
        frameHeight = frame.shape[0]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), (127.5, 127.5, 127.5), False)
        self.model_mnet.setInput(blob)
        mnet_detections = self.model_mnet.forward()
        bboxes = []
        for i in range(mnet_detections.shape[2]):
            obj_name = self.classNames[mnet_detections[0, 0, i, 1]]
            if obj_name != 'person':
                break
            mnet_confidence = mnet_detections[0, 0, i, 2]
            if mnet_confidence > conf_threshold:
                x1 = int(mnet_detections[0, 0, i, 3] * frameWidth)
                y1 = int(mnet_detections[0, 0, i, 4] * frameHeight)
                x2 = int(mnet_detections[0, 0, i, 5] * frameWidth)
                y2 = int(mnet_detections[0, 0, i, 6] * frameHeight)
                w = x2 - x1
                h = y2 - y1
                bboxes.append([x1,y1,w,h])
        return bboxes
