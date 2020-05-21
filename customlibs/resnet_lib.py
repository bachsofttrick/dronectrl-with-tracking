import cv2

# Library for Resnet10 face detector in OpenCV dnn module
class Resnetdnn:
    def __init__(self):
        modelFile = "Models/opencv_face_detector_uint8.pb"
        configFile = "Models/opencv_face_detector.pbtxt"
        self.model_rnet = cv2.dnn.readNetFromTensorflow(modelFile, configFile)

    def detect(self, frame, conf_threshold):
        frameWidth = frame.shape[1]
        frameHeight = frame.shape[0]
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), [104, 117, 123], False, False)
        self.model_rnet.setInput(blob)
        rnet_detections = self.model_rnet.forward()
        bboxes = []
        for i in range(rnet_detections.shape[2]):
            rnet_confidence = rnet_detections[0, 0, i, 2]
            if rnet_confidence > conf_threshold:
                x1 = int(rnet_detections[0, 0, i, 3] * frameWidth)
                y1 = int(rnet_detections[0, 0, i, 4] * frameHeight)
                x2 = int(rnet_detections[0, 0, i, 5] * frameWidth)
                y2 = int(rnet_detections[0, 0, i, 6] * frameHeight)
                bboxes.append([x1,y1,x2,y2])
        return bboxes
