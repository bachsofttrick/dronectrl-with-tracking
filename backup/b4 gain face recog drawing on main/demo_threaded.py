#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function, absolute_import

from timeit import time
import warnings
import cv2
import numpy as np
from PIL import Image
from yolo import YOLO
from deep_sort import preprocessing
from deep_sort import nn_matching
from deep_sort.detection import Detection
from deep_sort.tracker import Tracker
from tools import generate_detections as gdet
from deep_sort.detection import Detection as ddet
warnings.filterwarnings('ignore')

# Custom import lib
from face_recog_lib import Recognizer
from VideoGet import VideoGet
from dronectrl import Drone

def main():
    # Open YOLO
    yolo = YOLO('full')

    # Definition of the parameters
    max_cosine_distance = 0.3
    nn_budget = None
    nms_max_overlap = 1.0
    
    # deep_sort 
    model_filename = 'model_data/mars-small128.pb'
    encoder = gdet.create_box_encoder(model_filename,batch_size=1)
    metric = nn_matching.NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
    tracker = Tracker(metric)

    # Facenet-based face recognizer
    dettect = Recognizer('yolov2')

    # Flag to choose which model to run
    face_flag = True
    yolosort = False
    
    writeVideo_flag = False 
    
    # Open stream
    video_capture = VideoGet("http://192.168.4.106:8080/video").start()
    #video_capture = VideoGet("rtsp://192.168.100.1/encavc0-stream").start()
    
    # Enter drone
    do_you_have_drone = False
    if do_you_have_drone:
        dm107s = Drone().start()
    
    if writeVideo_flag:
    # Define the codec and create VideoWriter object
        w = int(video_capture.get(3))
        h = int(video_capture.get(4))
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        out = cv2.VideoWriter('output.avi', fourcc, 15, (w, h))
        list_file = open('detection.txt', 'w')
        frame_index = -1 

    fps = 0.0
    frame_no = 0

    while True:
        ret, frame = video_capture.update()
        frame_no +=1
        #print('frame no.', frame_no)
        if ret != True:
            break
        t1 = time.time()        
        frame = cv2.flip(frame, 1)
        #frame = cv2.resize(frame, (960, 540))

        # Face recognizer
        if face_flag:
            #pass
            dettect.recognize(frame)

        # Body recognizer
        if yolosort:
            image = Image.fromarray(frame[...,::-1]) #bgr to rgb
            boxs = yolo.detect_image(image)
            features = encoder(frame,boxs)
            
            # score to 1.0 here).
            detections = [Detection(bbox, 1.0, feature) for bbox, feature in zip(boxs, features)]
            
            # Run non-maxima suppression.
            boxes = np.array([d.tlwh for d in detections])
            scores = np.array([d.confidence for d in detections])
            indices = preprocessing.non_max_suppression(boxes, nms_max_overlap, scores)
            detections = [detections[i] for i in indices]
            
            # Call the tracker
            tracker.predict()
            tracker.update(detections)
            
            for track in tracker.tracks:
                if not track.is_confirmed() or track.time_since_update > 1:
                    continue 
                bbox = track.to_tlbr()
                cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,255,255), 2)
                cv2.putText(frame, str(track.track_id),(int(bbox[0]), int(bbox[1])),0, 5e-3 * 200, (0,255,0),2)
                
            for det in detections:
                bbox = det.to_tlbr()
                cv2.rectangle(frame,(int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,0,0), 2)
        
        # Scalable window
        cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
        cv2.imshow('Camera', frame)
        
        if writeVideo_flag:
            # save a frame
            out.write(frame)
            frame_index = frame_index + 1
            list_file.write(str(frame_index)+' ')
            if len(boxs) != 0:
                for i in range(0,len(boxs)):
                    list_file.write(str(boxs[i][0]) + ' '+str(boxs[i][1]) + ' '+str(boxs[i][2]) + ' '+str(boxs[i][3]) + ' ')
            list_file.write('\n')
            
        fps  = ( fps + (1./(time.time()-t1)) ) / 2
        print("fps= %f"%(fps))
        
        # Keypress action
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        if k == ord('r'):
            face_flag = not face_flag
            yolosort = not yolosort
        if do_you_have_drone:
            # Control drone
            # Takeoff and landing
            if k == ord('t'):
                dm107s.takeoff()

            # Throttle
            if k == ord('w'):
                dm107s.throttle_up()
            elif k == ord('s'):
                dm107s.throttle_dwn()

            # Yaw
            if k == ord('a'):
                dm107s.yaw_left()
            elif k == ord('d'):
                dm107s.yaw_right()

            # Pitch
            if k == ord('i'):
                dm107s.pitch_fwd()
            elif k == ord('k'):
                dm107s.pitch_bwd()

            # Roll
            if k == ord('j'):
                dm107s.roll_left()
            elif k == ord('l'):
                dm107s.roll_right()

            # Reset to default values
            if k == ord('e'):
                dm107s.emergency_stop()
            
    # Exiting
    video_capture.stop()
    if do_you_have_drone:
        dm107s.close_connection()
    if writeVideo_flag:
        out.release()
        list_file.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
