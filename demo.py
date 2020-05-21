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
from customlibs.face_recog_lib import Recognizer
from time import sleep

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
    face_dettect = Recognizer('resnet10')

    # Flag to choose which model to run
    face_flag = True
    yolosort = False
    
    # Flag to override autopilot
    auto_engaged = False
    
    # To be decided
    temp_face = None
    confirmed_number = 0
    
    writeVideo_flag = False 
    
    # Open stream
    video_capture = cv2.VideoCapture("demo2.mp4")

    # Enter safety zone coordinate
    safety_x = 100
    safety_y = 100
    
    if writeVideo_flag:
    # Define the codec and create VideoWriter object
        w = int(video_capture.get(3))
        h = int(video_capture.get(4))
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        out = cv2.VideoWriter('output.avi', fourcc, 24, (w, h))
        list_file = open('detection.txt', 'w')
        frame_index = -1 
    
    n = 0
    fps = 0.0

    while True:
        ret, frame = video_capture.read()  
        if ret != True:
            break
        t1 = time.time()
        #frame = cv2.flip(frame, 1)
        
        # Resize frame
        resize_to = (1280, 720)
        resize_div_2 = (int(resize_to[0]/2), int(resize_to[1]/2))
        frame = cv2.resize(frame, resize_to)

        # skip frame to remain real-time
        Frameskip = False
        if 9 != n and Frameskip:
            n+=1
            continue
        n = 0
        
        # Face recognizer
        if face_flag:
            person_to_follow = 'bach'
            face_bbox = face_dettect.recognize(frame, person_to_follow)
            for i in range(len(face_bbox)):
                face_name = face_bbox[i][4]
                if face_name == person_to_follow:
                    temp_face = face_bbox[i][0:4]
                    
                    # This calculates the vector from your ROI to the center of the screen
                    vector_true = np.array((resize_div_2[0], resize_div_2[1], 25000))
                    center_of_bound_box = np.array(((face_bbox[i][0] + face_bbox[i][2])/2, (face_bbox[i][1] + face_bbox[i][3])/2))
                    vector_target = np.array((int(center_of_bound_box[0]), int(center_of_bound_box[1]), int(face_bbox[i][2] - face_bbox[i][0]) * int(face_bbox[i][3] - face_bbox[i][1])))
                    vector_distance = vector_true-vector_target
                    
                    if auto_engaged:
                        if vector_distance[0] < -safety_x:
                            print("Yaw left.")
                        elif vector_distance[0] > safety_x:
                            print("Yaw right.")
                        else:
                            pass
                        
                        if vector_distance[1] > safety_y:
                            print("Fly up.")
                        elif vector_distance[1] < -safety_y:
                            print("Fly down.")
                        else:
                            pass
                        
                        if vector_distance[2] > 10000:
                            print("Push forward")
                        elif vector_distance[2] < -1000:
                            print("Pull back")
                        else:
                            pass
                    
                    print_out = str(int(vector_distance[0])) + " " + str(int(vector_distance[1])) + " " + str(int(vector_distance[2]))
                    cv2.circle(frame, (int(center_of_bound_box[0]), int(center_of_bound_box[1])), 5, (0,100,255), 2)
                    cv2.putText(frame, print_out,(0, (frame.shape[0] - 10)),0, 0.8, (0,255,0),2)
                
                # Draw bounding box over face
                cv2.rectangle(frame, (face_bbox[i][0], face_bbox[i][1]), (face_bbox[i][2], face_bbox[i][3]), (0, 255, 0), 2)
                _text_x = face_bbox[i][0]
                _text_y = face_bbox[i][3] + 20
                
                # Write name on frame
                cv2.putText(frame, str(face_name), (_text_x, _text_y + 17*2), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                            1, (255, 255, 255), thickness=1, lineType=2)
                cv2.putText(frame, str(face_bbox[i][0:4]), (_text_x, _text_y), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                            1, (255, 255, 255), thickness=1, lineType=2)
                cv2.putText(frame, str(round(face_bbox[i][5][0], 3)), (_text_x, _text_y + 17),
                            cv2.FONT_HERSHEY_COMPLEX_SMALL,
                            1, (255, 255, 255), thickness=1, lineType=2)
                            
                # Draw the safety zone
                cv2.rectangle(frame, (resize_div_2[0] - safety_x, resize_div_2[1] - safety_y), (resize_div_2[0] + safety_x, resize_div_2[1] + safety_y), (0,255,255), 2)

        # Face recognizer
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
                # Only track 1 person (WIP)
                '''
                if temp_face:
                    number_of_true = 0
                    number_of_true = (number_of_true + 1) if temp_face[0] > bbox[0] else number_of_true
                    number_of_true = (number_of_true + 1) if temp_face[1] > bbox[1] else number_of_true
                    number_of_true = (number_of_true + 1) if temp_face[2] < bbox[2] else number_of_true
                    number_of_true = (number_of_true + 1) if temp_face[3] < bbox[3] else number_of_true
                    if number_of_true == 4:
                        confirmed_number = track.track_id
                    else:
                        print("Retry capture.")
                    temp_face = None
                if confirmed_number != track.track_id:
                    continue
                '''
                cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,255,255), 2)
                cv2.putText(frame, str(track.track_id),(int(bbox[0]), int(bbox[1])),0, 5e-3 * 200, (0,255,0),2)
            
            for det in detections:
                bbox = det.to_tlbr()
                cv2.rectangle(frame,(int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,0,0), 2)
            
        # Draw the center of frame as a circle
        middle_of_frame = (int(resize_div_2[0]), int(resize_div_2[1]))
        cv2.circle(frame, middle_of_frame, 5, (255,128,0), 2)
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
        if k == ord('t'):
            face_flag = not face_flag
            yolosort = not yolosort
        if k == ord('o'):
            auto_engaged = not auto_engaged
        
    # Exiting
    video_capture.release()
    if writeVideo_flag:
        out.release()
        list_file.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
