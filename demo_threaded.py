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
    
    # To be decided
    temp_face = None
    confirmed_number = 0
    
    writeVideo_flag = False 
    
    # Open stream
    #video_capture = VideoGet("http://192.168.4.106:8080/video").start()
    video_capture = VideoGet("rtsp://192.168.100.1/encavc0-stream").start()
    #video_capture = VideoGet('0').start()
    
    # Enter drone
    do_you_have_drone = True
    if do_you_have_drone:
        dm107s = Drone().start()
    
    if writeVideo_flag:
    # Define the codec and create VideoWriter object
        w = int(video_capture.get(3))
        h = int(video_capture.get(4))
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        out = cv2.VideoWriter('output.avi', fourcc, 24, (w, h))
        list_file = open('detection.txt', 'w')
        frame_index = -1 

    fps = 0.0

    while True:
        ret, frame = video_capture.update()
        if ret != True:
            break
        t1 = time.time()        
        #frame = cv2.flip(frame, 1)
        
        # Resize frame
        frame = cv2.resize(frame, (1280, 720))

        # Face recognizer
        if face_flag:
            #pass
            face_bbox = dettect.recognize(frame)
            for i in range(len(face_bbox)):
                face_name = face_bbox[i][4]
                if face_name == 'bach':
                    temp_face = face_bbox[i][0:4]
                    
                    # This calculates the vector from your ROI to the center of the screen
                    vector_true = np.array((resize_div_2[0], resize_div_2[1], 25000))
                    center_of_bound_box = np.array(((face_bbox[i][0] + face_bbox[i][2])/2, (face_bbox[i][1] + face_bbox[i][3])/2))
                    vector_target = np.array((int(center_of_bound_box[0]), int(center_of_bound_box[1]), int(face_bbox[i][2] - face_bbox[i][0]) * int(face_bbox[i][3] - face_bbox[i][1])))
                    vector_distance = vector_true-vector_target
                    '''
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
                    elif vector_distance[2] < -10000:
                        print("Pull back")
                    else:
                        pass
                    '''
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
            velocity = 30
            # Control drone
            # Takeoff and landing
            if k == ord('t'):
                dm107s.takeoff()

            # Throttle
            if k == ord('w'):
                #dm107s.throttle_up()
                dm107s.incremt(0,0,velocity,0)
            elif k == ord('s'):
                #dm107s.throttle_dwn()
                dm107s.incremt(0,0,-velocity,0)

            # Yaw
            if k == ord('a'):
                #dm107s.yaw_left()
                dm107s.incremt(0,0,0,velocity)
            elif k == ord('d'):
                #dm107s.yaw_right()
                dm107s.incremt(0,0,0,-velocity)

            # Pitch
            if k == ord('i'):
                #dm107s.pitch_fwd()
                dm107s.incremt(0,velocity,0,0)
            elif k == ord('k'):
                #dm107s.pitch_bwd()
                dm107s.incremt(0,-velocity,0,0)

            # Roll
            if k == ord('j'):
                #dm107s.roll_left()
                dm107s.incremt(-velocity,0,0,0)
            elif k == ord('l'):
                #dm107s.roll_right()
                dm107s.incremt(velocity,0,0,0)
            
            if k == ord('f'):
                dm107s.incremt(0,0,0,0)

            # STOP NOW
            if k == ord('e'):
                dm107s.emergency_stop()
            
            # Calibrate gyro
            if k == ord('c'):
                dm107s.calib_gyro()
            
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
