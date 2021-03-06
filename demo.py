#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function, absolute_import

from timeit import time
from time import strftime
from math import sqrt
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
from customlibs.mobilenet_lib import Mobilenetdnn

def main():
    # Open YOLO
    #yolo = YOLO('tiny')
    
    # Open MobileNet SSD
    model_mnet = Mobilenetdnn()
    
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
    person_to_follow = 'bach'
    face_dettect = Recognizer('resnet10')
    # Count how many frames until we switch to person-tracking
    person_found = False
    pno = 0
    total_pno = 0

    # Flag to choose which model to run
    face_flag = True
    yolosort = False
    
    # Flag to override autopilot
    auto_engaged = False
    
    # Transfer to person tracking
    person_to_track = None
    face_locked = False
    confirmed_number = 0
    # String to convert to ID that needs tracking
    confirmed_string = ""
    
    # Open stream
    video_capture = cv2.VideoCapture("demo2.mp4")

    # Enter safety zone coordinate
    # For face tracking
    safety_x = 100
    safety_y = 100
    # For person tracking
    safety_x_person = 150
    safety_y_person = 100
    
    writeVideo_flag = False 
    if writeVideo_flag:
    # Define the codec and create VideoWriter object
        w = 1280
        h = 720
        localtime = strftime("m%md%d-%H%M%S")
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        out = cv2.VideoWriter('output %s.avi' % localtime, fourcc, 24, (w, h))
        frame_index = -1 
    
    n = 0
    fps = 0.0
    fno = 0

    while True:
        ret, frame = video_capture.read()  
        fno += 1
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
        if 3 != n and Frameskip:
            n+=1
            continue
        n = 0
        
        # Show control on the right corner of frame
        control_disp = ""        
        
        # Show autopilot status
        if auto_engaged:
            print_out = "AUTOPILOT "
        else:
            print_out = "MANUAL "
        
        # Show model in use
        if face_flag:
            model_in_use = "FACE"
        elif yolosort:
            model_in_use = "PERSON"
        
        # Face recognizer
        vector_true = np.array((resize_div_2[0], resize_div_2[1]))
        if face_flag:
            face_bbox = face_dettect.recognize(frame, person_to_follow)
            person_found = False
            if auto_engaged:
                for i in range(len(face_bbox)):
                    face_name = face_bbox[i][4]
                    # Calculate face area
                    face_area = int(face_bbox[i][2] - face_bbox[i][0]) * int(face_bbox[i][3] - face_bbox[i][1])
                    
                    if face_name == person_to_follow:
                        person_found = True
                        pno += 1
                        total_pno += 1

                        # This calculates the vector from your ROI to the center of the screen
                        center_of_bound_box = np.array(((face_bbox[i][0] + face_bbox[i][2])/2, (face_bbox[i][1] + face_bbox[i][3])/2))
                        vector_target = np.array((int(center_of_bound_box[0]), int(center_of_bound_box[1])))
                        vector_distance = vector_true-vector_target
                        
                        if vector_distance[0] > safety_x:
                            print("Yaw left.")
                            control_disp += "y<- "
                        elif vector_distance[0] < -safety_x:
                            print("Yaw right.")
                            control_disp += "y-> "
                        else:
                            pass
                        
                        if vector_distance[1] > safety_y:
                            print("Fly up.")
                            control_disp += "t^ "
                        elif vector_distance[1] < -safety_y:
                            print("Fly down.")
                            control_disp += "tV "
                        else:
                            pass
                        
                        if face_area < 9000:
                            print("Push forward")
                            control_disp += "p^ "
                        elif face_area > 16000:
                            print("Pull back")
                            control_disp += "pV "
                        else:
                            pass
                        
                        # Print center of bounding box and vector calculations
                        print_out += str(face_area)
                        cv2.circle(frame, (int(center_of_bound_box[0]), int(center_of_bound_box[1])), 5, (0,100,255), 2)
                        # Draw the safety zone
                        cv2.rectangle(frame, (resize_div_2[0] - safety_x, resize_div_2[1] - safety_y), (resize_div_2[0] + safety_x, resize_div_2[1] + safety_y), (0,255,255), 2)
                        
                        # Transfer face to person tracking
                        if total_pno >= 30 and (pno / total_pno) >= 0.5:
                            person_to_track = face_bbox[i][0:4]
                            face_flag = False
                            yolosort = True
                            pno = 0
                            total_pno = 0
                        
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
                
                # Count how many frames until tracked person is lost
                if not person_found:
                    if pno > 0:
                        total_pno += 1
                    if total_pno >= 30 and (pno / total_pno) < 0.5:
                        pno = 0
                        total_pno = 0
            
        # Person tracking
        if yolosort:
            #image = Image.fromarray(frame[...,::-1]) #bgr to rgb
            #boxs = yolo.detect_image(image)
            boxs = model_mnet.detect(frame, 0.7)
            features = encoder(frame,boxs)
            
            # score to 1.0 here).
            detections = [Detection(bbox, 1.0, feature) for bbox, feature in zip(boxs, features)]
            
            # Run non-maxima suppression.
            boxes = np.array([d.tlwh for d in detections])
            scores = np.array([d.confidence for d in detections])
            indices = preprocessing.non_max_suppression(boxes, nms_max_overlap, scores)
            detections = [detections[i] for i in indices]
            #print(boxs)
            
            # Call the tracker
            tracker.predict()
            tracker.update(detections)
            
            for track in tracker.tracks:
                if not track.is_confirmed() or track.time_since_update > 1:
                    print(fno, track.track_id, 'not found.')
                    if auto_engaged and confirmed_number == track.track_id:
                        # Swap back to face detection if person can't be found
                        face_flag = True
                        yolosort = False
                    continue 
                bbox = track.to_tlbr()
                # Only track 1 person
                if person_to_track:
                    if person_to_track[0] > bbox[0] and person_to_track[2] < bbox[2] and person_to_track[3] < bbox[3] and person_to_track[0] < bbox[2] and person_to_track[2] > bbox[0] and person_to_track[3] > bbox[1]:
                        print("Captured.")
                        face_locked = True
                        confirmed_number = track.track_id
                    else:
                        face_locked = False
                        print("Retry capture.")
                        face_flag = True
                        yolosort = False
                person_to_track = None
               
                # Calculate person bounding box area
                person_area = int(bbox[2] - bbox[0]) * int(bbox[3] - bbox[1])
                
                if face_locked:
                    print(fno, track.track_id, confirmed_number == track.track_id)
                    if confirmed_number == track.track_id:
                        if auto_engaged:
                            # This calculates the vector from your ROI to the center of the screen
                            center_of_bound_box = np.array(((bbox[0] + bbox[2])/2, (bbox[1] + bbox[3])/2))
                            vector_target = np.array((int(center_of_bound_box[0]), int(center_of_bound_box[1])))
                            vector_distance = vector_true-vector_target
                            
                            if vector_distance[0] > safety_x_person:
                                print("Yaw left.")
                                control_disp += "y<- "
                            elif vector_distance[0] < -safety_x_person:
                                print("Yaw right.")
                                control_disp += "y-> "
                            else:
                                pass
                            
                            if vector_distance[1] > safety_y_person:
                                print("Fly up.")
                                control_disp += "t^ "
                            elif vector_distance[1] < -safety_y_person:
                                print("Fly down.")
                                control_disp += "tV "
                            else:
                                pass
                            
                            if person_area < 45000:
                                print("Push forward")
                                control_disp += "p^ "
                            elif person_area > 80000:
                                print("Pull back")
                                control_disp += "pV "
                            else:
                                pass
                        
                            # Print center of bounding box and vector calculations
                            print_out += str(confirmed_number) + " "
                            print_out += str(person_area)
                            cv2.circle(frame, (int(center_of_bound_box[0]), int(center_of_bound_box[1])), 5, (0,255,255), 2)
                        # Draw selected bounding box
                        cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,255,255), 2)
                        cv2.putText(frame, "Tracked-" + str(track.track_id),(int(bbox[0]), int(bbox[1]) + 100),0, 5e-3 * 200, (0,255,0),2)
                        # Draw the safety zone
                        cv2.rectangle(frame, (resize_div_2[0] - safety_x_person, resize_div_2[1] - safety_y_person), (resize_div_2[0] + safety_x_person, resize_div_2[1] + safety_y_person), (0,255,255), 2)
                        break
                else:
                    # Draw bounding box and calculate box area
                    cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,255,255), 2)
                    cv2.putText(frame, str(track.track_id) + "," + str(person_area),(int(bbox[0]), int(bbox[1]) + 100),0, 5e-3 * 200, (0,255,0),2)
            
            for det in detections:
                bbox = det.to_tlbr()
                cv2.rectangle(frame,(int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,0,0), 2)
        
        # Show number while typing
        print_out += confirmed_string
        # Draw the center of frame as a circle and autopilot status
        middle_of_frame = (int(resize_div_2[0]), int(resize_div_2[1]))
        cv2.circle(frame, middle_of_frame, 5, (255,128,0), 2)
        cv2.putText(frame, print_out,(0, (frame.shape[0] - 10)),0, 0.8, (0,0,255),2)
        
        # Keypress action
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        if k == ord('t'):
            face_flag = not face_flag
            yolosort = not yolosort
            face_locked = False
            pno = 0
            total_pno = 0
        if k == ord('o'):
            auto_engaged = not auto_engaged
        # Number key for entering ID to track
        num_string = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
        if not face_locked and yolosort:
            if k >= 48 and k <= 57:
                confirmed_string += num_string[k - 48]
            if k == 13 and confirmed_string != '':
                confirmed_number = int(confirmed_string)
                face_locked = True
                confirmed_string = ""
        if k == ord('c'):
            confirmed_string = ""
            face_locked = False
        
        # Show model in use on frame
        cv2.putText(frame, model_in_use,(0, 20),0, 0.8, (255,0,0),2)
        # Draw drone control
        cv2.putText(frame, control_disp,((frame.shape[1] - 150), (frame.shape[0] - 10)),0, 0.8, (0,0,255),2)
        # Scalable window
        cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
        cv2.imshow('Camera', frame)
        
        if writeVideo_flag:
            # save a frame
            out.write(frame)
            frame_index = frame_index + 1
            
        fps  = ( fps + (1./(time.time()-t1)) ) / 2
        print("fps= %f"%(fps))
        #print("frame= %d, bach= %d/%d, person_found= %d" % (fno, pno, total_pno, person_found))
        
    # Exiting
    video_capture.release()
    if writeVideo_flag:
        out.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
