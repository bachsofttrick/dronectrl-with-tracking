#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function, absolute_import

from timeit import time
from time import strftime 
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
from customlibs.VideoGet import VideoGet
from customlibs.dronectrl import Drone

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
    person_to_follow = 'bach'
    face_dettect = Recognizer('resnet10')

    # Flag to choose which model to run
    face_flag = True
    yolosort = False
    
    # Flag to override autopilot
    auto_engaged = False
    # This is for controlling altitude manually
    auto_throttle = True
    
    # Transfer to person tracking
    face_to_track = None
    face_locked = False
    confirmed_number = 0
        
    # Open stream
    #video_capture = VideoGet("http://192.168.43.99:8080/video").start()
    video_capture = VideoGet("rtsp://192.168.100.1/encavc0-stream").start()
    #video_capture = VideoGet('0').start()
    
    # Enter drone and control speed
    do_you_have_drone = True
    velocity = 30
    velocity2 = 100
    if do_you_have_drone:
        dm107s = Drone().start()
    
    # Enter safety zone coordinate
    safety_x = 100
    safety_y = 100
    
    writeVideo_flag = True 
    if writeVideo_flag:
    # Define the codec and create VideoWriter object
        w = 1280
        h = 720
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        localtime = strftime("m%md%d-%H%M%S")
        out = cv2.VideoWriter('output %s.avi' % localtime, fourcc, 15, (w, h))
        frame_index = -1 

    fps = 0.0
    f_drop = 0

    while True:
        ret, frame = video_capture.update()
        if ret != True:
            f_drop += 1
            if f_drop > 10:
                print("Dropped frames.")
                break
        else:
            f_drop = 0
        t1 = time.time()        
        #frame = cv2.flip(frame, 1)
        
        # Resize frame
        resize_to = (1280, 720)
        resize_div_2 = (int(resize_to[0]/2), int(resize_to[1]/2))
        #frame = cv2.resize(frame, resize_to)

        # Show control on the right corner of frame
        control_disp = "" 
        
        # Show autopilot status
        if auto_engaged:
            print_out = "AUTOPILOT "
        else:
            print_out = "MANUAL "
        
        # Face recognizer
        vector_true = np.array((resize_div_2[0], resize_div_2[1], 16000))
        if face_flag:
            face_bbox = face_dettect.recognize(frame, person_to_follow)
            # Prevent autopilot when there is no face detected
            if len(face_bbox) == 0:
                if auto_engaged:
                    dm107s.yaw = 128
                    dm107s.pitch = 128
                    if auto_throttle:
                        dm107s.throttle = 128
            else:
                for i in range(len(face_bbox)):
                    face_name = face_bbox[i][4]
                    if auto_engaged:
                        if face_name == person_to_follow:
                            face_to_track = face_bbox[i][0:4]
                            
                            # This calculates the vector from your ROI to the center of the screen
                            center_of_bound_box = np.array(((face_bbox[i][0] + face_bbox[i][2])/2, (face_bbox[i][1] + face_bbox[i][3])/2))
                            vector_target = np.array((int(center_of_bound_box[0]), int(center_of_bound_box[1]), int(face_bbox[i][2] - face_bbox[i][0]) * int(face_bbox[i][3] - face_bbox[i][1])))
                            vector_distance = vector_true-vector_target
                            
                            if vector_distance[0] < -safety_x:
                                print("Yaw left.")
                                control_disp += "y<- "
                                dm107s.yaw = 128 + velocity
                            elif vector_distance[0] > safety_x:
                                print("Yaw right.")
                                control_disp += "y-> "
                                dm107s.yaw = 128 - velocity
                            else:
                                dm107s.yaw = 128
                            
                            if vector_distance[1] > safety_y:
                                print("Fly up.")
                                control_disp += "t^ "
                                if auto_throttle:
                                    dm107s.throttle = 128 + 15
                            elif vector_distance[1] < -safety_y:
                                print("Fly down.")
                                control_disp += "tV "
                                if auto_throttle:
                                    dm107s.throttle = 128 - 70
                            else:
                                if auto_throttle:
                                    dm107s.throttle = 128
                            
                            if vector_distance[2] > 7000:
                                print("Push forward")
                                control_disp += "p^ "
                                dm107s.pitch = 128 + velocity
                            elif vector_distance[2] < 0:
                                print("Pull back")
                                control_disp += "pV "
                                dm107s.pitch = 128 - velocity - 5
                            else:
                                dm107s.pitch = 128
                        
                            # Print center of bounding box and vector calculations
                            #print_out += str(int(vector_distance[0])) + " " + str(int(vector_distance[1])) + " " + str(int(vector_distance[2]))
                            print_out += str(int(vector_distance[2]))
                            cv2.circle(frame, (int(center_of_bound_box[0]), int(center_of_bound_box[1])), 5, (0,100,255), 2)
                            # Draw the safety zone
                            cv2.rectangle(frame, (resize_div_2[0] - safety_x, resize_div_2[1] - safety_y), (resize_div_2[0] + safety_x, resize_div_2[1] + safety_y), (0,255,255), 2)
                        
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
                # Only track 1 person (WIP)
                vector_true[2] *= 3
                if face_to_track:
                    number_of_true = 0
                    number_of_true = (number_of_true + 1) if face_to_track[0] >= bbox[0] else number_of_true
                    number_of_true = (number_of_true + 1) if face_to_track[1] >= bbox[1] else number_of_true
                    number_of_true = (number_of_true + 1) if face_to_track[2] <= bbox[2] else number_of_true
                    number_of_true = (number_of_true + 1) if face_to_track[3] < bbox[3] else number_of_true
                    if number_of_true == 4:
                        print("Captured.")
                        face_locked = True
                        confirmed_number = track.track_id
                    else:
                        face_locked = False
                        print("Retry capture.")
                    face_to_track = None
                if face_locked:
                    if confirmed_number == track.track_id:
                        # This calculates the vector from your ROI to the center of the screen
                        center_of_bound_box = np.array(((bbox[0] + bbox[2])/2, (bbox[1] + bbox[3])/2))
                        vector_target = np.array((int(center_of_bound_box[0]), int(center_of_bound_box[1]), int(bbox[2] - bbox[0]) * int(bbox[3] - bbox[1])))
                        vector_distance = vector_true-vector_target
                        
                        if auto_engaged:
                            if vector_distance[0] < -safety_x*2:
                                print("Yaw left.")
                            elif vector_distance[0] > safety_x*2:
                                print("Yaw right.")
                            else:
                                pass
                            
                            if vector_distance[1] > safety_y*2:
                                print("Fly up.")
                            elif vector_distance[1] < -safety_y*2:
                                print("Fly down.")
                            else:
                                pass
                            
                            if vector_distance[2] > 50000:
                                print("Push forward")
                            elif vector_distance[2] < -50000:
                                print("Pull back")
                            else:
                                pass
                        
                        # Print center of bounding box and vector calculations
                        #print_out += str(int(vector_distance[0])) + " " + str(int(vector_distance[1])) + " " + str(int(vector_distance[2]))
                        print_out += str(int(vector_distance[2]))
                        cv2.circle(frame, (int(center_of_bound_box[0]), int(center_of_bound_box[1])), 5, (0,0,255), 2)
                        # Draw selected bounding box
                        cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,255,255), 2)
                        cv2.putText(frame, person_to_follow + "-" + str(track.track_id),(int(bbox[0]), int(bbox[1])),0, 5e-3 * 200, (0,255,0),2)
                        # Draw the safety zone
                        cv2.rectangle(frame, (resize_div_2[0] - safety_x, resize_div_2[1] - safety_y), (resize_div_2[0] + safety_x, resize_div_2[1] + safety_y), (0,255,255), 2)
                        break
                else:
                    # This calculates the vector from your ROI to the center of the screen
                    center_of_bound_box = np.array(((bbox[0] + bbox[2])/2, (bbox[1] + bbox[3])/2))
                    vector_target = np.array((int(center_of_bound_box[0]), int(center_of_bound_box[1]), int(bbox[2] - bbox[0]) * int(bbox[3] - bbox[1])))
                    vector_distance = vector_true-vector_target
                    # Draw bounding box and calculate box area
                    cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,255,255), 2)
                    cv2.putText(frame, str(track.track_id) + "," + str(int(vector_distance[2]+25000)),(int(bbox[0]), int(bbox[1])),0, 5e-3 * 200, (0,255,0),2)
                
            for det in detections:
                bbox = det.to_tlbr()
                cv2.rectangle(frame,(int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,0,0), 2)
        
        # Draw the center of frame as a circle and autopilot status
        middle_of_frame = (int(resize_div_2[0]), int(resize_div_2[1]))
        cv2.circle(frame, middle_of_frame, 5, (255,128,0), 2)
        cv2.putText(frame, print_out,(0, (frame.shape[0] - 10)),0, 0.8, (0,0,255),2)
        
        # Keypress action
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        if k == ord('r'):
            face_flag = not face_flag
            yolosort = not yolosort
            face_locked = False
            # Reset control to prevent moving when switching model
            auto_engaged = False
            dm107s.default()
        if do_you_have_drone:
            # Control drone
            # Override autopilot
            if k == ord('o'):
                auto_engaged = not auto_engaged
			
            # Takeoff and landing
            if k == ord('t'):
                dm107s.takeoff()

            # Throttle
            if not auto_throttle:
                if k == ord('w'):
                    #dm107s.throttle_up()
                    control_disp += "t^ "
                    dm107s.incremt(0,0,velocity2,0)
                elif k == ord('s'):
                    #dm107s.throttle_dwn()
                    control_disp += "tV "
                    dm107s.incremt(0,0,-velocity2,0)
                if k == ord('f'):
                    dm107s.incremt(0,0,0,0)
            
            if not auto_engaged:
                # Throttle
                if k == ord('w'):
                    #dm107s.throttle_up()
                    dm107s.incremt(0,0,velocity2,0)
                elif k == ord('s'):
                    #dm107s.throttle_dwn()
                    dm107s.incremt(0,0,-velocity2,0)
                    
                # Yaw
                if k == ord('a'):
                    #dm107s.yaw_left()
                    control_disp += "y<- "
                    dm107s.incremt(0,0,0,velocity2)
                elif k == ord('d'):
                    #dm107s.yaw_right()
                    control_disp += "y-> "
                    dm107s.incremt(0,0,0,-velocity2)

                # Pitch
                if k == ord('i'):
                    #dm107s.pitch_fwd()
                    control_disp += "p^ "
                    dm107s.incremt(0,velocity2,0,0)
                elif k == ord('k'):
                    #dm107s.pitch_bwd()
                    control_disp += "pV "
                    dm107s.incremt(0,-velocity2,0,0)

                # Roll
                if k == ord('j'):
                    #dm107s.roll_left()
                    control_disp += "r<- "
                    dm107s.incremt(-velocity2,0,0,0)
                elif k == ord('l'):
                    #dm107s.roll_right()
                    control_disp += "r-> "
                    dm107s.incremt(velocity2,0,0,0)
                
                if k == ord('f'):
                    dm107s.incremt(0,0,0,0)

            # STOP NOW
            if k == ord('e'):
                control_disp = "STOP!"
                dm107s.emergency_stop()
            
            # Calibrate gyro
            if k == ord('c'):
                control_disp = "Calibrate..."
                dm107s.calib_gyro()
        
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
        
    # Exiting
    video_capture.stop()
    if do_you_have_drone:
        dm107s.close_connection()
    if writeVideo_flag:
        out.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
