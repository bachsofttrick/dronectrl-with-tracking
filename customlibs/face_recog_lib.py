import tensorflow as tf
from . import facenet
import pickle
import align.detect_face
import numpy as np
import cv2
from . import yolov2face
from .resnet_lib import Resnetdnn

class Recognizer:
    def __init__(self, option):
        # Setup necessary values
        self.option = option
        self.MINSIZE = 20
        self.THRESHOLD = [0.6, 0.7, 0.7]
        self.FACTOR = 0.709
        self.INPUT_IMAGE_SIZE = 160
        self.CLASSIFIER_PATH = 'Models/facemodel.pkl'
        self.FACENET_MODEL_PATH = 'Models/20180402-114759.pb'

        # Load The Custom Classifier
        with open(self.CLASSIFIER_PATH, 'rb') as infile:
            self.model, self.class_names = pickle.load(infile)
        print("Custom Classifier, Successfully loaded.")

        # [!]per_process_gpu_memory_fraction only work once
        #self.gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.2)
        #self.sess = tf.Session(config=tf.ConfigProto(gpu_options=self.gpu_options, log_device_placement=False))
        self.sess = tf.Session()

        # Load the model
        print('Loading feature extraction model')
        facenet.load_model(self.FACENET_MODEL_PATH)
        print('Facenet loaded.')
        if option == 'mtcnn':
            self.pnet, self.rnet, self.onet = align.detect_face.create_mtcnn(self.sess, "align")
            print('MTCNN loaded.')
        elif option == 'yolov2':
            self.model_yface = yolov2face.load_model('Models/yolov2_tiny-face.h5')
            print('Yolov2_tiny-face loaded.')
        elif option == 'resnet10':
            self.model_rnet = Resnetdnn()
            print('Resnet10 loaded.')

        # Get input and output tensors
        self.images_placeholder = tf.get_default_graph().get_tensor_by_name("input:0")
        self.embeddings = tf.get_default_graph().get_tensor_by_name("embeddings:0")
        self.phase_train_placeholder = tf.get_default_graph().get_tensor_by_name("phase_train:0")
        self.embedding_size = self.embeddings.get_shape()[1]

    def recognize(self, frame):
        # Detect face, return bounding_boxes
        if self.option == 'mtcnn':
            bounding_boxes, _ = align.detect_face.detect_face(frame, self.MINSIZE, self.pnet, self.rnet, self.onet, self.THRESHOLD, self.FACTOR)
        elif self.option == 'yolov2':
            bounding_boxes = yolov2face.show_results(frame, self.model_yface)
        elif self.option == 'resnet10':
            bounding_boxes = self.model_rnet.detect(frame)
        faces_found = len(bounding_boxes)
        bbox_result = []
        bach_count = 0

        try:
            if faces_found > 0:
                det = bounding_boxes
                bb = np.zeros((faces_found, 4), dtype=np.int32)
                for i in range(faces_found):
                    bb[i][0] = det[i][0]
                    bb[i][1] = det[i][1]
                    bb[i][2] = det[i][2]
                    bb[i][3] = det[i][3]
                    # Crop face out
                    #if (bb[i][3]-bb[i][1])/frame.shape[0]>0.25:
                    if (bb[i][3]-bb[i][1])/frame.shape[0]>0.1:
                        cropped = frame[bb[i][1]:bb[i][3], bb[i][0]:bb[i][2], :]
                        scaled = cv2.resize(cropped, (self.INPUT_IMAGE_SIZE, self.INPUT_IMAGE_SIZE),
                                            interpolation=cv2.INTER_CUBIC)
                        #cv2.imshow('face', scaled)
                        scaled = facenet.prewhiten(scaled)
                        scaled_reshape = scaled.reshape(-1, self.INPUT_IMAGE_SIZE, self.INPUT_IMAGE_SIZE, 3)
                        feed_dict = {self.images_placeholder: scaled_reshape, self.phase_train_placeholder: False}
                        emb_array = self.sess.run(self.embeddings, feed_dict=feed_dict)

                        # Send embedded array in SVM to classify
                        predictions = self.model.predict_proba(emb_array)
                        best_class_indices = np.argmax(predictions, axis=1)
                        best_class_probabilities = predictions[
                            np.arange(len(best_class_indices)), best_class_indices]

                        # Select class name with the highest percent
                        best_name = self.class_names[best_class_indices[0]]

                        # 
                        bach_count = (bach_count + 1) if (best_name == 'bach' and best_class_probabilities > 0.7) else bach_count
                        #print(bach_count)
                        #print("Name: {}, Probability: {}".format(best_name, best_class_probabilities))

                        # If probability > (a certain value) then add to resulting bbox
                        #if best_class_probabilities > 0.8:
                        if best_class_probabilities > 0.7:
                            if bach_count > 1 and best_name == 'bach':
                                print('Lose 1 bach!')
                                pass
                            else:
                                bbox_result.append([bb[i][0], bb[i][1], bb[i][2], bb[i][3], best_name, best_class_probabilities])
        except:
            pass
        
        return bbox_result
