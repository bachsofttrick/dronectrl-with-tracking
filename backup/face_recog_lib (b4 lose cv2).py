import tensorflow as tf
import facenet
import pickle
import align.detect_face
import numpy as np
import cv2
import yolov2face

class Recognizer:
    def __init__(self, option):
        # Setup necessary values
        self.option = option
        self.bach_count = 0
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
            self.model_face = yolov2face.load_model('Models/yolov2_tiny-face.h5')
            print('Yolov2_tiny-face loaded.')

        # Get input and output tensors
        self.images_placeholder = tf.get_default_graph().get_tensor_by_name("input:0")
        self.embeddings = tf.get_default_graph().get_tensor_by_name("embeddings:0")
        self.phase_train_placeholder = tf.get_default_graph().get_tensor_by_name("phase_train:0")
        self.embedding_size = self.embeddings.get_shape()[1]

    def recognize(self, frame):
        # Detect face, return bounding_boxes
        if self.option == 'mtcnn':
            self.bounding_boxes, _ = align.detect_face.detect_face(frame, self.MINSIZE, self.pnet, self.rnet, self.onet, self.THRESHOLD, self.FACTOR)
        elif self.option == 'yolov2':
            self.bounding_boxes = yolov2face.show_results(frame, self.model_face)
        self.faces_found = len(self.bounding_boxes)

        try:
            if self.faces_found > 0:
                self.det = self.bounding_boxes
                self.bb = np.zeros((self.faces_found, 4), dtype=np.int32)
                for i in range(self.faces_found):
                    self.bb[i][0] = self.det[i][0]
                    self.bb[i][1] = self.det[i][1]
                    self.bb[i][2] = self.det[i][2]
                    self.bb[i][3] = self.det[i][3]
                    # Crop face out
                    #if (self.bb[i][3]-self.bb[i][1])/frame.shape[0]>0.25:
                    if (self.bb[i][3]-self.bb[i][1])/frame.shape[0]>0.1:
                        self.cropped = frame[self.bb[i][1]:self.bb[i][3], self.bb[i][0]:self.bb[i][2], :]
                        self.scaled = cv2.resize(self.cropped, (self.INPUT_IMAGE_SIZE, self.INPUT_IMAGE_SIZE),
                                            interpolation=cv2.INTER_CUBIC)
                        #cv2.imshow('face', self.scaled)
                        self.scaled = facenet.prewhiten(self.scaled)
                        self.scaled_reshape = self.scaled.reshape(-1, self.INPUT_IMAGE_SIZE, self.INPUT_IMAGE_SIZE, 3)
                        self.feed_dict = {self.images_placeholder: self.scaled_reshape, self.phase_train_placeholder: False}
                        self.emb_array = self.sess.run(self.embeddings, feed_dict=self.feed_dict)

                        # Send embedded array in SVM to classify
                        self.predictions = self.model.predict_proba(self.emb_array)
                        self.best_class_indices = np.argmax(self.predictions, axis=1)
                        self.best_class_probabilities = self.predictions[
                            np.arange(len(self.best_class_indices)), self.best_class_indices]

                        # Select class name with the highest percent
                        self.best_name = self.class_names[self.best_class_indices[0]]

                        # 
                        self.bach_count = (self.bach_count + 1) if (self.best_name == 'bach' and self.best_class_probabilities > 0.9) else self.bach_count
                        #print(self.bach_count)
                        print("Name: {}, Probability: {}".format(self.best_name, self.best_class_probabilities))

                        # If probability > (a certain value) then display name
                        #if self.best_class_probabilities > 0.8:
                        if self.best_class_probabilities > 0.7:
                            # Draw bounding box over face
                            cv2.rectangle(frame, (self.bb[i][0], self.bb[i][1]), (self.bb[i][2], self.bb[i][3]), (0, 255, 0), 2)
                            self.text_x = self.bb[i][0]
                            self.text_y = self.bb[i][3] + 20

                            # Write name on frame
                            self.name = self.class_names[self.best_class_indices[0]]
                            cv2.putText(frame, str(self.name), (self.text_x, self.text_y + 17*2), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                        1, (255, 255, 255), thickness=1, lineType=2)
                            cv2.putText(frame, str(self.bb[i]), (self.text_x, self.text_y), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                        1, (255, 255, 255), thickness=1, lineType=2)
                            cv2.putText(frame, str(round(self.best_class_probabilities[0], 3)), (self.text_x, self.text_y + 17),
                                        cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                        1, (255, 255, 255), thickness=1, lineType=2)
        except:
            pass
        
